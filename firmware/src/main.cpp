// Spin coater firmware
// ESP32 + A2212 1400KV + SimonK 30A ESC + reflective IR tach + OLED + 4x4 keypad
//
// Menu:    1 MANUAL RPM   2 PROGRAM   3 RAW TEST   4 UPDATE FW      (A/B move, C or # select)
// Manual:  type RPM, # run.  While running: A/B +-100, digits + # set a new RPM, D stop
// Program: up to 8 steps (rpm, ramp s, hold s) + eased ramp down. # runs it.
//          While running: A skips to the next step, D or * stops
// Raw test: A/B +-10us   C/D +-1us   0 = 1000us   * exit
// * always stops the motor.

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>
#include <Keypad.h>
#include "config.h"
#include "tach.h"
#include "program.h"
#include "ota.h"

static const byte KEY_MAP[4][4] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
static byte rowPins[4] = { KEY_ROW_PINS[0], KEY_ROW_PINS[1], KEY_ROW_PINS[2], KEY_ROW_PINS[3] };
static byte colPins[4] = { KEY_COL_PINS[0], KEY_COL_PINS[1], KEY_COL_PINS[2], KEY_COL_PINS[3] };
static Keypad keys = Keypad(makeKeymap(KEY_MAP), rowPins, colPins, 4, 4);

static Adafruit_SSD1306 display(128, 64, &Wire, -1);
static Servo esc;

enum State : uint8_t { S_MENU, S_MANUAL, S_SEEK, S_RUN, S_TEST, S_FAULT };
enum RunKind : uint8_t { RUN_MANUAL, RUN_PROGRAM };
static State state = S_MENU;
static RunKind runKind = RUN_MANUAL;
static const char* faultMsg = "";
static uint8_t menuIdx = 0;
static uint32_t doneUntilMs = 0;

static int   target = 1000;      // typed / displayed manual target
static int   runTarget = 1000;   // manual target the controller uses
static bool  typing = false;
static float sp = 0;             // setpoint rpm
static float us = ESC_MIN_US;    // current throttle pulse
static int   rpm = 0;            // smoothed tach rpm
static uint32_t lastGoodMs = 0, runStartMs = 0;
static uint8_t seekOk = 0;       // consecutive ticks at/above SEEK_HANDOFF_RPM
static float prevSp = 0;         // setpoint last tick (for feedforward)
static uint32_t seekHoldUntil = 0;
static uint8_t restarts = 0;
static int handUs = 0, peakRpm = 0;   // debug: throttle at hand-over, peak tach in first 5 s

// program run state
static uint8_t progIdx0 = 0;     // step the current timeline starts from
static uint32_t progT0 = 0;
static float progStartRpm = 0;
static ProgPos pos;

static void writeUs(float v, float hi = ESC_SAFE_US) {
  if (v < ESC_MIN_US) v = ESC_MIN_US;
  if (v > hi) v = hi;
  us = v;
  esc.writeMicroseconds((int)v);
}

static void stopMotor() {
  writeUs(ESC_MIN_US);
  sp = 0;
  state = S_MENU;
}

static void finishRun() {
  stopMotor();
  doneUntilMs = millis() + 6000;
}

static void fault(const char* msg) {
  writeUs(ESC_MIN_US);
  sp = 0;
  faultMsg = msg;
  state = S_FAULT;
}

static void startRun(RunKind kind) {
  runKind = kind;
  if (kind == RUN_MANUAL) {
    runTarget = constrain(target, RPM_MIN, RPM_MAX);
    target = runTarget;
  }
  tachReset();
  rpm = 0; sp = 0;
  pos = ProgPos();
  writeUs(SEEK_START_US);
  runStartMs = lastGoodMs = millis();
  seekOk = 0;
  restarts = 0;
  seekHoldUntil = 0;
  handUs = 0;
  peakRpm = 0;
  state = S_SEEK;
  Serial.println("t_ms,setpoint,rpm,us");
}

// ---- control tick (every TICK_MS) ----
static void control(uint32_t now) {
  bool fresh = tachRead(rpm);

  if (state == S_SEEK) {
    if (now < seekHoldUntil) { writeUs(ESC_MIN_US); return; }   // restart pause at idle
    // Creep up until the tach sees the rotor turning, then hand over.
    seekOk = (rpm >= SEEK_HANDOFF_RPM) ? seekOk + 1 : 0;
    if (seekOk >= 2) {
      writeUs(us - CATCH_DROP_US);       // start throttle is higher than hold throttle: pull back
      handUs = (int)us;
      sp = rpm;
      prevSp = sp;
      lastGoodMs = now;
      progIdx0 = 0;
      progT0 = now;
      progStartRpm = rpm;
      state = S_RUN;
    } else {
      writeUs(us < SEEK_START_US ? SEEK_START_US : us + SEEK_STEP_US);
      if (us > SEEK_MAX_US) fault("NO SPIN (tach/ESC?)");
    }
  } else if (state == S_RUN) {
    if (rpm > 0) lastGoodMs = now;
    if (now - progT0 < 5000UL && rpm > peakRpm) peakRpm = rpm;
    if (rpm > RPM_MAX + 500) { fault("OVERSPEED"); return; }

    if (runKind == RUN_PROGRAM) {
      pos = progAt(prog, progIdx0, progStartRpm, now - progT0);
      if (pos.phase == PH_DONE) { finishRun(); return; }
      sp = pos.sp;
      // End ramp: below the minimum stable speed the motor just coasts to a stop.
      if (pos.phase == PH_END && (sp < RPM_MIN || rpm == 0)) { finishRun(); return; }
    } else {
      float maxd = SLEW_RPM_S * TICK_MS / 1000.0f;
      float d = (float)runTarget - sp;
      if (d > maxd) d = maxd;
      if (d < -maxd) d = -maxd;
      sp += d;
    }

    if (now - lastGoodMs > STALL_MS) {
      if (restarts < START_RETRIES && now - progT0 < 20000UL) {   // early stall: retry the start
        restarts++;
        writeUs(ESC_MIN_US);
        sp = 0;
        tachReset();
        seekOk = 0;
        seekHoldUntil = now + 1500;
        state = S_SEEK;
      } else {
        fault("STALL / TACH LOST");
      }
      return;
    }

    // Feedforward: move the throttle with the setpoint so ramps don't depend on
    // the slow trim loop (which used to hover too low and stall).
    writeUs(us + (sp - prevSp) / FF_RPM_PER_US);
    prevSp = sp;

    if (fresh && rpm > 0) {              // only act on fresh tach data
      float step = KP_US_PER_RPM * (sp - (float)rpm);
      if (step > STEP_MAX_US) step = STEP_MAX_US;
      if (step < -STEP_DOWN_MAX_US) step = -STEP_DOWN_MAX_US;
      writeUs(us + step);
    }
  }

  if (state == S_SEEK || state == S_RUN) {
    Serial.printf("%lu,%d,%d,%d\n", (unsigned long)(now - runStartMs), (int)sp, rpm, (int)us);
  }
}

// ---- blocking number editor (motor is off whenever this is used) ----
static bool editNumber(int &val, int lo, int hi, int step, const char* lbl) {
  int v = val;
  bool typed = false;
  while (true) {
    display.clearDisplay(); display.setCursor(0, 0);
    display.setTextSize(1); display.printf("%s (%d-%d)\n", lbl, lo, hi);
    display.setTextSize(2); display.printf("%d\n", v);
    display.setTextSize(1);
    display.println();
    display.println("0-9:type  #:del");
    display.printf("A/B: +/-%d\n", step);
    display.println("C:OK  D:cancel");
    display.display();

    char k;
    while ((k = keys.getKey()) == NO_KEY) delay(10);
    if (k >= '0' && k <= '9') {
      if (!typed) { v = 0; typed = true; }
      if (v < 100000) v = v * 10 + (k - '0');
    } else if (k == '#') {
      if (typed) v /= 10;
    } else if (k == 'A') {
      typed = false; v = constrain(v, lo, hi); v = min(hi, v + step);
    } else if (k == 'B') {
      typed = false; v = constrain(v, lo, hi); v = max(lo, v - step);
    } else if (k == 'C') {
      if (v < lo || v > hi) { v = constrain(v, lo, hi); typed = false; continue; }  // C again to accept
      val = v;
      return true;
    } else if (k == 'D') {
      return false;
    }
  }
}

// ---- program editor. Returns true if the user pressed # to run ----
static void progDraw(int cur, int rows) {
  display.clearDisplay(); display.setTextSize(1); display.setCursor(0, 0);
  display.printf("PROGRAM  total %lus\n", (unsigned long)progTotalS(prog));
  int top = cur < 5 ? 0 : cur - 4;
  for (int r = top; r < top + 5 && r < rows; r++) {
    char b[24];
    if (r < prog.n) snprintf(b, sizeof b, "%d %4d r%d h%d", r + 1, prog.s[r].rpm, prog.s[r].rampS, prog.s[r].holdS);
    else if (r == prog.n) snprintf(b, sizeof b, "END ramp %ds", prog.endRampS);
    else if (r == prog.n + 1) snprintf(b, sizeof b, "EASE %s", easeName(prog.ease));
    else snprintf(b, sizeof b, "+ add step");
    display.print(r == cur ? '>' : ' ');
    display.println(b);
  }
  display.setCursor(0, 48);
  display.println("A/B move C edit 7 del");
  display.println("#:RUN  D:back");
  display.display();
}

static bool progEditor() {
  int cur = 0;
  while (true) {
    int rows = prog.n + 2 + (prog.n < MAX_STEPS ? 1 : 0);
    if (cur >= rows) cur = rows - 1;
    progDraw(cur, rows);

    char k;
    while ((k = keys.getKey()) == NO_KEY) delay(10);
    if (k == 'A') cur = (cur + rows - 1) % rows;
    else if (k == 'B') cur = (cur + 1) % rows;
    else if (k == 'D') return false;
    else if (k == '#') return true;
    else if (k == '7') {                              // delete step
      if (cur < prog.n && prog.n > 1) {
        for (int i = cur; i < prog.n - 1; i++) prog.s[i] = prog.s[i + 1];
        prog.n--;
        progSave();
      }
    } else if (k == 'C') {
      if (cur < prog.n) {
        int r = prog.s[cur].rpm, u = prog.s[cur].rampS, h = prog.s[cur].holdS;
        if (editNumber(r, RPM_MIN, RPM_MAX, 100, "Step RPM") &&
            editNumber(u, 1, 300, 1, "Ramp s") &&
            editNumber(h, 0, 3600, 5, "Hold s")) {
          prog.s[cur].rpm = r; prog.s[cur].rampS = u; prog.s[cur].holdS = h;
          progSave();
        }
      } else if (cur == prog.n) {
        int e = prog.endRampS;
        if (editNumber(e, 1, 300, 1, "End ramp s")) { prog.endRampS = e; progSave(); }
      } else if (cur == prog.n + 1) {
        prog.ease = (prog.ease + 1) % 4;
        progSave();
      } else {                                        // add: copy of the last step
        prog.s[prog.n] = prog.s[prog.n - 1];
        prog.n++;
        cur = prog.n - 1;
        progSave();
      }
    }
  }
}

// ---- keys ----
static void menuSelect() {
  switch (menuIdx) {
    case 0: typing = false; state = S_MANUAL; break;
    case 1: if (progEditor()) startRun(RUN_PROGRAM); break;
    case 2: writeUs(ESC_MIN_US); state = S_TEST; break;
    case 3: otaUpdate(display, keys); break;           // motor is off here
  }
}

static void skipStep() {
  if (state != S_RUN || runKind != RUN_PROGRAM) return;
  if (pos.idx < prog.n) {                              // rebase from the current speed
    progIdx0 = pos.idx + 1;
    progT0 = millis();
    progStartRpm = sp;
  }
}

static void handleKey(char k) {
  if (k == '*') {                                      // STOP always works
    if (state == S_SEEK || state == S_RUN || state == S_TEST) stopMotor();
    return;
  }

  if (state == S_FAULT) {
    if ((k == 'D' || k == '#') && rpm == 0) state = S_MENU;
    return;
  }

  if (state == S_TEST) {
    if (k == 'A') writeUs(us + 10, TEST_MAX_US);
    else if (k == 'B') writeUs(us - 10, TEST_MAX_US);
    else if (k == 'C') writeUs(us + 1, TEST_MAX_US);
    else if (k == 'D') writeUs(us - 1, TEST_MAX_US);
    else if (k == '0') writeUs(ESC_MIN_US);
    return;
  }

  if (state == S_MENU) {
    if (k >= '1' && k <= '4') { menuIdx = k - '1'; menuSelect(); }
    else if (k == 'A') menuIdx = (menuIdx + 3) % 4;
    else if (k == 'B') menuIdx = (menuIdx + 1) % 4;
    else if (k == 'C' || k == '#') menuSelect();
    return;
  }

  bool manual = (state == S_MANUAL) || runKind == RUN_MANUAL;
  if (manual) {
    if (k >= '0' && k <= '9') {
      if (!typing) { target = 0; typing = true; }
      if (target < 10000) target = target * 10 + (k - '0');
    } else if (k == 'A' || k == 'B') {
      typing = false;
      target = constrain(target + (k == 'A' ? 100 : -100), RPM_MIN, RPM_MAX);
      if (state != S_MANUAL) runTarget = target;
    } else if (k == '#') {
      typing = false;
      target = constrain(target, RPM_MIN, RPM_MAX);
      if (state == S_MANUAL) startRun(RUN_MANUAL);
      else runTarget = target;
    } else if (k == 'D') {
      if (state == S_MANUAL) state = S_MENU;
      else stopMotor();
    }
  } else {                                             // program running
    if (k == 'A') skipStep();
    else if (k == 'D') stopMotor();
  }
}

// ---- display ----
static void draw() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  switch (state) {
    case S_MENU: {
      static const char* rows[4] = {"1 MANUAL RPM", "2 PROGRAM", "3 RAW TEST", "4 UPDATE FW"};
      display.printf("SPIN COATER %s\n", FW_VERSION);
      for (uint8_t i = 0; i < 4; i++) {
        display.print(i == menuIdx ? '>' : ' ');
        display.println(rows[i]);
      }
      display.printf("Tach: %d rpm\n", rpm);
      display.println(millis() < doneUntilMs ? "Program DONE" : "");
      display.println("A/B:move C:select");
      break;
    }
    case S_MANUAL:
      display.println("MANUAL");
      display.printf("Set : %d rpm%s\n", target, typing ? "_" : "");
      display.printf("Tach: %d rpm\n", rpm);
      display.printf("Range %d-%d\n", RPM_MIN, RPM_MAX);
      display.println();
      display.println("0-9 type  A/B +-100");
      display.println("#:RUN  D:back");
      break;
    case S_SEEK:
      display.printf("STARTING... try %d\n", restarts + 1);
      display.printf("ESC : %d us\n", (int)us);
      display.printf("Tach: %d rpm\n", rpm);
      display.println();
      display.println("D/*:STOP");
      break;
    case S_RUN:
      if (runKind == RUN_MANUAL) {
        display.println("RUNNING (manual)");
        display.printf("Set : %d rpm\n", runTarget);
        display.printf("Ramp: %d rpm\n", (int)sp);
        display.printf("Tach: %d rpm\n", rpm);
        display.printf("ESC : %d us\n", (int)us);
        display.println();
        display.println("A/B:+-100 #:apply");
        display.println("D/*:STOP");
      } else {
        if (pos.idx >= prog.n) display.println("PROGRAM  END RAMP");
        else display.printf("PROG step %d/%d %s\n", pos.idx + 1, prog.n, pos.phase == PH_RAMP ? "RAMP" : "HOLD");
        display.printf("Set : %d rpm\n", (int)sp);
        display.printf("Tach: %d rpm\n", rpm);
        display.printf("ESC : %d us\n", (int)us);
        display.printf("Phase: %lus left\n", (unsigned long)(pos.phaseLeftMs / 1000));
        unsigned long tl = pos.totalLeftMs / 1000;
        display.printf("Total: %lu:%02lu left\n", tl / 60, tl % 60);
        display.printf("H:%d pk:%d\n", handUs, peakRpm);
        display.println("A:skip step D:STOP");
      }
      break;
    case S_TEST:
      display.println("RAW THROTTLE TEST");
      display.setTextSize(2);
      display.printf("%d us\n", (int)us);
      display.setTextSize(1);
      display.printf("Tach: %d rpm\n", rpm);
      display.printf("Pulses %lu Rej %lu\n", (unsigned long)tachCount, (unsigned long)tachRejected);
      display.println("A/B +-10  C/D +-1");
      display.println("0:1000us  *:exit");
      break;
    case S_FAULT:
      display.println("!! FAULT !!");
      display.println(faultMsg);
      display.println();
      display.println("Motor output cut.");
      display.println(rpm > 0 ? "Wait: rotor turning" : "D/#: clear");
      break;
  }
  display.display();
}

void setup() {
  Serial.begin(115200);

  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    pinMode(2, OUTPUT);
    while (1) { digitalWrite(2, millis() & 0x200); delay(200); }
  }
  display.setTextColor(SSD1306_WHITE);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Arming ESC...");
  display.display();

  esc.setPeriodHertz(50);
  esc.attach(ESC_PIN, ESC_MIN_US, ESC_MAX_US);
  esc.setTimerWidth(16);               // must follow attach(): attach resets width
  esc.writeMicroseconds(ESC_MIN_US);
  delay(ESC_ARM_MS);

  tachBegin();
  progLoad();
  Serial.printf("Spin coater %s ready\n", FW_VERSION);
}

void loop() {
  char k = keys.getKey();
  if (k != NO_KEY) handleKey(k);

  static uint32_t lastTick = 0, lastDraw = 0;
  uint32_t now = millis();
  if (now - lastTick >= TICK_MS) {
    lastTick = now;
    if (state == S_SEEK || state == S_RUN) control(now);
    else tachRead(rpm);
  }
  if (now - lastDraw >= 120) {
    lastDraw = now;
    draw();
  }
}