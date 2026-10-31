#pragma once
#include <Preferences.h>
#include "config.h"

// A program = up to MAX_STEPS steps, then an eased ramp down to stop.
// Each step: ramp (s) from the previous speed to this step's RPM, then hold (s).
#define MAX_STEPS  8
#define PROG_MAGIC 0xA8

struct Step { uint16_t rpm, rampS, holdS; };

struct Program {
  uint8_t  magic = PROG_MAGIC;
  uint8_t  n = 2;
  uint8_t  ease = 1;            // 0 Linear, 1 S-curve, 2 EaseOut, 3 EaseIn
  uint16_t endRampS = 10;       // eased ramp down at the end
  Step     s[MAX_STEPS] = { {500, 5, 30}, {1000, 5, 60} };
};

static Program prog;

static float ease01(float t, uint8_t e) {
  if (t <= 0) return 0;
  if (t >= 1) return 1;
  switch (e) {
    case 1: return t * t * (3.0f - 2.0f * t);
    case 2: return 1.0f - (1.0f - t) * (1.0f - t);
    case 3: return t * t;
    default: return t;
  }
}

static const char* easeName(uint8_t e) {
  switch (e) {
    case 1: return "S-Curve";
    case 2: return "EaseOut";
    case 3: return "EaseIn";
    default: return "Linear";
  }
}

enum ProgPhase : uint8_t { PH_RAMP, PH_HOLD, PH_END, PH_DONE };

struct ProgPos {
  float sp = 0;                 // setpoint rpm
  uint8_t idx = 0;              // current step (== n during the end ramp)
  ProgPhase phase = PH_RAMP;
  uint32_t phaseLeftMs = 0;
  uint32_t totalLeftMs = 0;
};

// Setpoint at 'el' ms after starting from step firstIdx, ramping from startRpm.
static ProgPos progAt(const Program &p, uint8_t firstIdx, float startRpm, uint32_t el) {
  ProgPos r;
  uint32_t total = (uint32_t)p.endRampS * 1000UL;
  for (uint8_t i = firstIdx; i < p.n; i++) total += ((uint32_t)p.s[i].rampS + p.s[i].holdS) * 1000UL;
  r.totalLeftMs = el < total ? total - el : 0;

  uint32_t t = el;
  float prev = startRpm;
  for (uint8_t i = firstIdx; i < p.n; i++) {
    uint32_t rm = (uint32_t)p.s[i].rampS * 1000UL;
    uint32_t hm = (uint32_t)p.s[i].holdS * 1000UL;
    r.idx = i;
    if (t < rm) {
      r.phase = PH_RAMP;
      r.sp = prev + ((float)p.s[i].rpm - prev) * ease01((float)t / (float)rm, p.ease);
      r.phaseLeftMs = rm - t;
      return r;
    }
    t -= rm;
    if (t < hm) {
      r.phase = PH_HOLD;
      r.sp = p.s[i].rpm;
      r.phaseLeftMs = hm - t;
      return r;
    }
    t -= hm;
    prev = p.s[i].rpm;
  }
  uint32_t dn = (uint32_t)p.endRampS * 1000UL;
  r.idx = p.n;
  if (t < dn) {
    r.phase = PH_END;
    r.sp = prev * (1.0f - ease01((float)t / (float)dn, p.ease));
    r.phaseLeftMs = dn - t;
    return r;
  }
  r.phase = PH_DONE;
  r.sp = 0;
  return r;
}

static uint32_t progTotalS(const Program &p) {
  uint32_t t = p.endRampS;
  for (uint8_t i = 0; i < p.n; i++) t += (uint32_t)p.s[i].rampS + p.s[i].holdS;
  return t;
}

static void progSave() {
  Preferences pr;
  if (!pr.begin("prog", false)) return;
  pr.putBytes("p", &prog, sizeof(prog));
  pr.end();
}

static void progLoad() {
  Preferences pr;
  if (!pr.begin("prog", true)) return;       // namespace not created yet -> keep defaults
  if (pr.getBytesLength("p") == sizeof(Program)) {
    Program t;
    pr.getBytes("p", &t, sizeof(t));
    bool ok = t.magic == PROG_MAGIC && t.n >= 1 && t.n <= MAX_STEPS && t.ease < 4 &&
              t.endRampS >= 1 && t.endRampS <= 300;
    for (uint8_t i = 0; ok && i < t.n; i++)
      ok = t.s[i].rpm >= RPM_MIN && t.s[i].rpm <= RPM_MAX &&
           t.s[i].rampS >= 1 && t.s[i].rampS <= 300 && t.s[i].holdS <= 3600;
    if (ok) prog = t;
  }
  pr.end();
}