Spin coating is a way of making very thin, even films on a flat surface. A few drops of liquid are placed on a substrate, which is then spun at high speed.
The centrifugal force pushes the liquid outward and spreads it into a thin layer. 
The centripetal force, helped by the liquid’s  surface tension, holds it on the surface. Once the solvent dries, a smooth film only nanometres to a few microns thick is left.

## 1. Components and Cost

The following components were used to develop the prototype spin coater.

| Sl. No. | Component                                         |        Cost (₹) |
| :-----: | ------------------------------------------------- | --------------: |
|    1    | ESP32 Development Board                           |          442.50 |
|    2    | OLED Display (0.92 inch)                          |          169.92 |
|    3    | Filament                                          | to be specified |
|    4    | A2212 10T 1400KV Brushless Motor + SimonK 30A ESC |          710.82 |
|    5    | DC-DC Step-Down Buck Converter (24V/12V to 9V/5V) |             199 |
|    6    | SMPS 12V 33.33A (400W)                            |             600 |
|    7    | 4×4 Matrix Keypad                                 |              44 |
|    8    | IR Sensor                                         |              29 |
|    9    | 1000µF 50V Electrolytic Capacitor                 |              20 |
|   10    | 0.1µF Ceramic Capacitor (Pack of 5)               |              20 |
|   11    | Electrical Wire                                   |    30 per metre |
|   12    | Jumper Wires                                      |              60 |
|   13    | M3 × 0.5 Screws                                   |               5 |

---

## 2. Introduction

Thin-film deposition is important in electronics, optics, materials science, and laboratory research. Spin coating is a comparatively simple method for producing thin films from liquid solutions.

A small amount of coating liquid is placed on a substrate, which is then rotated. The liquid spreads outward and excess material is expelled from the edge. Film thickness and uniformity depend on factors such as rotational speed, acceleration, spin duration, liquid viscosity, solvent evaporation, dispense volume, and substrate condition.

Commercial spin coaters can be expensive or inaccessible for student projects. The objective of this project is to build a compact, low-cost prototype using readily available electronics and fabricated mechanical parts. An ESP32 provides a flexible platform for controlling operating sequences and can support a display, buttons, speed feedback, and programmable recipes.

A key design challenge is that the motor, ESC, shaft, chuck, and power supply must be selected as a system. High rotational speed makes shaft alignment, rotor balance, secure mounting, and containment especially important. The prototype therefore requires gradual testing and must not be operated at high speed until the shaft and chuck are mechanically secure and the assembly is guarded.

---

## 3. Principle and Working

The spin coater works by rotating a substrate while a small quantity of liquid is present on its upper surface. Rotation produces radial motion in the liquid, spreading it over the substrate. Excess liquid is ejected from the edges.

After spinning, a thin layer remains; its final properties depend on the material and process settings.

---

### 3.1 Working Principle

1. Place the substrate on the rotating chuck and secure it.
2. A small quantity of coating liquid is dispensed onto the substrate.
3. The BLDC motor rotates the chuck at the programmed speed.
4. Centrifugal effects cause the liquid to spread radially across the substrate.
5. Excess liquid is expelled from the edges.
6. After the spinning process, a thin coating remains on the substrate.
---

### 3.2 Electronic Control System

The ESP32 acts as the main controller. It produces a servo-style control pulse for the ESC, which commutates the three-phase BLDC motor.

The motor rotates the shaft and chuck. The power supply provides the required DC power, while a separate regulated supply or DC-DC converter provides the appropriate voltage for the ESP32 and its peripherals.

The major system components perform the following functions:

| Component            | Function                                                      |
| -------------------- | ------------------------------------------------------------- |
| ESP32                | Main controller for the spin coater                           |
| BLDC Motor           | Rotational drive unit with an appropriate RPM                                |
| ESC                  | Controls the BLDC motor                                       |
| SMPS                 | Supplies DC power to the system                               |
| DC-DC Buck Converter | Provides regulated voltage for the controller and peripherals |
| OLED Display         | Displays operating information (the rpm & its staus etc )                               |
| 4×4 Matrix Keypad    | Allows user input and parameter selection                     |
| IR Sensor            | Detects rotational speed for RPM feedback                     |

---



