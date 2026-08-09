<p align="center">
  <h1 align="center">🤖 RoboServo</h1>
  <p align="center">
    <strong>A lightweight servo control library for ESP32, ESP8266, nRF52/nRF53, RP2040, STM32, AVR, UNO R4, UNO Q, and Mbed</strong>
  </p>
  <p align="center">
    <img src="https://img.shields.io/badge/version-1.2.0-blue?style=flat-square" alt="Version">
    <img src="https://img.shields.io/badge/ESP32-supported-blue?style=flat-square" alt="ESP32">
    <img src="https://img.shields.io/badge/ESP32--S2/S3-supported-blue?style=flat-square" alt="ESP32-S2/S3">
    <img src="https://img.shields.io/badge/ESP32--C3/C6/H2-supported-blue?style=flat-square" alt="ESP32-C3/C6/H2">
    <img src="https://img.shields.io/badge/ESP32--P4-supported-blue?style=flat-square" alt="ESP32-P4">
    <img src="https://img.shields.io/badge/ESP8266-supported-orange?style=flat-square" alt="ESP8266">
    <img src="https://img.shields.io/badge/nRF52-supported-teal?style=flat-square" alt="nRF52">
    <img src="https://img.shields.io/badge/RP2040-supported-purple?style=flat-square" alt="RP2040">
    <img src="https://img.shields.io/badge/STM32-supported-red?style=flat-square" alt="STM32">
    <img src="https://img.shields.io/badge/AVR-supported-yellow?style=flat-square" alt="AVR">
    <img src="https://img.shields.io/badge/UNO%20R4-supported-green?style=flat-square" alt="UNO R4">
    <img src="https://img.shields.io/badge/UNO%20Q-supported-green?style=flat-square" alt="UNO Q">
    <img src="https://img.shields.io/badge/Mbed-supported-blue?style=flat-square" alt="Mbed">
    <img src="https://img.shields.io/badge/license-MIT-green?style=flat-square" alt="License">
  </p>
</p>

---

## ✨ Features

- 🎯 **Simple API** — Familiar Arduino Servo-style interface
- 📦 **Multi-servo support** — Up to 8 servos (6 on smaller variants)
- 🔄 **Servo types** — 180°, 270°, 360° continuous, or custom angles
- ⚡ **High precision** — 14-bit PWM (ESP32) / 10-bit PWM (ESP8266)
- 🔧 **Configurable** — Adjustable frequency (40-400Hz) and pulse width
- 🏎️ **High-speed PWM** — 333Hz digital servos and kHz motor/ESC outputs via `RoboMotor`
- 🧵 **Thread-safe** — Safe channel allocation for RTOS (ESP32)

---

## 📋 Supported Boards

| Chip | Max Servos | PWM Resolution | Status |
|:-----|:----------:|:--------------:|:------:|
| ESP32 | 8 | 14-bit | ✅ |
| ESP32-S2 | 8 | 14-bit | ✅ |
| ESP32-S3 | 8 | 14-bit | ✅ |
| ESP32-C3 | 6 | 14-bit | ✅ |
| ESP32-C6 | 6 | 14-bit | ✅ |
| ESP32-H2 | 6 | 14-bit | ✅ |
| ESP32-P4 | 8 | 14-bit | ✅ |
| ESP8266 | 8 | 10-bit | ✅ |
| ArduinoNRF nRF52 | 8 | 10-bit | ✅ |
| Adafruit / nRF52 DK | 6 | 10-bit | ✅ |
| RP2040 / RP2350 | 8 | 10-bit | ✅ |
| STM32 | 6 | 10-bit | ✅ |
| AVR (UNO/Nano) | 6 | 8-bit (ISR) | ✅ |
| AVR (Mega) | 12 | 8-bit (ISR) | ✅ |
| UNO R4 Minima / WiFi | 8 | 10-bit | ✅ |
| UNO Q | 6 | 10-bit | ✅ |
| Mbed (Nano 33 BLE) | 8 | 10-bit | ✅ |
| Mbed (Portenta H7 / X8) | 8 | 10-bit | ✅ |
| Mbed (Nicla Sense / Vision / Voice) | 8 | 10-bit | ✅ |
| Mbed (Giga R1 / Opta / Edge / Stella) | 8 | 10-bit | ✅ |
| Mbed (Nano RP2040 Connect / Pico) | 8 | 10-bit | ✅ |
| nRF5340 (generic core) | 6 | 10-bit | ✅ |

### High-Speed PWM Outputs (RoboMotor)

| Chip | Max Motors | Default Freq | PWM Resolution | Mix with 50Hz Servo |
|:-----|:----------:|:------------:|:--------------:|:-------------------:|
| ESP32 | 4 | 20 kHz | 10-bit | ✅ (isolated timer group) |
| ESP32-S2 / S3 | 4 | 20 kHz | 10-bit | ✅ |
| ESP32-C3 / C6 / H2 | 2 | 20 kHz | 10-bit | ✅ |
| ESP32-P4 | 4 | 20 kHz | 10-bit | ✅ |
| ESP8266 | 4 | 20 kHz | 10-bit | ❌ (global single frequency) |
| ArduinoNRF nRF52 | 4 | 20 kHz | 10-bit | ✅ (per-pin frequency groups) |
| Adafruit / nRF52 DK | 2 | 20 kHz | 10-bit | ❌ (shared frequency) |
| RP2040 / RP2350 | 2 | 20 kHz | 10-bit | ❌ (shared frequency) |
| STM32 | 2 | 20 kHz | 10-bit | ❌ (shared frequency) |
| AVR | — | — | — | ❌ (servo only) |
| UNO R4 / UNO Q / Mbed | — | — | — | ❌ (servo only) |

---

## 🚀 Quick Start

### Installation

**Arduino IDE 2.x (Library Manager):**  
Select an **ESP32**, **ESP8266**, **nRF52**, **RP2040**, **STM32**, or **AVR** board, then `Tools` → `Manage Libraries…` → search **RoboServo** → Install.

**Arduino IDE (ZIP):**  
`Sketch` → `Include Library` → `Add .ZIP Library...`

**PlatformIO:**
```ini
lib_deps = https://github.com/dunknowcoding/RoboServo.git#v1.2.0
```

### Basic Example

```cpp
#include <RoboServo.h>

RoboServo myServo;

void setup() {
    myServo.attach(13);   // Attach to GPIO 13
    myServo.write(90);    // Move to 90°
}

void loop() {
    myServo.write(0);
    delay(1000);
    myServo.write(180);
    delay(1000);
}
```

---

## 📖 API Reference

### RoboServo Class

#### Attachment

```cpp
uint8_t attach(int pin);
uint8_t attach(int pin, int minPulseUs, int maxPulseUs);
uint8_t attach(int pin, int minPulseUs, int maxPulseUs, RoboServoType type);
uint8_t attach(int pin, int minPulseUs, int maxPulseUs, int maxAngle);
uint8_t attach(int pin, int minPulseUs, int maxPulseUs, RoboServoType type, int frequency);

void detach();
bool attached();
```

#### Position Control

```cpp
void write(int angle);               // Set angle (0 to maxAngle)
void writeMicroseconds(int pulseUs); // Set pulse width directly
int read();                          // Get current angle
int readMicroseconds();              // Get current pulse width
```

#### Configuration

```cpp
void setServoType(RoboServoType type);   // SERVO_TYPE_180, _270, _360, _CUSTOM
void setMaxAngle(int angle);              // Custom max angle
void setPulseLimits(int minUs, int maxUs);
void setFrequency(int frequency);         // 40-400 Hz (default: 50)

RoboServoType getServoType();
int getMaxAngle();
int getMinPulse();
int getMaxPulse();
int getFrequency();
int getPin();
```

#### 360° Continuous Servo

```cpp
void setSpeed(int speed);  // -100 (full reverse) to +100 (full forward)
void stop();               // Stop rotation (center pulse)
void release();            // Release PWM signal (go limp)
```

#### Static Methods

```cpp
static int getAttachedCount();
static uint32_t getDefaultFrequency();  // 50 Hz
static uint8_t getServoResolution();    // 14-bit
```

---

### RoboServoGroup Class

Control multiple servos as a coordinated group.

```cpp
// Add/Remove
int addServo(int pin);
int addServo(int pin, int minPulseUs, int maxPulseUs, RoboServoType type);
bool removeServo(int index);
void removeAll();

// Info
int count();
RoboServo* getServo(int index);

// Group Control
void writeAll(int angle);
void writeAllMicroseconds(int pulseUs);
void writeMultiple(const int* angles, int count);
void stopAll();
void detachAll();

// Individual Control
void write(int index, int angle);
void writeMicroseconds(int index, int pulseUs);
int read(int index);
```

---

### High Refresh-Rate Servos (333–400 Hz)

Standard `RoboServo` API — attach at a higher frequency for digital servos that support it:

```cpp
#include <RoboServoHighSpeed.h>

RoboServo servo;
roboServoAttachHighSpeed(servo, 13);   // 333 Hz, 500-2500 us
servo.write(90);
```

Or set frequency manually: `servo.attach(13, 500, 2500, SERVO_TYPE_180, 333);`

---

### RoboMotor Class

High-frequency PWM for motor driver enable pins and ESC inputs (1–40 kHz, default 20 kHz).

#### Attachment

```cpp
uint8_t attach(int pin);
uint8_t attach(int pin, int frequency);
uint8_t attach(int pin, int frequency, uint8_t resolution);

void detach();
bool attached();
```

#### Duty Cycle Control

```cpp
void write(int dutyPercent);    // 0-100 %
void writeRaw(uint32_t duty);   // 0 to 2^resolution - 1
int read();                     // Current duty %
uint32_t readRaw();             // Current raw duty
```

#### Configuration

```cpp
void setFrequency(int frequency);   // 1000-40000 Hz (default: 20000)
int getFrequency();
uint8_t getResolution();
int getPin();
uint8_t getChannel();
```

#### Motor Control

```cpp
void stop();    // 0% duty
void brake();   // Alias for stop()
```

#### Static Methods

```cpp
static int getAttachedCount();
static uint32_t getDefaultFrequency();  // 20000 Hz
static uint8_t getMotorResolution();    // 10-bit
```

---

### RoboMotorGroup Class

Control multiple motors as a coordinated group.

```cpp
// Add/Remove
int addMotor(int pin);
int addMotor(int pin, int frequency);
bool removeMotor(int index);
void removeAll();

// Info
int count();
RoboMotor* getMotor(int index);

// Group Control
void writeAll(int dutyPercent);
void writeMultiple(const int* duties, int count);
void stopAll();
void detachAll();

// Individual Control
void write(int index, int dutyPercent);
int read(int index);
```

---

## Driver modules

`RoboDrivers.h` adds profile-aware control for common boards between the MCU
and the actuator. The profile prevents physically different modules from
sharing an unsafe truth table.

| API | Modules |
|:----|:--------|
| `RoboHBridge` | L298N, L293D, L298P shield, TB6612FNG, both common DRV8833 breakouts, DRV8871, BTS7960/IBT_2 |
| `RoboL293DShield` | four-channel L293D/74HC595 motor shield |
| `RoboStepDir` | A4988, DRV8825, TMC2208, TMC2209, ATD5833, TB6600 |
| `RoboFourWireStepper` | ULN2003 with four-wire phase sequencing |
| `RoboPca9685` | PCA9685 16-channel servo-signal board, 40-1000 Hz |

The 1803BK is listed as manual-only and cannot be mounted by a control class:
it has a potentiometer and no MCU input. `RoboStepDir` keeps the TMC2208 and
TMC2209 standalone mode tables separate, supports high-impedance ATD5833 mode
states, enforces the TB6600's longer pulse floor, and treats the TB6600 DIP
setting as an externally confirmed value instead of pretending GPIO changed it.
For an Arduino Motor Shield Rev3/L298P, pass its active-high brake pin as the
fourth argument to `beginPwmDirection()`; `drive()` and `coast()` release it,
while `brake()` asserts it. On an IBT-2/BTS7960, `coast()` inhibits both half
bridges and `brake()` enables both low sides.

```cpp
#include <RoboDrivers.h>

RoboHBridge motor;
RoboStepDir axis;

void setup() {
  motor.beginIn1In2(ROBO_DRIVER_TB6612, 4, 5, 6, 7);
  axis.begin(ROBO_DRIVER_A4988, 2, 3, 8, 9, 10, 11, 12, 13);
  axis.setMicrosteps(16);
}

void loop() {
  motor.drive(60);       // signed percentage; negative reverses
  axis.step(true);       // one admitted STEP pulse
}
```

Driver current limits, motor power, and logic power remain physical settings.
Set A4988/DRV8825/TMC/ATD current limits before enabling the motor. Use an
external actuator supply where required and join its ground to the MCU ground.

---

## 🎛️ Servo Types

```cpp
// Standard 180° servo (default)
servo.attach(13, 500, 2500, SERVO_TYPE_180);
servo.write(90);  // Center position

// Extended 270° servo  
servo.attach(13, 500, 2500, SERVO_TYPE_270);
servo.write(135); // Center position

// Custom angle range (e.g., 120°)
servo.attach(13, 500, 2500, 120);
servo.write(60);  // Center position

// Continuous rotation 360° servo
servo.attach(13, 500, 2500, SERVO_TYPE_360);
servo.setSpeed(50);   // 50% forward
servo.setSpeed(-50);  // 50% reverse
servo.stop();         // Stop rotation
```

---

## 🔌 Valid GPIO Pins

| Variant | Valid Pins |
|:--------|:-----------|
| ESP32 | 2, 4, 5, 12-19, 21-23, 25-27, 32-33 |
| ESP32-S2 | 1-21, 26, 33-42 |
| ESP32-S3 | 1-21, 35-45, 47-48 |
| ESP32-C3 | 0-10, 18-21 |
| ESP32-C6 | 0-23 |
| ESP32-H2 | 0-14, 25-27 |
| ESP32-P4 | 0-54 (except 24-25) |
| ESP8266 | 0-5, 12-16 |
| ArduinoNRF nRF52 | PWM-capable pins (see board variant) |
| Adafruit / nRF52 DK | PWM-capable pins (see core docs) |
| RP2040 / RP2350 | PWM-capable pins (`digitalPinHasPWM`) |
| STM32 | PWM-capable pins (`digitalPinHasPWM`) |
| AVR (UNO/Nano/Mega) | Any digital pin (Timer1 ISR) |
| UNO R4 | PWM-capable pins (`digitalPinHasPWM`) |
| UNO Q | Digital pins with `counter_servo` support |
| Mbed (Nano 33 BLE, Portenta, Nicla, etc.) | Any digital pin (20 ms Ticker frame) |
| nRF5340 DK | PWM-capable pins (core-dependent) |

---

## 🔧 Wiring

```
ESP32                    Servo
─────                    ─────
GPIO x  ──────────────►  Signal (Orange/White)
5V      ──────────────►  VCC (Red)
GND     ──────────────►  GND (Brown/Black)
```

> ⚠️ **Power Tip:** Use an external 5V power supply (~1A per servo) when driving multiple servos. The ESP32's 5V pin cannot supply enough current.

---

## ⚡ Timer Conflicts

RoboServo uses PWM at 50Hz by default. Conflicts may occur if `analogWrite()` uses different settings.

**ESP32 Solutions:**

| Method | Description |
|:-------|:------------|
| **Order matters** | Call `analogWrite()` before `servo.attach()` |
| **Use LEDC directly** | Replace `analogWrite()` with `ledcAttach()` + `ledcWrite()` |
| **Match frequency** | `analogWriteFrequency(pin, 50)` |

**ESP8266 Notes:**
- All PWM channels share the same frequency
- RoboServo sets the global PWM frequency on `attach()`
- For best results, use the same frequency for all PWM outputs
- **RoboMotor cannot run alongside RoboServo** — pick one domain per sketch

**nRF52 / RP2040 / STM32 Notes:**
- ArduinoNRF boards use per-pin frequency groups via `nrfPwmSetPinFrequency`
- Adafruit nRF52, RP2040, and STM32 cores share a single PWM frequency per sketch
- RoboServo sets frequency on `attach()`; avoid mixing different frequencies
- **RoboMotor on shared-frequency cores** is limited to 2 outputs and cannot mix 50Hz servo with 20kHz motor

**AVR Notes:**
- Uses Timer1 compare interrupts (same resource as `Tone` and the built-in `Servo` library)
- Do not mix with `Tone()` or the Arduino `Servo` library in the same sketch
- **RoboMotor is not supported on AVR** — use RoboServo for 50Hz hobby servos only

**UNO R4 / UNO Q / Mbed Notes:**
- UNO R4 uses FSP PWM with a 20 ms frame configured in microseconds
- UNO Q requires the board `counter_servo` device (provided on the UNO Q variant)
- Mbed boards use a 20 ms Ticker + GPIO pulse backend (not `analogWrite` at 500 Hz)
- **UNO Q:** install `Arduino_RouterBridge` from Library Manager (required by the Zephyr core)
- **RoboMotor is not supported** on UNO R4, UNO Q, Mbed, or AVR

**nRF5340 Notes:**
- Detected via `ARDUINO_ARCH_NRF53` / `NRF5340_XXAA` macros
- Mbed and Zephyr nRF5340 cores use the Mbed/Zephyr backends above
- Other nRF5340 Arduino cores fall back to shared-frequency `analogWrite`

**RoboMotor on ESP32:**
- Motor outputs use a separate LEDC channel group (above channel 7) to avoid interfering with 50Hz servos
- See [ServoAndMotor](examples/ServoAndMotor) for mixed low-speed servo + high-speed motor usage

---

## 📁 Examples

| Example | Description |
|:--------|:------------|
| [BasicServo](examples/BasicServo) | Single servo sweep |
| [PlatformSmoke](examples/PlatformSmoke) | Minimal attach/write (no Serial, for UNO Q / Mbed CI) |
| [MultipleServos](examples/MultipleServos) | Independent multi-servo control |
| [ServoGroup](examples/ServoGroup) | Coordinated group movements |
| [Servo360](examples/Servo360) | Continuous rotation control |
| [ServoTypes](examples/ServoTypes) | 180°, 270°, custom angles |
| [CustomPulseWidth](examples/CustomPulseWidth) | Pulse calibration tool |
| [ServoWithPWM](examples/ServoWithPWM) | Coexisting with LED PWM |
| [ADCServoControl](examples/ADCServoControl) | Potentiometer control |
| [HighSpeedServo](examples/HighSpeedServo) | 333Hz digital servo sweep |
| [MotorPwm](examples/MotorPwm) | 20kHz motor duty cycle ramp |
| [MotorGroup](examples/MotorGroup) | Coordinated multi-motor control |
| [ServoAndMotor](examples/ServoAndMotor) | 50Hz servo + 20kHz motor together |
| [HBridgeDriver](examples/HBridgeDriver) | Signed drive, brake, and coast through a TB6612 |
| [StepDirDriver](examples/StepDirDriver) | A4988 microstep, reset, and bounded STEP pulses |
| [FourWireStepper](examples/FourWireStepper) | ULN2003 half-step phase sequence |
| [Pca9685Servos](examples/Pca9685Servos) | PCA9685 servo pulse output over I2C |

---

## 🔍 Troubleshooting

| Problem | Solution |
|:--------|:---------|
| Servo not moving | Check 5V power supply; verify GPIO is valid for your board |
| Servo jittering | Use external power; add 100μF capacitor near servo |
| Limited rotation range | Calibrate pulse width (try 1000-2000μs range) |
| Stops after `analogWrite()` | Timer conflict — see solutions above |
| `attach()` returns 255 | No channels available, invalid pin, or pin already in use |
| Motor not spinning | Verify driver wiring; RoboMotor drives PWM enable, not direction pins |

---

## 📄 License

MIT License — see [LICENSE](LICENSE) for details.

---

<p align="center">
  Made with ❤️ for the embedded robotics community
</p>
<p align="center">
  <sub>v1.2.0 • Servos, motors, H-bridges, step/dir, four-wire steppers, and PCA9685</sub>
</p>
