<p align="center">
  <h1 align="center">🤖 RoboServo</h1>
  <p align="center">
    <strong>A lightweight servo control library for ESP32 and ESP8266</strong>
  </p>
  <p align="center">
    <img src="https://img.shields.io/badge/version-1.1.0-blue?style=flat-square" alt="Version">
    <img src="https://img.shields.io/badge/ESP32-supported-blue?style=flat-square" alt="ESP32">
    <img src="https://img.shields.io/badge/ESP32--S2/S3-supported-blue?style=flat-square" alt="ESP32-S2/S3">
    <img src="https://img.shields.io/badge/ESP32--C3/C6/H2-supported-blue?style=flat-square" alt="ESP32-C3/C6/H2">
    <img src="https://img.shields.io/badge/ESP32--P4-supported-blue?style=flat-square" alt="ESP32-P4">
    <img src="https://img.shields.io/badge/ESP8266-supported-orange?style=flat-square" alt="ESP8266">
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

---

## 🚀 Quick Start

### Installation

**Arduino IDE:**  
`Sketch` → `Include Library` → `Add .ZIP Library...`

**PlatformIO:**
```ini
lib_deps = RoboServo
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

---

## 📁 Examples

| Example | Description |
|:--------|:------------|
| [BasicServo](examples/BasicServo) | Single servo sweep |
| [MultipleServos](examples/MultipleServos) | Independent multi-servo control |
| [ServoGroup](examples/ServoGroup) | Coordinated group movements |
| [Servo360](examples/Servo360) | Continuous rotation control |
| [ServoTypes](examples/ServoTypes) | 180°, 270°, custom angles |
| [CustomPulseWidth](examples/CustomPulseWidth) | Pulse calibration tool |
| [ServoWithPWM](examples/ServoWithPWM) | Coexisting with LED PWM |
| [ADCServoControl](examples/ADCServoControl) | Potentiometer control |

---

## 🔍 Troubleshooting

| Problem | Solution |
|:--------|:---------|
| Servo not moving | Check 5V power supply; verify GPIO is valid for your board |
| Servo jittering | Use external power; add 100μF capacitor near servo |
| Limited rotation range | Calibrate pulse width (try 1000-2000μs range) |
| Stops after `analogWrite()` | Timer conflict — see solutions above |
| `attach()` returns 255 | No channels available or invalid pin |

---

## 📄 License

MIT License — see [LICENSE](LICENSE) for details.

---

<p align="center">
  Made with ❤️ for the ESP32 & ESP8266 community
</p>
<p align="center">
  <sub>v1.1.0 • Supports ESP32, ESP32-S2/S3, ESP32-C3/C6/H2, ESP32-P4, ESP8266</sub>
</p>
