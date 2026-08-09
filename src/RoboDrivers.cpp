#include "RoboDrivers.h"

static const RoboDriverInfo DRIVER_INFO[] = {
    { ROBO_DRIVER_L298N, "l298n", "L298N module", ROBO_DRIVER_HBRIDGE, 2, 1, 0 },
    { ROBO_DRIVER_L293D, "l293d", "L293D", ROBO_DRIVER_HBRIDGE, 2, 1, 0 },
    { ROBO_DRIVER_L298P_SHIELD, "l298p", "L298P shield", ROBO_DRIVER_HBRIDGE, 2, 1, 0 },
    { ROBO_DRIVER_L293D_SHIELD, "l293d_shield", "L293D motor shield", ROBO_DRIVER_HBRIDGE, 4, 1, 0 },
    { ROBO_DRIVER_TB6612, "tb6612", "TB6612FNG", ROBO_DRIVER_HBRIDGE, 2, 1, 0 },
    { ROBO_DRIVER_DRV8833_RED, "drv8833_red", "DRV8833 red breakout", ROBO_DRIVER_HBRIDGE, 2, 1, 0 },
    { ROBO_DRIVER_DRV8833_BLACK, "drv8833_black", "DRV8833 black breakout", ROBO_DRIVER_HBRIDGE, 2, 1, 0 },
    { ROBO_DRIVER_DRV8871, "drv8871", "DRV8871", ROBO_DRIVER_HBRIDGE, 1, 1, 0 },
    { ROBO_DRIVER_BTS7960, "bts7960", "BTS7960 / IBT_2", ROBO_DRIVER_HBRIDGE, 1, 1, 0 },
    { ROBO_DRIVER_ULN2003, "uln2003", "ULN2003 board", ROBO_DRIVER_FOUR_WIRE_STEPPER, 1, 1, 0 },
    { ROBO_DRIVER_A4988, "a4988", "A4988", ROBO_DRIVER_STEP_DIR, 1, 16, 2 },
    { ROBO_DRIVER_DRV8825, "drv8825", "DRV8825", ROBO_DRIVER_STEP_DIR, 1, 32, 2 },
    { ROBO_DRIVER_TMC2208, "tmc2208", "TMC2208", ROBO_DRIVER_STEP_DIR, 1, 16, 2 },
    { ROBO_DRIVER_TMC2209, "tmc2209", "TMC2209", ROBO_DRIVER_STEP_DIR, 1, 64, 2 },
    { ROBO_DRIVER_ATD5833, "atd5833", "ATD5833", ROBO_DRIVER_STEP_DIR, 1, 16, 2 },
    { ROBO_DRIVER_TB6600, "tb6600", "TB6600 module", ROBO_DRIVER_STEP_DIR, 1, 32, 10 },
    { ROBO_DRIVER_1803BK, "1803bk", "1803BK manual controller", ROBO_DRIVER_MANUAL_ONLY, 1, 1, 0 },
    { ROBO_DRIVER_PCA9685, "pca9685", "PCA9685 servo driver", ROBO_DRIVER_PWM_EXPANDER, 16, 1, 0 }
};

const RoboDriverInfo *roboDriverInfo(RoboDriverModule module)
{
    if ((unsigned)module >= (unsigned)ROBO_DRIVER_MODULE_COUNT) return 0;
    return &DRIVER_INFO[(unsigned)module];
}

static int clampPercent(int value)
{
    if (value < -100) return -100;
    if (value > 100) return 100;
    return value;
}

static void writePwmPercent(int pin, int percent)
{
    if (pin < 0) return;
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    analogWrite(pin, (int)((unsigned long)percent * 255UL / 100UL));
}

static bool isIn1In2(RoboDriverModule m)
{
    return m == ROBO_DRIVER_L298N || m == ROBO_DRIVER_L293D ||
           m == ROBO_DRIVER_TB6612 || m == ROBO_DRIVER_DRV8833_RED ||
           m == ROBO_DRIVER_DRV8833_BLACK || m == ROBO_DRIVER_DRV8871;
}

RoboHBridge::RoboHBridge()
    : _module(ROBO_DRIVER_MODULE_COUNT), _mode(MODE_NONE), _a(-1), _b(-1),
      _pwm(-1), _enableA(-1), _enableB(-1), _value(0), _enabled(false)
{}

void RoboHBridge::pwmWritePercent(int pin, int percent)
{
    writePwmPercent(pin, percent);
}

bool RoboHBridge::beginIn1In2(RoboDriverModule module, int in1, int in2,
                               int pwm, int standby)
{
    if (!isIn1In2(module) || in1 < 0 || in2 < 0 || in1 == in2) return false;
    _module = module; _mode = MODE_IN1_IN2; _a = in1; _b = in2;
    _pwm = pwm; _enableA = standby; _enableB = -1; _value = 0;
    pinMode(_a, OUTPUT); pinMode(_b, OUTPUT);
    if (_pwm >= 0) pinMode(_pwm, OUTPUT);
    if (_enableA >= 0) pinMode(_enableA, OUTPUT);
    setEnabled(true);
    coast();
    return true;
}

bool RoboHBridge::beginPwmDirection(RoboDriverModule module, int pwm,
                                     int direction, int brake)
{
    if (module != ROBO_DRIVER_L298P_SHIELD || pwm < 0 || direction < 0 ||
        pwm == direction) return false;
    _module = module; _mode = MODE_PWM_DIR; _a = direction; _b = -1;
    _pwm = pwm; _enableA = brake; _enableB = -1; _value = 0;
    pinMode(_a, OUTPUT); pinMode(_pwm, OUTPUT);
    if (_enableA >= 0) pinMode(_enableA, OUTPUT);
    _enabled = true;
    coast();
    return true;
}

bool RoboHBridge::beginDualPwm(RoboDriverModule module, int forwardPwm,
                                int reversePwm, int forwardEnable,
                                int reverseEnable)
{
    if (module != ROBO_DRIVER_BTS7960 || forwardPwm < 0 || reversePwm < 0 ||
        forwardEnable < 0 || reverseEnable < 0 || forwardPwm == reversePwm)
        return false;
    _module = module; _mode = MODE_DUAL_PWM; _a = forwardPwm; _b = reversePwm;
    _pwm = -1; _enableA = forwardEnable; _enableB = reverseEnable; _value = 0;
    pinMode(_a, OUTPUT); pinMode(_b, OUTPUT);
    pinMode(_enableA, OUTPUT); pinMode(_enableB, OUTPUT);
    setEnabled(true);
    coast();
    return true;
}

void RoboHBridge::setEnabled(bool enabled)
{
    if (_mode == MODE_NONE) return;
    _enabled = enabled;
    if (_mode == MODE_DUAL_PWM) {
        digitalWrite(_enableA, enabled ? HIGH : LOW);
        digitalWrite(_enableB, enabled ? HIGH : LOW);
        if (!enabled) {
            pwmWritePercent(_a, 0);
            pwmWritePercent(_b, 0);
        }
    } else if (_mode == MODE_PWM_DIR) {
        /* The L298P shield's third pin is BRAKE, not ENABLE. */
        if (_enableA >= 0) digitalWrite(_enableA, LOW);
        if (!enabled) pwmWritePercent(_pwm, 0);
    } else {
        if (_enableA >= 0) digitalWrite(_enableA, enabled ? HIGH : LOW);
        if (!enabled) coast();
    }
}

void RoboHBridge::drive(int signedPercent)
{
    int magnitude;
    if (_mode == MODE_NONE || !_enabled) return;
    _value = clampPercent(signedPercent);
    if (_value == 0) { coast(); return; }
    magnitude = _value < 0 ? -_value : _value;
    if (_mode == MODE_IN1_IN2) {
        if (_pwm >= 0) {
            digitalWrite(_a, _value >= 0 ? HIGH : LOW);
            digitalWrite(_b, _value >= 0 ? LOW : HIGH);
            pwmWritePercent(_pwm, magnitude);
        } else {
            pwmWritePercent(_a, _value > 0 ? magnitude : 0);
            pwmWritePercent(_b, _value < 0 ? magnitude : 0);
        }
    } else if (_mode == MODE_PWM_DIR) {
        if (_enableA >= 0) digitalWrite(_enableA, LOW);
        digitalWrite(_a, _value >= 0 ? HIGH : LOW);
        pwmWritePercent(_pwm, magnitude);
    } else if (_mode == MODE_DUAL_PWM) {
        digitalWrite(_enableA, HIGH); digitalWrite(_enableB, HIGH);
        pwmWritePercent(_a, _value > 0 ? magnitude : 0);
        pwmWritePercent(_b, _value < 0 ? magnitude : 0);
    }
}

void RoboHBridge::coast()
{
    _value = 0;
    if (_mode == MODE_IN1_IN2) {
        digitalWrite(_a, LOW); digitalWrite(_b, LOW);
        if (_pwm >= 0) pwmWritePercent(_pwm, 0);
    } else if (_mode == MODE_PWM_DIR) {
        if (_enableA >= 0) digitalWrite(_enableA, LOW);
        pwmWritePercent(_pwm, 0);
    } else if (_mode == MODE_DUAL_PWM) {
        pwmWritePercent(_a, 0); pwmWritePercent(_b, 0);
        digitalWrite(_enableA, LOW); digitalWrite(_enableB, LOW);
    }
}

void RoboHBridge::brake()
{
    _value = 0;
    if (_mode == MODE_IN1_IN2) {
        if (_pwm >= 0) pwmWritePercent(_pwm, 100);
        digitalWrite(_a, HIGH); digitalWrite(_b, HIGH);
    } else if (_mode == MODE_DUAL_PWM) {
        digitalWrite(_enableA, HIGH); digitalWrite(_enableB, HIGH);
        pwmWritePercent(_a, 0); pwmWritePercent(_b, 0);
    } else if (_mode == MODE_PWM_DIR) {
        pwmWritePercent(_pwm, 100);
        if (_enableA >= 0) digitalWrite(_enableA, HIGH);
    } else {
        coast();
    }
}

bool RoboHBridge::attached() const { return _mode != MODE_NONE; }
int RoboHBridge::read() const { return _value; }
RoboDriverModule RoboHBridge::module() const { return _module; }

RoboStepDir::RoboStepDir()
    : _module(ROBO_DRIVER_MODULE_COUNT), _step(-1), _direction(-1), _enable(-1),
      _reset(-1), _sleep(-1), _microsteps(1), _minimumPulseUs(2),
      _activeLow(true), _attached(false)
{ _ms[0] = _ms[1] = _ms[2] = -1; }

bool RoboStepDir::begin(RoboDriverModule module, int stepPin, int directionPin,
                         int enablePin, int ms1, int ms2, int ms3,
                         int resetPin, int sleepPin,
                         bool controlActiveLow)
{
    const RoboDriverInfo *info = roboDriverInfo(module);
    if (!info || info->kind != ROBO_DRIVER_STEP_DIR || stepPin < 0 ||
        directionPin < 0 || stepPin == directionPin) return false;
    _module = module; _step = stepPin; _direction = directionPin;
    _enable = enablePin; _ms[0] = ms1; _ms[1] = ms2; _ms[2] = ms3;
    _reset = resetPin; _sleep = sleepPin;
    _minimumPulseUs = info->minimumStepPulseUs;
    _activeLow = controlActiveLow; _attached = true;
    pinMode(_step, OUTPUT); pinMode(_direction, OUTPUT);
    digitalWrite(_step, LOW); digitalWrite(_direction, LOW);
    if (_enable >= 0) pinMode(_enable, OUTPUT);
    if (_reset >= 0) { pinMode(_reset, OUTPUT); digitalWrite(_reset, HIGH); }
    if (_sleep >= 0) { pinMode(_sleep, OUTPUT); digitalWrite(_sleep, HIGH); }
    setEnabled(true);
    return true;
}

bool RoboStepDir::setBinaryMode(uint8_t a, uint8_t b, uint8_t c)
{
    uint8_t bits[3] = { a, b, c };
    uint8_t i;
    for (i = 0; i < 3; ++i) {
        if (_ms[i] < 0) {
            if (bits[i]) return false;
            continue;
        }
        pinMode(_ms[i], OUTPUT);
        digitalWrite(_ms[i], bits[i] ? HIGH : LOW);
    }
    return true;
}

bool RoboStepDir::setTriStateMode(int8_t a, int8_t b)
{
    int8_t state[2] = { a, b };
    uint8_t i;
    for (i = 0; i < 2; ++i) {
        if (_ms[i] < 0) return false;
        if (state[i] < 0) pinMode(_ms[i], INPUT);
        else {
            pinMode(_ms[i], OUTPUT);
            digitalWrite(_ms[i], state[i] ? HIGH : LOW);
        }
    }
    return true;
}

bool RoboStepDir::setMicrosteps(uint8_t value)
{
    bool ok = false;
    if (!_attached) return false;
    switch (_module) {
    case ROBO_DRIVER_A4988:
        if (value == 1) ok = setBinaryMode(0,0,0);
        else if (value == 2) ok = setBinaryMode(1,0,0);
        else if (value == 4) ok = setBinaryMode(0,1,0);
        else if (value == 8) ok = setBinaryMode(1,1,0);
        else if (value == 16) ok = setBinaryMode(1,1,1);
        break;
    case ROBO_DRIVER_DRV8825:
        if (value == 1) ok = setBinaryMode(0,0,0);
        else if (value == 2) ok = setBinaryMode(1,0,0);
        else if (value == 4) ok = setBinaryMode(0,1,0);
        else if (value == 8) ok = setBinaryMode(1,1,0);
        else if (value == 16) ok = setBinaryMode(0,0,1);
        else if (value == 32) ok = setBinaryMode(1,0,1);
        break;
    case ROBO_DRIVER_TMC2208:
        if (value == 8) ok = setBinaryMode(0,0,0);
        else if (value == 2) ok = setBinaryMode(1,0,0);
        else if (value == 4) ok = setBinaryMode(0,1,0);
        else if (value == 16) ok = setBinaryMode(1,1,0);
        break;
    case ROBO_DRIVER_TMC2209:
        if (value == 8) ok = setBinaryMode(0,0,0);
        else if (value == 32) ok = setBinaryMode(1,0,0);
        else if (value == 64) ok = setBinaryMode(0,1,0);
        else if (value == 16) ok = setBinaryMode(1,1,0);
        break;
    case ROBO_DRIVER_ATD5833:
        if (value == 1) ok = setTriStateMode(0,0);
        else if (value == 2) ok = setTriStateMode(1,0);
        else if (value == 4) ok = setTriStateMode(1,-1);
        else if (value == 16) ok = setTriStateMode(-1,-1);
        break;
    case ROBO_DRIVER_TB6600:
        break;
    default:
        break;
    }
    if (ok) _microsteps = value;
    return ok;
}

bool RoboStepDir::confirmExternalMicrosteps(uint8_t value)
{
    if (!_attached || _module != ROBO_DRIVER_TB6600) return false;
    if (value != 1 && value != 2 && value != 4 && value != 8 &&
        value != 16 && value != 32) return false;
    _microsteps = value;
    return true;
}

void RoboStepDir::setEnabled(bool enabled)
{
    if (_enable >= 0)
        digitalWrite(_enable, (enabled != _activeLow) ? HIGH : LOW);
}

void RoboStepDir::sleep(bool sleeping)
{
    if (_sleep >= 0) digitalWrite(_sleep, sleeping ? LOW : HIGH);
}

void RoboStepDir::resetIndexer()
{
    if (_reset < 0) return;
    digitalWrite(_reset, LOW);
    delayMicroseconds(2);
    digitalWrite(_reset, HIGH);
}

bool RoboStepDir::step(bool forward) { return step(forward, _minimumPulseUs); }

bool RoboStepDir::step(bool forward, uint16_t pulseUs)
{
    if (!_attached || pulseUs < _minimumPulseUs) return false;
    digitalWrite(_direction, forward ? HIGH : LOW);
    delayMicroseconds(_minimumPulseUs);
    digitalWrite(_step, HIGH);
    delayMicroseconds(pulseUs);
    digitalWrite(_step, LOW);
    return true;
}

bool RoboStepDir::attached() const { return _attached; }
uint8_t RoboStepDir::microsteps() const { return _microsteps; }
uint16_t RoboStepDir::minimumPulseUs() const { return _minimumPulseUs; }
RoboDriverModule RoboStepDir::module() const { return _module; }

RoboFourWireStepper::RoboFourWireStepper()
    : _module(ROBO_DRIVER_MODULE_COUNT), _phase(0), _position(0), _attached(false)
{ _pins[0] = _pins[1] = _pins[2] = _pins[3] = -1; }

bool RoboFourWireStepper::begin(RoboDriverModule module, int in1, int in2,
                                 int in3, int in4)
{
    int pins[4] = { in1, in2, in3, in4 };
    uint8_t i, j;
    if (module != ROBO_DRIVER_ULN2003) return false;
    for (i = 0; i < 4; ++i) {
        if (pins[i] < 0) return false;
        for (j = 0; j < i; ++j) if (pins[i] == pins[j]) return false;
    }
    _module = module; _phase = 0; _position = 0; _attached = true;
    for (i = 0; i < 4; ++i) { _pins[i] = pins[i]; pinMode(_pins[i], OUTPUT); }
    release();
    return true;
}

void RoboFourWireStepper::applyPhase()
{
    static const uint8_t HALF[8] = { 0x1, 0x3, 0x2, 0x6, 0x4, 0xC, 0x8, 0x9 };
    uint8_t mask = HALF[_phase & 7u];
    uint8_t i;
    for (i = 0; i < 4; ++i) digitalWrite(_pins[i], (mask & (1u << i)) ? HIGH : LOW);
}

bool RoboFourWireStepper::step(bool forward)
{
    if (!_attached) return false;
    _phase = (uint8_t)((_phase + (forward ? 1u : 7u)) & 7u);
    _position += forward ? 1L : -1L;
    applyPhase();
    return true;
}

void RoboFourWireStepper::release()
{
    uint8_t i;
    if (!_attached) return;
    for (i = 0; i < 4; ++i) digitalWrite(_pins[i], LOW);
}

long RoboFourWireStepper::position() const { return _position; }
bool RoboFourWireStepper::attached() const { return _attached; }

RoboL293DShield::RoboL293DShield()
    : _data(-1), _latch(-1), _clock(-1), _oe(-1), _state(0), _attached(false)
{}

bool RoboL293DShield::begin(int dataPin, int latchPin, int clockPin,
                             int outputEnablePin)
{
    if (dataPin < 0 || latchPin < 0 || clockPin < 0 || outputEnablePin < 0)
        return false;
    _data = dataPin; _latch = latchPin; _clock = clockPin; _oe = outputEnablePin;
    pinMode(_data, OUTPUT); pinMode(_latch, OUTPUT); pinMode(_clock, OUTPUT);
    pinMode(_oe, OUTPUT); digitalWrite(_oe, LOW);
    _state = 0; _attached = true; commit();
    return true;
}

void RoboL293DShield::commit()
{
    digitalWrite(_latch, LOW);
    shiftOut(_data, _clock, MSBFIRST, _state);
    digitalWrite(_latch, HIGH);
}

bool RoboL293DShield::setDirection(uint8_t channel, int direction, bool braking)
{
    static const uint8_t A[4] = { 2, 1, 5, 0 };
    static const uint8_t B[4] = { 3, 4, 7, 6 };
    uint8_t maskA, maskB;
    if (!_attached || channel < 1 || channel > 4) return false;
    --channel; maskA = (uint8_t)(1u << A[channel]); maskB = (uint8_t)(1u << B[channel]);
    _state &= (uint8_t)~(maskA | maskB);
    if (braking) _state |= (uint8_t)(maskA | maskB);
    else if (direction > 0) _state |= maskA;
    else if (direction < 0) _state |= maskB;
    commit();
    return true;
}

bool RoboL293DShield::drive(uint8_t channel, int signedPercent)
{
    static const uint8_t PWM[4] = { 11, 3, 6, 5 };
    int v = clampPercent(signedPercent);
    if (!setDirection(channel, v, false)) return false;
    writePwmPercent(PWM[channel - 1], v < 0 ? -v : v);
    return true;
}

bool RoboL293DShield::coast(uint8_t channel)
{
    static const uint8_t PWM[4] = { 11, 3, 6, 5 };
    if (!setDirection(channel, 0, false)) return false;
    writePwmPercent(PWM[channel - 1], 0);
    return true;
}

bool RoboL293DShield::brake(uint8_t channel)
{
    static const uint8_t PWM[4] = { 11, 3, 6, 5 };
    if (!setDirection(channel, 0, true)) return false;
    writePwmPercent(PWM[channel - 1], 100);
    return true;
}

void RoboL293DShield::releaseAll()
{
    uint8_t i;
    for (i = 1; i <= 4; ++i) coast(i);
}

RoboPca9685::RoboPca9685()
    : _wire(0), _address(0x40), _frequency(50), _attached(false)
{}

bool RoboPca9685::write8(uint8_t reg, uint8_t value)
{
    _wire->beginTransmission(_address);
    _wire->write(reg); _wire->write(value);
    return _wire->endTransmission() == 0;
}

bool RoboPca9685::read8(uint8_t reg, uint8_t &value)
{
    _wire->beginTransmission(_address); _wire->write(reg);
    if (_wire->endTransmission(false) != 0) return false;
    if (_wire->requestFrom((int)_address, 1) != 1) return false;
    value = _wire->read();
    return true;
}

bool RoboPca9685::begin(TwoWire &wire, uint8_t address)
{
    if (address < 0x40 || address > 0x7F) return false;
    _wire = &wire; _address = address; _wire->begin();
    _attached = write8(0x00, 0x20);
    return _attached && setFrequency(50);
}

bool RoboPca9685::setFrequency(uint16_t hz)
{
    uint8_t oldMode;
    uint8_t prescale;
    unsigned long divisor;
    unsigned long rounded;
    if (!_attached || hz < 40 || hz > 1000 || !read8(0x00, oldMode)) return false;
    divisor = 4096UL * (unsigned long)hz;
    rounded = (25000000UL + divisor / 2UL) / divisor;
    if (rounded < 4UL || rounded > 256UL) return false;
    prescale = (uint8_t)(rounded - 1UL);
    if (!write8(0x00, (uint8_t)((oldMode & 0x7Fu) | 0x10u))) return false;
    if (!write8(0xFE, prescale)) return false;
    if (!write8(0x00, oldMode)) return false;
    delayMicroseconds(500);
    if (!write8(0x00, (uint8_t)(oldMode | 0xA0u))) return false;
    _frequency = hz;
    return true;
}

bool RoboPca9685::writeTicks(uint8_t channel, uint16_t onTick, uint16_t offTick)
{
    uint8_t reg;
    if (!_attached || channel >= 16 || onTick > 4095 || offTick > 4095) return false;
    reg = (uint8_t)(0x06 + channel * 4u);
    _wire->beginTransmission(_address); _wire->write(reg);
    _wire->write((uint8_t)onTick); _wire->write((uint8_t)(onTick >> 8));
    _wire->write((uint8_t)offTick); _wire->write((uint8_t)(offTick >> 8));
    return _wire->endTransmission() == 0;
}

bool RoboPca9685::writeMicroseconds(uint8_t channel, uint16_t pulseUs)
{
    unsigned long ticks;
    if (!_attached || pulseUs > 25000u) return false;
    ticks = (unsigned long)pulseUs * (unsigned long)_frequency * 4096UL / 1000000UL;
    if (ticks > 4095UL) ticks = 4095UL;
    return writeTicks(channel, 0, (uint16_t)ticks);
}

void RoboPca9685::allOff()
{
    uint8_t channel;
    if (!_attached) return;
    for (channel = 0; channel < 16; ++channel) {
        uint8_t reg = (uint8_t)(0x06 + channel * 4u);
        _wire->beginTransmission(_address); _wire->write(reg);
        _wire->write((uint8_t)0); _wire->write((uint8_t)0);
        _wire->write((uint8_t)0); _wire->write((uint8_t)0x10);
        _wire->endTransmission();
    }
}

uint16_t RoboPca9685::frequency() const { return _frequency; }
