/**
 * @file RoboMotor.cpp
 * @brief Implementation for high-frequency PWM motor control
 */

#include "RoboMotor.h"

// =============================================================================
// Platform-specific variables
// =============================================================================

#if defined(ROBOMOTOR_PLATFORM_ESP32)
    static portMUX_TYPE _motorSlotMux = portMUX_INITIALIZER_UNLOCKED;
    #define ROBOMOTOR_ENTER_CRITICAL() portENTER_CRITICAL(&_motorSlotMux)
    #define ROBOMOTOR_EXIT_CRITICAL()  portEXIT_CRITICAL(&_motorSlotMux)
#else
    #define ROBOMOTOR_ENTER_CRITICAL()
    #define ROBOMOTOR_EXIT_CRITICAL()
#endif

static uint8_t _motorSlotMask = 0;

// =============================================================================
// RoboMotor Implementation
// =============================================================================

RoboMotor::RoboMotor()
    : _pin(-1), _slot(ROBOMOTOR_INVALID), _hwChannel(0)
    , _frequency(ROBOMOTOR_DEFAULT_FREQUENCY)
    , _resolution(ROBOMOTOR_PWM_RESOLUTION)
    , _currentDuty(0), _currentPercent(0), _attached(false)
{}

RoboMotor::~RoboMotor() {
    if (_attached) detach();
}

uint8_t RoboMotor::slotToHwChannel(uint8_t slot) {
    return (uint8_t)(ROBOMOTOR_CHANNEL_BASE + slot);
}

uint8_t RoboMotor::allocateSlot() {
    ROBOMOTOR_ENTER_CRITICAL();
    for (uint8_t i = 0; i < ROBOMOTOR_MAX_MOTORS; i++) {
        if (!(_motorSlotMask & (1 << i))) {
            _motorSlotMask |= (1 << i);
            ROBOMOTOR_EXIT_CRITICAL();
            return i;
        }
    }
    ROBOMOTOR_EXIT_CRITICAL();
    return ROBOMOTOR_INVALID;
}

void RoboMotor::releaseSlot(uint8_t slot) {
    if (slot >= ROBOMOTOR_MAX_MOTORS) return;
    ROBOMOTOR_ENTER_CRITICAL();
    _motorSlotMask &= ~(1 << slot);
    ROBOMOTOR_EXIT_CRITICAL();
}

int RoboMotor::getAttachedCount() {
    int count = 0;
    uint8_t mask = _motorSlotMask;
    while (mask) { count += (mask & 1); mask >>= 1; }
    return count;
}

uint8_t RoboMotor::attach(int pin) {
    return attach(pin, ROBOMOTOR_DEFAULT_FREQUENCY);
}

uint8_t RoboMotor::attach(int pin, int frequency) {
    return attach(pin, frequency, ROBOMOTOR_PWM_RESOLUTION);
}

uint8_t RoboMotor::attach(int pin, int frequency, uint8_t resolution) {
    if (_attached) {
        if (_pin == pin) {
            _frequency = frequency;
            _resolution = resolution;
            return _slot;
        }
        detach();
    }

    if (pin < 0 || !RoboPwmBackend::isValidPwmPin(pin)) return ROBOMOTOR_INVALID;
    if (RoboPwmBackend::isPinInUse(pin)) return ROBOMOTOR_INVALID;

    if (frequency < ROBOMOTOR_MIN_FREQUENCY) frequency = ROBOMOTOR_MIN_FREQUENCY;
    else if (frequency > ROBOMOTOR_MAX_FREQUENCY) frequency = ROBOMOTOR_MAX_FREQUENCY;
    _frequency = frequency;

    if (resolution < 8) resolution = 8;
    else if (resolution > 14) resolution = 14;
    _resolution = resolution;

    _slot = allocateSlot();
    if (_slot == ROBOMOTOR_INVALID) return ROBOMOTOR_INVALID;

    _hwChannel = slotToHwChannel(_slot);
    _pin = pin;

    if (!RoboPwmBackend::attachPin(_pin, _hwChannel, _frequency, _resolution, ROBOPWM_DOMAIN_MOTOR)) {
        releaseSlot(_slot);
        _slot = ROBOMOTOR_INVALID;
        _pin = -1;
        return ROBOMOTOR_INVALID;
    }

    _attached = true;
    _currentDuty = 0;
    _currentPercent = 0;
    RoboPwmBackend::writeDuty(_pin, _hwChannel, 0, ROBOPWM_DOMAIN_MOTOR);
    RoboPwmBackend::markPinUsed(_pin);

    return _slot;
}

void RoboMotor::detach() {
    if (!_attached) return;

    RoboPwmBackend::detachPin(_pin, _hwChannel, ROBOPWM_DOMAIN_MOTOR);
    releaseSlot(_slot);
    RoboPwmBackend::markPinFree(_pin);

    _pin = -1;
    _slot = ROBOMOTOR_INVALID;
    _currentDuty = 0;
    _currentPercent = 0;
    _attached = false;
}

bool RoboMotor::attached() const { return _attached; }

uint32_t RoboMotor::percentToTicks(int percent) const {
    return RoboPwmBackend::percentToTicks(percent, _resolution);
}

void RoboMotor::write(int dutyPercent) {
    if (!_attached) return;

    if (dutyPercent < 0) dutyPercent = 0;
    else if (dutyPercent > 100) dutyPercent = 100;

    _currentPercent = dutyPercent;
    _currentDuty = percentToTicks(dutyPercent);
    RoboPwmBackend::writeDuty(_pin, _hwChannel, _currentDuty, ROBOPWM_DOMAIN_MOTOR);
}

void RoboMotor::writeRaw(uint32_t duty) {
    if (!_attached) return;

    uint32_t maxDuty = (1UL << _resolution) - 1;
    if (duty > maxDuty) duty = maxDuty;

    _currentDuty = duty;
    _currentPercent = (maxDuty > 0) ? (int)((duty * 100UL) / maxDuty) : 0;
    RoboPwmBackend::writeDuty(_pin, _hwChannel, _currentDuty, ROBOPWM_DOMAIN_MOTOR);
}

int RoboMotor::read() const { return _currentPercent; }
uint32_t RoboMotor::readRaw() const { return _currentDuty; }

void RoboMotor::setFrequency(int frequency) {
    if (frequency < ROBOMOTOR_MIN_FREQUENCY) frequency = ROBOMOTOR_MIN_FREQUENCY;
    else if (frequency > ROBOMOTOR_MAX_FREQUENCY) frequency = ROBOMOTOR_MAX_FREQUENCY;
    if (frequency == _frequency) return;

    _frequency = frequency;
    if (_attached) {
        int savedPin = _pin;
        uint8_t savedResolution = _resolution;
        int savedPercent = _currentPercent;

        detach();
        if (attach(savedPin, _frequency, savedResolution) != ROBOMOTOR_INVALID) {
            write(savedPercent);
        }
    }
}

int RoboMotor::getFrequency() const { return _frequency; }
uint8_t RoboMotor::getResolution() const { return _resolution; }
int RoboMotor::getPin() const { return _pin; }
uint8_t RoboMotor::getChannel() const { return _slot; }

void RoboMotor::stop() { write(0); }
void RoboMotor::brake() { write(0); }

// =============================================================================
// RoboMotorGroup Implementation
// =============================================================================

RoboMotorGroup::RoboMotorGroup() : _count(0) {
    for (int i = 0; i < ROBOMOTOR_MAX_MOTORS; i++) _motors[i] = nullptr;
}

RoboMotorGroup::~RoboMotorGroup() { removeAll(); }

int RoboMotorGroup::addMotor(int pin) {
    return addMotor(pin, ROBOMOTOR_DEFAULT_FREQUENCY);
}

int RoboMotorGroup::addMotor(int pin, int frequency) {
    if (_count >= ROBOMOTOR_MAX_MOTORS) return -1;

    int index = -1;
    for (int i = 0; i < ROBOMOTOR_MAX_MOTORS; i++) {
        if (_motors[i] == nullptr) { index = i; break; }
    }
    if (index == -1) return -1;

    RoboMotor* motor = new RoboMotor();
    if (motor == nullptr) return -1;

    if (motor->attach(pin, frequency) == ROBOMOTOR_INVALID) {
        delete motor;
        return -1;
    }

    _motors[index] = motor;
    _count++;
    return index;
}

bool RoboMotorGroup::removeMotor(int index) {
    if (index < 0 || index >= ROBOMOTOR_MAX_MOTORS || _motors[index] == nullptr) return false;
    _motors[index]->detach();
    delete _motors[index];
    _motors[index] = nullptr;
    _count--;
    return true;
}

void RoboMotorGroup::removeAll() {
    for (int i = 0; i < ROBOMOTOR_MAX_MOTORS; i++) {
        if (_motors[i] != nullptr) {
            _motors[i]->detach();
            delete _motors[i];
            _motors[i] = nullptr;
        }
    }
    _count = 0;
}

int RoboMotorGroup::count() const { return _count; }

RoboMotor* RoboMotorGroup::getMotor(int index) {
    if (index < 0 || index >= ROBOMOTOR_MAX_MOTORS) return nullptr;
    return _motors[index];
}

void RoboMotorGroup::writeAll(int dutyPercent) {
    for (int i = 0; i < ROBOMOTOR_MAX_MOTORS; i++)
        if (_motors[i]) _motors[i]->write(dutyPercent);
}

void RoboMotorGroup::writeMultiple(const int* duties, int count) {
    if (!duties) return;
    int di = 0;
    for (int i = 0; i < ROBOMOTOR_MAX_MOTORS && di < count; i++)
        if (_motors[i]) _motors[i]->write(duties[di++]);
}

void RoboMotorGroup::write(int index, int dutyPercent) {
    if (index >= 0 && index < ROBOMOTOR_MAX_MOTORS && _motors[index])
        _motors[index]->write(dutyPercent);
}

int RoboMotorGroup::read(int index) const {
    if (index >= 0 && index < ROBOMOTOR_MAX_MOTORS && _motors[index])
        return _motors[index]->read();
    return -1;
}

void RoboMotorGroup::detachAll() {
    for (int i = 0; i < ROBOMOTOR_MAX_MOTORS; i++)
        if (_motors[i]) _motors[i]->detach();
}

void RoboMotorGroup::stopAll() {
    for (int i = 0; i < ROBOMOTOR_MAX_MOTORS; i++)
        if (_motors[i]) _motors[i]->stop();
}
