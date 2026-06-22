#include "RoboPlatform.h"

#if defined(ROBOSERVO_PLATFORM_ZEPHYR)

#include <zephyr/devicetree.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/kernel.h>

#include "RoboServo.h"
#include "RoboZephyrBackend.h"

#if DT_NODE_HAS_STATUS(DT_NODELABEL(counter_servo), okay)
    #define ROBOSERVO_ZEPHYR_HAS_COUNTER 1
    #define ROBOSERVO_ZEPHYR_COUNTER DT_NODELABEL(counter_servo)
#else
    #define ROBOSERVO_ZEPHYR_HAS_COUNTER 0
#endif

#if ROBOSERVO_ZEPHYR_HAS_COUNTER

namespace {

constexpr uint32_t kTickUs = 4;
constexpr uint32_t kFrameTicks = 20000 / kTickUs;

struct ZephyrServoSlot {
    uint8_t pin;
    uint32_t positionTicks;
    bool active;
};

static ZephyrServoSlot _slots[ROBOSERVO_MAX_SERVOS];
static volatile uint32_t _timerTick = 0;
static bool _counterReady = false;

static void counterTopHandler(const struct device* counterDev, void* userData) {
    (void)counterDev;
    (void)userData;

    for (uint8_t i = 0; i < ROBOSERVO_MAX_SERVOS; i++) {
        if (!_slots[i].active) continue;
        digitalWrite(_slots[i].pin, (_timerTick > _slots[i].positionTicks) ? LOW : HIGH);
    }

    if (_timerTick >= kFrameTicks) {
        _timerTick = 0;
    } else {
        _timerTick++;
    }
}

static bool ensureCounterStarted() {
    if (_counterReady) return true;

    const struct device* counterDev = DEVICE_DT_GET(ROBOSERVO_ZEPHYR_COUNTER);
    if (!device_is_ready(counterDev)) return false;

    if (counter_start(counterDev) != 0) return false;

    counter_top_cfg topCfg = {};
    topCfg.ticks = counter_us_to_ticks(counterDev, kTickUs);
    topCfg.callback = counterTopHandler;
    topCfg.user_data = nullptr;
    topCfg.flags = 0;

    if (counter_set_top_value(counterDev, &topCfg) != 0) return false;

    _counterReady = true;
    return true;
}

static uint32_t dutyToPulseUs(uint32_t duty, int frequency, uint8_t resolution) {
    const uint32_t maxDuty = (1UL << resolution) - 1UL;
    const uint32_t periodUs = (frequency > 0) ? (1000000UL / (uint32_t)frequency) : 20000UL;
    if (maxDuty == 0) return 1500UL;
    return (duty * periodUs) / maxDuty;
}

} // namespace

#endif // ROBOSERVO_ZEPHYR_HAS_COUNTER

namespace RoboZephyrBackend {

bool isSupported() {
#if ROBOSERVO_ZEPHYR_HAS_COUNTER
    return true;
#else
    return false;
#endif
}

bool isValidPin(int pin) {
    #if defined(PINS_COUNT)
    return pin >= 0 && pin < PINS_COUNT;
    #elif defined(NUM_DIGITAL_PINS)
    return pin >= 0 && pin < NUM_DIGITAL_PINS;
    #else
    return pin >= 0 && pin < 64;
    #endif
}

bool attachChannel(uint8_t slot, int pin, int frequency, RoboPwmDomain domain) {
#if !ROBOSERVO_ZEPHYR_HAS_COUNTER
    (void)slot;
    (void)pin;
    (void)frequency;
    (void)domain;
    return false;
#else
    if (domain != ROBOPWM_DOMAIN_SERVO) return false;
    if (slot >= ROBOSERVO_MAX_SERVOS || !isValidPin(pin)) return false;
    if (frequency < ROBOSERVO_MIN_FREQUENCY || frequency > ROBOSERVO_MAX_FREQUENCY) return false;
    if (!ensureCounterStarted()) return false;

    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);

    _slots[slot].pin = (uint8_t)pin;
    _slots[slot].positionTicks = 1500UL / kTickUs;
    _slots[slot].active = true;
    return true;
#endif
}

void detachChannel(uint8_t slot) {
#if ROBOSERVO_ZEPHYR_HAS_COUNTER
    if (slot >= ROBOSERVO_MAX_SERVOS) return;
    if (_slots[slot].active) {
        digitalWrite(_slots[slot].pin, LOW);
        _slots[slot].active = false;
    }
#else
    (void)slot;
#endif
}

void writeDuty(uint8_t slot, uint32_t duty, int frequency, uint8_t resolution) {
#if !ROBOSERVO_ZEPHYR_HAS_COUNTER
    (void)slot;
    (void)duty;
    (void)frequency;
    (void)resolution;
#else
    if (slot >= ROBOSERVO_MAX_SERVOS || !_slots[slot].active) return;
    const uint32_t pulseUs = dutyToPulseUs(duty, frequency, resolution);
    _slots[slot].positionTicks = pulseUs / kTickUs;
#endif
}

}

#endif // ROBOSERVO_PLATFORM_ZEPHYR
