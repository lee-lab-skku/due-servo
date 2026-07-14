#if defined(ARDUINO_ARCH_ZEPHYR)

#include <Arduino.h>
#include <Servo.h>

#include <zephyr/device.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>

#include <errno.h>

namespace {

struct ServoSlot {
    uint8_t pin = 0;
    uint16_t pulse_us = DEFAULT_PULSE_WIDTH;
    bool active = false;
    bool pin_high = false;
};

enum class SchedulerPhase : uint8_t {
    Idle,
    Pulse,
    Refresh,
};

K_MUTEX_DEFINE(servo_timer_init_mutex);

const struct device *const counter_dev = DEVICE_DT_GET(SERVO_TIMER_NODE);
ServoSlot servos[MAX_ZEPHYR_SERVOS];
uint8_t ServoCount = 0;

bool timer_initialized = false;
bool scheduler_running = false;
bool alarm_armed = false;
SchedulerPhase scheduler_phase = SchedulerPhase::Idle;
uint8_t current_channel = INVALID_SERVO;
uint32_t counter_top_ticks = 0;
uint32_t alarm_guard_ticks = 0;
uint32_t alarm_chunk_limit_ticks = 0;
uint32_t alarm_deadline_ticks = 0;
uint32_t refresh_interval_ticks = 0;
uint32_t minimum_delay_ticks = 1;
uint32_t frame_elapsed_ticks = 0;
uint32_t interval_total_ticks = 0;
uint32_t interval_remaining_ticks = 0;
uint32_t alarm_chunk_ticks = 0;

void alarmCallback(const struct device *dev, uint8_t chan_id, uint32_t ticks, void *user_data);

uint32_t addCounterTicks(uint32_t base, uint32_t delta)
{
    const uint64_t period = static_cast<uint64_t>(counter_top_ticks) + 1U;
    return static_cast<uint32_t>((static_cast<uint64_t>(base) + delta) % period);
}

bool anyServoActive()
{
    for (uint8_t i = 0; i < ServoCount; ++i) {
        if (servos[i].active) {
            return true;
        }
    }
    return false;
}

void forceOutputsLow()
{
    for (uint8_t i = 0; i < ServoCount; ++i) {
        if (servos[i].pin_high) {
            digitalWrite(servos[i].pin, LOW);
            servos[i].pin_high = false;
        }
    }
}

void stopScheduler(bool cancel_alarm)
{
    if (cancel_alarm && alarm_armed) {
        (void)counter_cancel_channel_alarm(counter_dev, SERVO_TIMER_ALARM_CHANNEL);
    }

    alarm_armed = false;
    scheduler_running = false;
    scheduler_phase = SchedulerPhase::Idle;
    current_channel = INVALID_SERVO;
    interval_total_ticks = 0;
    interval_remaining_ticks = 0;
    alarm_chunk_ticks = 0;
    forceOutputsLow();
}

void failScheduler(int error)
{
    (void)error;
    stopScheduler(false);
}

bool armNextAlarmChunk(bool first_chunk)
{
    const uint32_t chunk_count =
        (interval_remaining_ticks + alarm_chunk_limit_ticks - 1U) / alarm_chunk_limit_ticks;
    uint32_t chunk =
        (interval_remaining_ticks + chunk_count - 1U) / chunk_count;
    if (chunk == 0) {
        chunk = 1;
    }

    const bool use_absolute_alarm = interval_total_ticks > alarm_guard_ticks;

    struct counter_alarm_cfg alarm_cfg{};
    alarm_cfg.callback = alarmCallback;
    alarm_cfg.user_data = nullptr;

    if (use_absolute_alarm) {
        if (first_chunk) {
            uint32_t now = 0;
            const int err = counter_get_value(counter_dev, &now);
            if (err != 0) {
                failScheduler(err);
                return false;
            }
            alarm_deadline_ticks = addCounterTicks(now, chunk);
        } else {
            alarm_deadline_ticks = addCounterTicks(alarm_deadline_ticks, chunk);
        }

        alarm_cfg.ticks = alarm_deadline_ticks;
        alarm_cfg.flags =
            COUNTER_ALARM_CFG_ABSOLUTE | COUNTER_ALARM_CFG_EXPIRE_WHEN_LATE;
    } else {
        alarm_cfg.ticks = chunk;
        alarm_cfg.flags = 0;
    }

    const int err = counter_set_channel_alarm(
        counter_dev,
        SERVO_TIMER_ALARM_CHANNEL,
        &alarm_cfg
    );
    if (err != 0 && !(use_absolute_alarm && err == -ETIME)) {
        failScheduler(err);
        return false;
    }

    alarm_chunk_ticks = chunk;
    alarm_armed = true;
    return true;
}

bool beginInterval(uint32_t ticks)
{
    if (ticks == 0) {
        ticks = 1;
    }

    interval_total_ticks = ticks;
    interval_remaining_ticks = ticks;
    return armNextAlarmChunk(true);
}

bool beginChannel(uint8_t channel)
{
    current_channel = channel;
    scheduler_phase = SchedulerPhase::Pulse;

    ServoSlot &servo = servos[channel];
    uint32_t pulse_ticks = counter_us_to_ticks(counter_dev, servo.pulse_us);
    if (pulse_ticks == 0) {
        pulse_ticks = 1;
    }

    if (!beginInterval(pulse_ticks)) {
        return false;
    }

    if (servo.active) {
        digitalWrite(servo.pin, HIGH);
        servo.pin_high = true;
    }
    return true;
}

bool beginFrame()
{
    if (!anyServoActive() || ServoCount == 0) {
        stopScheduler(false);
        return false;
    }

    scheduler_running = true;
    frame_elapsed_ticks = 0;
    return beginChannel(0);
}

bool beginRefreshInterval()
{
    scheduler_phase = SchedulerPhase::Refresh;
    current_channel = INVALID_SERVO;

    uint32_t delay_ticks = minimum_delay_ticks;
    if (frame_elapsed_ticks < refresh_interval_ticks) {
        delay_ticks = refresh_interval_ticks - frame_elapsed_ticks;
    }
    return beginInterval(delay_ticks);
}

void alarmCallback(const struct device *dev, uint8_t chan_id, uint32_t ticks, void *user_data)
{
    (void)dev;
    (void)chan_id;
    (void)ticks;
    (void)user_data;

    alarm_armed = false;

    if (interval_remaining_ticks > alarm_chunk_ticks) {
        interval_remaining_ticks -= alarm_chunk_ticks;
        (void)armNextAlarmChunk(false);
        return;
    }
    interval_remaining_ticks = 0;

    if (scheduler_phase == SchedulerPhase::Pulse) {
        ServoSlot &servo = servos[current_channel];
        if (servo.pin_high) {
            digitalWrite(servo.pin, LOW);
            servo.pin_high = false;
        }

        frame_elapsed_ticks += interval_total_ticks;

        const uint8_t next_channel = current_channel + 1;
        if (next_channel < ServoCount) {
            (void)beginChannel(next_channel);
        } else {
            (void)beginRefreshInterval();
        }
        return;
    }

    if (scheduler_phase == SchedulerPhase::Refresh) {
        (void)beginFrame();
        return;
    }

    stopScheduler(false);
}

bool initializeTimer()
{
    k_mutex_lock(&servo_timer_init_mutex, K_FOREVER);

    if (timer_initialized) {
        k_mutex_unlock(&servo_timer_init_mutex);
        return true;
    }

    if (!device_is_ready(counter_dev) ||
        counter_get_num_of_channels(counter_dev) <= SERVO_TIMER_ALARM_CHANNEL) {
        k_mutex_unlock(&servo_timer_init_mutex);
        return false;
    }

    (void)counter_stop(counter_dev);

    const uint32_t top_ticks = counter_get_max_top_value(counter_dev);
    if (top_ticks < 2U) {
        k_mutex_unlock(&servo_timer_init_mutex);
        return false;
    }

    struct counter_top_cfg top_cfg{};
    top_cfg.ticks = top_ticks;
    top_cfg.callback = nullptr;
    top_cfg.user_data = nullptr;
    top_cfg.flags = 0;

    int err = counter_set_top_value(counter_dev, &top_cfg);
    if (err == 0) {
        err = counter_start(counter_dev);
    }
    if (err != 0) {
        k_mutex_unlock(&servo_timer_init_mutex);
        return false;
    }

    uint32_t guard_ticks = counter_us_to_ticks(counter_dev, SERVO_TIMER_GUARD_US);
    if (guard_ticks == 0) {
        guard_ticks = 1;
    }
    if (guard_ticks >= top_ticks) {
        (void)counter_stop(counter_dev);
        k_mutex_unlock(&servo_timer_init_mutex);
        return false;
    }

    err = counter_set_guard_period(
        counter_dev,
        guard_ticks,
        COUNTER_GUARD_PERIOD_LATE_TO_SET
    );
    if (err != 0) {
        (void)counter_stop(counter_dev);
        k_mutex_unlock(&servo_timer_init_mutex);
        return false;
    }

    uint32_t refresh_ticks = counter_us_to_ticks(counter_dev, REFRESH_INTERVAL);
    uint32_t minimum_ticks = counter_us_to_ticks(counter_dev, SERVO_TIMER_MIN_DELAY_US);
    if (minimum_ticks == 0) {
        minimum_ticks = 1;
    }

    const unsigned int key = irq_lock();
    counter_top_ticks = top_ticks;
    alarm_guard_ticks = guard_ticks;
    alarm_chunk_limit_ticks = top_ticks - guard_ticks;
    refresh_interval_ticks = refresh_ticks;
    minimum_delay_ticks = minimum_ticks;
    timer_initialized = true;
    irq_unlock(key);

    k_mutex_unlock(&servo_timer_init_mutex);
    return true;
}

} // namespace

Servo::Servo()
{
    const unsigned int key = irq_lock();
    if (ServoCount < MAX_ZEPHYR_SERVOS) {
        servoIndex = ServoCount++;
        servos[servoIndex].pulse_us = DEFAULT_PULSE_WIDTH;
        this->min = 0;
        this->max = 0;
    } else {
        servoIndex = INVALID_SERVO;
        this->min = 0;
        this->max = 0;
    }
    irq_unlock(key);
}

uint8_t Servo::attach(int pin)
{
    return attach(pin, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
}

uint8_t Servo::attach(int pin, int min, int max)
{
    if (servoIndex == INVALID_SERVO || !initializeTimer()) {
        return INVALID_SERVO;
    }

    pinMode(pin, OUTPUT);

    const unsigned int key = irq_lock();
    ServoSlot &servo = servos[servoIndex];

    if (servo.pin_high) {
        digitalWrite(servo.pin, LOW);
        servo.pin_high = false;
    }

    servo.pin = static_cast<uint8_t>(pin);
    this->min = static_cast<int8_t>((MIN_PULSE_WIDTH - min) / 4);
    this->max = static_cast<int8_t>((MAX_PULSE_WIDTH - max) / 4);
    servo.active = true;

    if (!scheduler_running) {
        (void)beginFrame();
    }

    irq_unlock(key);
    return servoIndex;
}

void Servo::detach()
{
    if (servoIndex == INVALID_SERVO) {
        return;
    }

    const unsigned int key = irq_lock();
    ServoSlot &servo = servos[servoIndex];

    if (servo.pin_high) {
        digitalWrite(servo.pin, LOW);
        servo.pin_high = false;
    }
    servo.active = false;

    if (!anyServoActive()) {
        stopScheduler(true);
    }

    irq_unlock(key);
}

void Servo::write(int value)
{
    if (value < MIN_PULSE_WIDTH) {
        if (value < 0) {
            value = 0;
        } else if (value > 180) {
            value = 180;
        }
        value = map(
            value,
            0,
            180,
            MIN_PULSE_WIDTH - this->min * 4,
            MAX_PULSE_WIDTH - this->max * 4
        );
    }
    writeMicroseconds(value);
}

void Servo::writeMicroseconds(int value)
{
    if (servoIndex == INVALID_SERVO) {
        return;
    }

    const int min_pulse = MIN_PULSE_WIDTH - this->min * 4;
    const int max_pulse = MAX_PULSE_WIDTH - this->max * 4;

    if (value < min_pulse) {
        value = min_pulse;
    } else if (value > max_pulse) {
        value = max_pulse;
    }

    const unsigned int key = irq_lock();
    servos[servoIndex].pulse_us = static_cast<uint16_t>(value);

    if (servos[servoIndex].active && timer_initialized && !scheduler_running) {
        (void)beginFrame();
    }
    irq_unlock(key);
}

int Servo::read()
{
    return map(
        readMicroseconds() + 1,
        MIN_PULSE_WIDTH - this->min * 4,
        MAX_PULSE_WIDTH - this->max * 4,
        0,
        180
    );
}

int Servo::readMicroseconds()
{
    if (servoIndex == INVALID_SERVO) {
        return 0;
    }

    const unsigned int key = irq_lock();
    const int pulse = servos[servoIndex].pulse_us;
    irq_unlock(key);
    return pulse;
}

bool Servo::attached()
{
    if (servoIndex == INVALID_SERVO) {
        return false;
    }

    const unsigned int key = irq_lock();
    const bool active = servos[servoIndex].active;
    irq_unlock(key);
    return active;
}

#endif
