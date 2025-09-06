#include "config.hpp"
#include "fan.hpp"
#include "handler_helpers.hpp"
#include "sdk/ble_data_types.hpp"
#include "sdk/btstack.hpp"
#include "sdk/pwm.hpp"
#include "sensors.hpp"
#include "sensors/tachometer.hpp"
#include "settings.hpp"
#include "utility/fan_policy.hpp"
#include "utility/fan_policy_thermal.hpp"
#include "utility/timer.hpp"
#include <chrono>
#include <limits>

using namespace std;

namespace nevermore::gatt::peltier {

namespace {
struct PeltierSettings {
    float min_temp_cold;
    float max_temp_cold;
    float min_temp_hot;
    float max_temp_hot;
    float max_deviation;
    float dew_point_safety;
    float hot_side_safety;
    float dew_point_base;
    float dew_point_range;
    float enable_delay;
    float smooth_time;
    float kp;
    float ki;
    float kd;
    float cycle_time;
    float max_pwm;
    bool use_pid;
};

PeltierSettings p_settings;


struct PeltierControl {
    using Clock = std::chrono::steady_clock;

    void update(sensors::PeltierSensors const& sensors = sensors::p_sensors,
            PeltierSettings const& peltier_settings = p_settings);

    void target(BLE::Temperature target) {
        _target = target;
    }

    void enable(BLE::Bool enable) {
        _enable = enable;
    }

    [[nodiscard]] BLE::Percentage8 power() const {
        return _power;
    }

private:
    [[nodiscard]] bool can_turn_on() const {
        return Clock::now() < enable_time;
    }

    Clock::time_point enable_time = Clock::time_point::min();
    Clock::time_point prev_temp_time = Clock::time_point::min();
    double prev_error = 0;
    double prev_der = 0;
    double int_sum = 0;
    double prev_temp = 0;

    BLE::Percentage8 _power = 0;
    BLE::Temperature _target = 0;
    BLE::Bool _enable = 0;

};

PeltierControl g_peltier;

void peltier_target_temp_set(BLE::Temperature target) {
    g_peltier.target(target);
}

void PeltierControl::update(sensors::PeltierSensors const& sensors = sensors::p_sensors,
        PeltierSettings const& peltier_settings = p_settings) {
    if (sensors.temperature_cold < peltier_settings.min_temp_cold) {
        //ERROR
    }
    if (sensors.temperature_cold > peltier_settings.max_temp_cold) {
        //ERROR
    }
    if (sensors.temperature_hot < peltier_settings.min_temp_hot) {
        //ERROR
    }
    if (sensors.temperature_hot > peltier_settings.max_temp_hot) {
        //ERROR
    }
    if (abs(sensors.temperature_hot - sensors.temperature_cold) > peltier_settings.max_deviation) {
        //ERROR
    }

    double power = 100;

    if (!peltier_settings.use_pid) {
        if (sensors.temperature_cold < _target) {
            if (can_turn_on()) {
                power = 100;
            } else {
                power = 0;
            }
        } else {
            power = 0;
            enable_time =
                    Clock::now() + std::chrono::milliseconds(uint64_t(1000 * p_settings.enable_delay));
        }
    } else {
        double error = _target - sensors.temperature_cold;
        double dt = (Clock::now() - prev_temp_time).count();
        double ic = ((prev_error + error) / 2.0) * dt;

        double i = int_sum + ic;

        double n = max(1.0, p_settings.smooth_time / dt);
        double dc = -(sensors.temperature_cold - prev_temp) / dt;
        dc = ((n - 1.0) * prev_der + dc) / n;
        
        double o = p_settings.kp * error + p_settings.ki * i + p_settings.kd * dc;
        double so = max(0.0, min(p_settings.max_pwm, o));

        double pwm = p_settings.max_pwm - so;
        pwm = pwm * 100;


    }


    if (_power != power) {
        _power = power;
        g_notify_fan_power_tacho_aggregate.notify();  // `g_fan_power` changed
        g_notify_aggregate.notify();                  // `g_fan_power` changed
    }

    auto duty = uint16_t(numeric_limits<uint16_t>::max() * (power / 100.));
    for (auto&& pin : Pins::active().peltier_pwm)
        if (pin) pwm_set_gpio_duty(pin, duty);
}

bool init() {
    // setup PWM configurations for fan PWM and fan tachometer
    for (auto&& pin : Pins::active().peltier_pwm) {
        if (!pin) continue;

        auto cfg = pwm_get_default_config();
        pwm_config_set_freq_hz(cfg, 1/p_settings.cycle_time);
        pwm_init(pwm_gpio_to_slice_num_(pin), &cfg, true);
    }

    // HACK:  We'd like to notify on write to tachometer changes, but the code base isn't setup
    //        for that yet. Internally poll and update based on diffs for now.
    mk_timer("gatt-fan-tachometer-notify", SENSOR_UPDATE_PERIOD)([](auto*) {
        static decltype(fan_rpm()) g_prev;
        if (g_prev == fan_rpm()) return;

        g_prev = fan_rpm();
        g_notify_fan_power_tacho_aggregate.notify();
        g_notify_aggregate.notify();
    });

    mk_timer("fan-control", 1.s / FAN_CONTROL_UPDATE_HZ)([](auto*) {
        static auto g_instance = settings::g_active.fan_policy_env.instance();
        // keep updating even w/ `g_fan_power_override` set b/c:
        // * need to refresh to account for thermal throttling policy
        // * automatic PID needs to be kept up to date for when we disengage
        auto perc = ({
            // HACK: FIXME: The fan control timer has been observed to stall.
            //              (Presumably this means all timers stall, but I haven't been able to confirm.)
            //              I've  had no luck reproducing this w/ debugger.
            //              For now, disable the guard. This'll allow read races,
            //              but those should be benign.
            // auto _ = sensors::sensors_guard();
            g_instance(sensors::g_sensors);
        });

        g_peltier.update();
    });

    return true;
}