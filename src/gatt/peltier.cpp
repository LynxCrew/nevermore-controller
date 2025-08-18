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
    bool use_pid;
};

PeltierSettings p_settings;


struct PeltierControl {
    void update(sensors::PeltierSensors const& sensors = sensors::p_sensors,
            PeltierSettings const& peltier_settings = p_settings);

    void target(BLE::Percentage8 target) {
        _target = target;
    }

    void enable(BLE::Bool enable) {
        _enable = enable;
    }

    [[nodiscard]] BLE::Percentage8 power() const {
        return _power;
    }

private:
    BLE::Percentage8 _power = 0;
    BLE::Peltier_Float _target = 0;
    BLE::Bool _enable = 0;
};

void PeltierControl::update(sensors::PeltierSensors const& sensors = sensors::p_sensors,
        PeltierSettings const& peltier_settings = p_settings) {
    if (sensors.
}

