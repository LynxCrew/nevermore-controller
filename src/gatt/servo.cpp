#include "servo.hpp"
#include "config/pins.hpp"
#include "handler_helpers.hpp"
#include "nevermore.h"
#include "sdk/ble_data_types.hpp"
#include "sdk/pwm.hpp"
#include <cstdint>
#include <limits>

using namespace std;

#define SERVO 2B04_07
#define SERVO_OVERRIDE 2B04_08

namespace nevermore::gatt::servo {

namespace {

constexpr uint32_t PWM_HZ = 10'000;

BLE::Percentage8 g_power_override;  // not-known -> automatic control

void servo_set(BLE::Percentage8 const& power) {
    auto scale = power.value_or(0) / 100.;
    auto duty = uint16_t(numeric_limits<uint16_t>::max() * scale);
    for (auto&& pin : Pins::active().servo_pwm)
        if (pin) pwm_set_gpio_duty(pin, duty);
}

void servo_override(BLE::Percentage8 override) {
    g_power_override = override;
    servo_set(override);
}

}  // namespace

bool init() {
    // setup PWM configurations for fan PWM and fan tachometer
    for (auto&& pin : Pins::active().servo_pwm) {
        if (!pin) continue;

        auto cfg = pwm_get_default_config();
        pwm_config_set_freq_hz(cfg, PWM_HZ);
        pwm_init(pwm_gpio_to_slice_num_(pin), &cfg, true);
    }

    // set fan PWM level
    servo_override(g_power_override);

    return true;
}

void disconnected(hci_con_handle_t) {}

optional<uint16_t> attr_read(
        hci_con_handle_t conn, uint16_t att_handle, uint16_t offset, uint8_t* buffer, uint16_t buffer_size) {
    switch (att_handle) {
        USER_DESCRIBE(SERVO, "Servo %")
        USER_DESCRIBE(SERVO_OVERRIDE, "Servo % - Override")

        READ_VALUE(SERVO, g_power_override.or_(0))
        READ_VALUE(SERVO_OVERRIDE, g_power_override)

    default: return {};
    }
}

optional<int> attr_write(hci_con_handle_t conn, uint16_t att_handle, uint16_t offset, uint8_t const* buffer,
        uint16_t buffer_size) {
    if (buffer_size < offset) return ATT_ERROR_INVALID_OFFSET;
    WriteConsumer consume{offset, buffer, buffer_size};

    switch (att_handle) {
    case HANDLE_ATTR(SERVO_OVERRIDE, VALUE): {
        servo_override((BLE::Percentage8)consume);
        return 0;
    }

    default: return {};
    }
}

}  // namespace nevermore::gatt::servo
