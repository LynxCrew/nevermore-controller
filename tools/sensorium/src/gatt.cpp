#include "gatt.hpp"
#include "ble/att_server.h"
#include "ble/sm.h"
#include "bluetooth_gatt.h"
#include "btstack_event.h"
#include "config.hpp"
#include "hci_dump.h"
#include "l2cap.h"
#include "picowota/reboot.h"
#include "sdk/gap.hpp"
#include "sensorium.h"
#include "utility/bt_advert.hpp"
#include <chrono>
#include <cstdio>
#include <optional>
#include <span>
#include <tuple>

using namespace std;
using namespace std::chrono_literals;
using namespace bt::advert;

namespace nevermore::sensorium::gatt {

namespace {

// coincidentally packed b/c all `bt::advert` funcs return only tuples of packed members
constexpr tuple ADVERT{
        flags({
                Flag::LE_DISCOVERABLE,
                Flag::EDR_NOT_SUPPORTED,
        }),
        shortened_local_name("Nevermore"),
};

void hci_handler(uint8_t packet_type, [[maybe_unused]] uint16_t channel, uint8_t* packet,
        [[maybe_unused]] uint16_t size) {
    if (packet_type != HCI_EVENT_PACKET) return;

    auto const event_type = hci_event_packet_get_type(packet);
    switch (event_type) {
    case BTSTACK_EVENT_STATE: {
        if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING) break;

        bd_addr_t local_addr;
        gap_local_bd_addr(local_addr);
        printf("BLE GATT - ready; address is %s\n", bd_addr_to_str(local_addr));

        // setup advertisements
        static_assert(sizeof(ADVERT) <= 31, "too large for non-extended advertisement");
        gap_advertisements_set_params(ADVERTISE_INTERVAL_MIN, ADVERTISE_INTERVAL_MAX);
        gap_advertisements_set_data(sizeof(ADVERT), (uint8_t*)&ADVERT);  // NOLINT
        gap_advertisements_enable(1);
    } break;

    case ATT_EVENT_CONNECTED: {
        auto const conn = att_event_connected_get_handle(packet);
        printf("BLE GATT - connected conn=%d\n", conn);
    } break;

    case ATT_EVENT_DISCONNECTED: {
        auto const conn = att_event_disconnected_get_handle(packet);
        printf("BLE GATT - disconnected conn=%d\n", conn);
    } break;
    }
}

optional<uint16_t> attr_read(hci_con_handle_t conn, uint16_t attr, uint16_t offset, span<uint8_t> buffer) {
    switch (attr) {
    default: {
        printf("WARN - BLE GATT - attr_read unhandled attr 0x%04x\n", int(attr));
        return {};
    }
    }
}

int attr_write(hci_con_handle_t conn, uint16_t attr, uint16_t offset, span<uint8_t const> buffer) {
    // FUTURE WORK: support offset'd writes to attributes
    if (offset != 0) return ATT_ERROR_REQUEST_NOT_SUPPORTED;

    switch (attr) {
    case ATT_CHARACTERISTIC_f48a18bb_e03c_4583_8006_5b54422e2045_01_VALUE_HANDLE: {
        if (buffer.size() != 1) return ATT_ERROR_VALUE_NOT_ALLOWED;
        if (2 <= buffer[0]) return ATT_ERROR_VALUE_NOT_ALLOWED;

        printf("!! GATT - Reboot Requested; OTA=%d\n", int(buffer[0]));
        picowota_reboot(buffer[0] == 1);
        return 0;
    }
    default: {
        printf("WARN - BLE GATT - attr_write unhandled attr 0x%04x\n", int(attr));
        return ATT_ERROR_INVALID_HANDLE;
    }
    }
}

uint16_t attr_read(
        hci_con_handle_t conn, uint16_t attr, uint16_t offset, uint8_t* buffer, uint16_t buffer_size) {
    assert(buffer || buffer_size == 0);
    auto result = attr_read(
            // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
            conn, attr, offset, buffer ? span<uint8_t>{buffer, buffer + buffer_size} : span<uint8_t>{});
    return result.value_or(ATT_READ_ERROR_CODE_OFFSET + ATT_ERROR_INVALID_HANDLE);
}

int attr_write(hci_con_handle_t conn, uint16_t attr, uint16_t transaction_mode, uint16_t offset,
        uint8_t* buffer, uint16_t buffer_size) {
    // `attr == 0` is an invalid handle, but the combination of `attr == 0` and a cancel transaction means
    // 'drop everything pending, the other side has disconnected'.
    // For us, this means a no-op; we don't support prepared writes so there's nothing to cancel.
    if (attr == 0 && transaction_mode == ATT_TRANSACTION_MODE_CANCEL) return 0;

    // We don't support prepared writes.
    if (transaction_mode != ATT_TRANSACTION_MODE_NONE) {
        printf("WARN - BLE GATT - attr_write unhandled transaction mode 0x%04x w/ attr=%d (conn=%d)\n",
                transaction_mode, attr, conn);
        return transaction_mode == ATT_TRANSACTION_MODE_CANCEL ? 0 : ATT_ERROR_WRITE_REQUEST_REJECTED;
    }

    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    return attr_write(conn, attr, offset, {buffer, buffer + buffer_size});
}

btstack_packet_callback_registration_t g_hci_handler{.callback = &hci_handler};

}  // namespace

bool init() {
    if constexpr (NEVERMORE_PICO_W_BT) {
#ifndef ENABLE_LOG_DEBUG
        hci_dump_enable_packet_log(false);  // don't spam out packet logs unless we're low level debugging
#endif

        l2cap_init();
        sm_init();  // FUTURE WORK: do we even need a security manager? can we ditch this?
    }

    if constexpr (NEVERMORE_PICO_W_BT) {
        hci_add_event_handler(&g_hci_handler);

        att_server_init(profile_data, attr_read, attr_write);
        att_server_register_packet_handler(hci_handler);

        gap_set_max_number_peripheral_connections(MAX_NR_HCI_CONNECTIONS);

        // turn on bluetooth
        if (auto err = hci_power_control(HCI_POWER_ON)) {
            printf("hci_power_control failed = 0x%08x\n", err);
            return false;
        }
    }

    return true;
}

}  // namespace nevermore::sensorium::gatt
