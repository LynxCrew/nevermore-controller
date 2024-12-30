#pragma once

#include "bluetooth.h"
#include "gatt.hpp"
#include <cstdint>
#include <optional>
#include <span>

namespace nevermore::gatt::device_info {

std::optional<uint16_t> attr_read(hci_con_handle_t, uint16_t attr, std::span<uint8_t>);
std::optional<int> attr_write(hci_con_handle_t, uint16_t attr, std::span<uint8_t const>);

bool init();
void disconnected(hci_con_handle_t);

}  // namespace nevermore::gatt::device_info
