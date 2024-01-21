/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (C) 2023-2024 Håvard F. Aasen <havard.f.aasen@pfft.no>
 */

#include "modbus.h"

#include <cerrno>
#include <iostream>
#include <sstream>

Modbus::Modbus(const std::string &device, const int baud_rate, const int data_bits,
               const char parity, const int stop_bits, const int target, const bool debug)
{
    std::cout << "Modbus RTU: device='" << device << "', baud=" << baud_rate
              << ", data bits=" << data_bits << ", parity='" << parity << "', stop bits="
              << stop_bits << ", target=" << target << "\n";

    mb_ctx = modbus_new_rtu(device.c_str(), baud_rate, parity, data_bits, stop_bits);
    if (!mb_ctx) {
        std::ostringstream oss;
        oss << "ERROR: Can't open modbus serial device: "
            << modbus_strerror(errno);
        throw std::runtime_error(oss.str());
    }

    if (modbus_connect(mb_ctx) != 0) {
        modbus_free(mb_ctx);
        std::ostringstream oss;
        oss << "ERROR: Can't connect to serial device:"
            << modbus_strerror(errno) << "\n";
        throw std::runtime_error(oss.str());
    }

    modbus_set_debug(mb_ctx, debug);
    modbus_set_slave(mb_ctx, target);
}

Modbus& Modbus::operator=(Modbus&& other) noexcept
{
    mb_ctx = std::exchange(other.mb_ctx, nullptr);
    return *this;
}

Modbus::~Modbus()
{
    if (mb_ctx) {
        modbus_close(mb_ctx);
        modbus_free(mb_ctx);
    }
}

std::vector<uint16_t> Modbus::read_registers(const int address, const int count) const
{
    // Modbus requires an array to read into, but we want to return a vector.
    std::vector<uint16_t> data{};
    if (count > MODBUS_MAX_READ_REGISTERS || count < 1) {
        return data;
    }

    uint16_t data_temp[static_cast<unsigned int>(count)];
    int retval = modbus_read_registers(mb_ctx, address, count, data_temp);
    if (retval == count) {
        data.reserve(static_cast<size_t>(count));
        data.insert(data.end(), data_temp, data_temp + count);
        return data;
    }
    std::cerr << "Modbus RTU: ERROR reading data for " << count << " registers, from register "
              << address << ": " << modbus_strerror(errno) << "\n";
    return data;
}

bool Modbus::write_register(const int address, const uint16_t value)
{
    return modbus_write_register(mb_ctx, address, value);
}
