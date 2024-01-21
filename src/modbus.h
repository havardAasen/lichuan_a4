/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (C) 2023-2024 Håvard F. Aasen <havard.f.aasen@pfft.no>
 */

/**
 * @file
 * @brief Wrapper for the C Modbus library.
 */

#ifndef LICHUAN_A4_MODBUS_H
#define LICHUAN_A4_MODBUS_H

#include <modbus.h>

#include <string>
#include <utility>
#include <vector>


class Modbus {
public:
    Modbus(const std::string &device, int baud_rate, int data_bits, char parity, int stop_bits,
           int target, bool debug = false);
    Modbus(const Modbus&) = delete;
    Modbus& operator=(const Modbus&) = delete;
    Modbus(Modbus&& other) noexcept : mb_ctx{std::exchange(other.mb_ctx, nullptr)} {};
    Modbus& operator=(Modbus&& other) noexcept;
    ~Modbus();

    /**
     * @brief Write a single value to Modbus register.
     *
     * Modbus function code 0x06 (preset single register).
     * @param address Modbus register address.
     * @param value Value to write.
     * @return @c true on successful write, otherwise @c false.
     */
    bool write_register(int address, uint16_t value);

    /**
     * @brief Read Modbus registers.
     *
     * Modbus function code 0x03 (read holding registers).
     * @param address Modbus register address.
     * @param count Number of registers to read.
     * @return On success, the received data, otherwise empty container.
     */
    [[nodiscard]] std::vector<uint16_t> read_registers(int address, int count) const;

private:
    modbus_t* mb_ctx;
};


#endif // LICHUAN_A4_MODBUS_H
