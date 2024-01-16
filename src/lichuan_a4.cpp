/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (C) 2022-2024 Håvard F. Aasen <havard.f.aasen@pfft.no>
 */

#include "lichuan_a4.h"

#include <bitset>
#include <iostream>
#include <string>
#include <sstream>


Lichuan_a4::Lichuan_a4(Target_data _target)
    : target{std::move(_target)}
    , mb_ctx(target.device, target.baud_rate, data_bits, parity, stop_bits, target.target, target.verbose)
{
    target.hal_comp_id = hal_init(target.hal_name.c_str());
    if (target.hal_comp_id < 0) {
        std::ostringstream oss;
        oss << target.hal_name << ": ERROR: hal_init() failed\n";
        throw std::runtime_error(oss.str());
    }

    hal = static_cast<Hal_data*>(hal_malloc(sizeof(Hal_data)));
    if (!hal) {
        std::ostringstream oss;
        oss << target.hal_name << ": ERROR: Unable to allocate shared memory\n";
        throw std::runtime_error(oss.str());
    }

    if (create_hal_pins()) {
        hal_exit(target.hal_comp_id);
        std::ostringstream oss;
        oss << target.hal_name << ": ERROR: create_hal_pins() failed\n";
        throw std::runtime_error(oss.str());
    }

    Hal_data::initialize_data(hal);
    hal_ready(target.hal_comp_id);
}

Lichuan_a4::~Lichuan_a4()
{
    int ret = hal_exit(target.hal_comp_id);
    if (ret < 0) {
        std::cerr << target.hal_name << ": ERROR: hal_exit() failed with code " << ret << "\n";
    }
}

void Lichuan_a4::read_data()
{
    read_speed_data();
    read_torque_data();
    read_digital_IO();
}

void Lichuan_a4::read_speed_data()
{
    for (int retries = 0; retries < modbus_retries; retries++) {
        const auto data = mb_ctx.readRegisters(speed_start_reg, speed_reg_count);

        if (data.size() == speed_reg_count) {
            // Speed values can be negative.
            *hal->commanded_speed = static_cast<int16_t>(data[0]);
            *hal->feedback_speed = static_cast<int16_t>(data[1]);
            *hal->deviation_speed = static_cast<int16_t>(data[2]);
            return;
        }
        hal->modbus_errors++;
    }
}

void Lichuan_a4::read_torque_data()
{
    for (int retries = 0; retries < modbus_retries; retries++) {
        const auto data = mb_ctx.readRegisters(torque_start_reg, torque_reg_count);

        if (data.size() == torque_reg_count) {
            *hal->commanded_torque = data[0] / 10.0;
            *hal->feedback_torque = data[1] / 10.0;
            *hal->deviation_torque = data[2] / 10.0;
            return;
        }
        hal->modbus_errors++;
    }
}

void Lichuan_a4::read_digital_IO()
{
    for (int retries = 0; retries < modbus_retries; retries++) {
        const auto data = mb_ctx.readRegisters(digital_IO_start_reg, digital_IO_reg_count);

        if (data.size() == digital_IO_reg_count) {
            const std::bitset<8> bits_in{data[0]};
            *hal->digital_in0 = bits_in[0];
            *hal->digital_in1 = bits_in[1];
            *hal->digital_in2 = bits_in[2];
            *hal->digital_in3 = bits_in[3];
            *hal->digital_in4 = bits_in[4];
            *hal->digital_in5 = bits_in[5];
            *hal->digital_in6 = bits_in[6];
            *hal->digital_in7 = bits_in[7];

            const std::bitset<8> bits_out{data[1]};
            *hal->digital_out0 = bits_out[0];
            *hal->digital_out1 = bits_out[1];
            *hal->digital_out2 = bits_out[2];
            *hal->digital_out3 = bits_out[3];
            *hal->digital_out4 = bits_out[4];
            *hal->digital_out5 = bits_out[5];
            return;
        }
        hal->modbus_errors++;
    }
}

int Lichuan_a4::create_hal_pins() const noexcept
{
    const int id = target.hal_comp_id;
    const char *hal_name = target.hal_name.c_str();

    if (hal_pin_float_newf(HAL_OUT, &hal->commanded_speed, id, "%s.commanded-speed", hal_name) != 0) return -1;
    if (hal_pin_float_newf(HAL_OUT, &hal->feedback_speed, id, "%s.feedback-speed", hal_name) != 0) return -1;
    if (hal_pin_float_newf(HAL_OUT, &hal->deviation_speed, id, "%s.deviation-speed", hal_name) != 0) return -1;
    if (hal_pin_float_newf(HAL_OUT, &hal->commanded_torque, id, "%s.commanded-torque", hal_name) != 0) return -1;
    if (hal_pin_float_newf(HAL_OUT, &hal->feedback_torque, id, "%s.feedback-torque", hal_name) != 0) return -1;
    if (hal_pin_float_newf(HAL_OUT, &hal->deviation_torque, id, "%s.deviation-torque", hal_name) != 0) return -1;
    if (hal_pin_float_newf(HAL_OUT, &hal->dc_bus_volt, id, "%s.dc-bus-volt", hal_name) != 0) return -1;
    if (hal_pin_float_newf(HAL_OUT, &hal->torque_load, id, "%s.torque-load", hal_name) != 0) return -1;
    if (hal_pin_float_newf(HAL_OUT, &hal->res_braking, id, "%s.res-braking", hal_name) != 0) return -1;
    if (hal_pin_float_newf(HAL_OUT, &hal->torque_overload, id, "%s.torque-overload", hal_name) != 0) return -1;

    if (hal_pin_bit_newf(HAL_OUT, &hal->digital_in0, id, "%s.servo-enabling", hal_name) != 0) return -1;
    if (hal_pin_bit_newf(HAL_OUT, &hal->digital_in1, id, "%s.clear-alarm", hal_name) != 0) return -1;
    if (hal_pin_bit_newf(HAL_OUT, &hal->digital_in2, id, "%s.clockwise-stroke-limit", hal_name) != 0) return -1;
    if (hal_pin_bit_newf(HAL_OUT, &hal->digital_in3, id, "%s.anticlockwise-stroke-limit", hal_name) != 0) return -1;
    if (hal_pin_bit_newf(HAL_OUT, &hal->digital_in4, id, "%s.clear-deviation-counter", hal_name) != 0) return -1;
    if (hal_pin_bit_newf(HAL_OUT, &hal->digital_in5, id, "%s.pulse-prohibition", hal_name) != 0) return -1;
    if (hal_pin_bit_newf(HAL_OUT, &hal->digital_in6, id, "%s.torque-limit-switchover", hal_name) != 0) return -1;
    if (hal_pin_bit_newf(HAL_OUT, &hal->digital_in7, id, "%s.homing", hal_name) != 0) return -1;

    if (hal_pin_bit_newf(HAL_OUT, &hal->digital_out0, id, "%s.servo-ready", hal_name) != 0) return -1;
    if (hal_pin_bit_newf(HAL_OUT, &hal->digital_out1, id, "%s.active-alarm", hal_name) != 0) return -1;
    if (hal_pin_bit_newf(HAL_OUT, &hal->digital_out2, id, "%s.location-arrival", hal_name) != 0) return -1;
    if (hal_pin_bit_newf(HAL_OUT, &hal->digital_out3, id, "%s.brake", hal_name) != 0) return -1;
    if (hal_pin_bit_newf(HAL_OUT, &hal->digital_out4, id, "%s.zero-speed", hal_name) != 0) return -1;
    if (hal_pin_bit_newf(HAL_OUT, &hal->digital_out5, id, "%s.torque-limiting", hal_name) != 0) return -1;

    // FIXME: If multiple devices, the 'modbus_polling' pin should be shared between all devices.
    if (hal_param_float_newf(HAL_RW, &hal->modbus_polling, id, "%s.modbus-polling", hal_name) != 0) return -1;
    if (hal_param_u32_newf(HAL_RO, &hal->modbus_errors, id, "%s.modbus-errors", hal_name) != 0) return -1;

    return 0;
}

constexpr void Hal_data::initialize_data(Hal_data *hal) noexcept
{
    *hal->commanded_speed = 0;
    *hal->feedback_speed = 0;
    *hal->deviation_speed = 0;
    *hal->commanded_torque = 0;
    *hal->feedback_torque = 0;
    *hal->deviation_torque = 0;
    *hal->dc_bus_volt = 0;
    *hal->torque_load = 0;
    *hal->res_braking = 0;
    *hal->torque_overload = 0;

    *hal->digital_in0 = false;
    *hal->digital_in1 = false;
    *hal->digital_in2 = false;
    *hal->digital_in3 = false;
    *hal->digital_in4 = false;
    *hal->digital_in5 = false;
    *hal->digital_in6 = false;
    *hal->digital_in7 = false;
    *hal->digital_out0 = false;
    *hal->digital_out1 = false;
    *hal->digital_out2 = false;
    *hal->digital_out3 = false;
    *hal->digital_out4 = false;
    *hal->digital_out5 = false;

    hal->modbus_polling = 1.0;
    hal->modbus_errors = 0;
}
