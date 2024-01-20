/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (C) 2022-2024 Håvard F. Aasen <havard.f.aasen@pfft.no>
 */

#include "lichuan_a4.h"

#include <bitset>
#include <iostream>
#include <string>


Lichuan_a4::Lichuan_a4(std::string_view _hal_name, std::string_view _device, int _target, int _baud_rate, bool _verbose)
    : hal_name{_hal_name}
    , target{_target}
    , baud_rate{_baud_rate}
    , verbose{_verbose}
    , device{_device}
    , hal{hal_name}
    , mb_ctx(device, baud_rate, data_bits, parity, stop_bits, target, verbose)
{}

void Lichuan_a4::read_data()
{
    read_speed_data();
    read_torque_data();
    read_digital_IO();
    update_internal_state();
}

Error_code Lichuan_a4::get_current_error() const noexcept
{
    return error_code;
}

constexpr std::string_view Lichuan_a4::get_error_message(Error_code error_code) noexcept
{
    switch (error_code) {
        case Error_code::system_error: return "system error";
        case Error_code::DI_configuration_error: return "DI configuration error";
        case Error_code::communication_error: return "communication error";
        case Error_code::control_power_is_off: return "control power is off";
        case Error_code::FPGA_internal_error: return "FPGA internal error";
        case Error_code::zeroing_timeout: return "zeroing timeout";
        case Error_code::overvoltage: return "overvoltage";
        case Error_code::undervoltage: return "undervoltage";
        case Error_code::overcurrent_and_grounding_errors: return "overcurrent and grounding errors";
        case Error_code::over_heating: return "over heating";
        case Error_code::excessive_load: return "excessive load";
        case Error_code::regen_discharge_resistance_overload: return "regenerative discharge resistance overload";
        case Error_code::encoder_error: return "encoder error";
        case Error_code::excessive_position_deviation: return "excessive position deviation";
        case Error_code::overspeed: return "overspeed";
        case Error_code::command_pulse_division_frequency: return "command pulse division frequency";
        case Error_code::deviation_counter_overflow: return "deviation counter overflow";
        case Error_code::EEPROM_parameter_error: return "EEPROM parameter error";
        case Error_code::stroke_limit_input_signal: return "stroke limit input signal";
        case Error_code::analog_command_overvoltage: return "analog command overvoltage";
        case Error_code::no_error: return "";
        default: return "unknown error code";
    }
}

void Lichuan_a4::read_speed_data()
{
    for (int retries = 0; retries < modbus_retries; retries++) {
        const auto data = mb_ctx.read_registers(speed_start_reg, speed_reg_count);

        if (data.size() == speed_reg_count) {
            // Speed values can be negative.
            *hal.data->commanded_speed = static_cast<int16_t>(data[0]);
            *hal.data->feedback_speed = static_cast<int16_t>(data[1]);
            *hal.data->deviation_speed = static_cast<int16_t>(data[2]);
            return;
        }
        hal.data->modbus_errors++;
    }
}

void Lichuan_a4::read_torque_data()
{
    for (int retries = 0; retries < modbus_retries; retries++) {
        const auto data = mb_ctx.read_registers(torque_start_reg, torque_reg_count);

        if (data.size() == torque_reg_count) {
            *hal.data->commanded_torque = data[0] / 10.0;
            *hal.data->feedback_torque = data[1] / 10.0;
            *hal.data->deviation_torque = data[2] / 10.0;
            return;
        }
        hal.data->modbus_errors++;
    }
}

void Lichuan_a4::read_digital_IO()
{
    for (int retries = 0; retries < modbus_retries; retries++) {
        const auto data = mb_ctx.read_registers(digital_IO_start_reg, digital_IO_reg_count);

        if (data.size() == digital_IO_reg_count) {
            const std::bitset<8> bits_in{data[0]};
            *hal.data->digital_in0 = bits_in[0];
            *hal.data->digital_in1 = bits_in[1];
            *hal.data->digital_in2 = bits_in[2];
            *hal.data->digital_in3 = bits_in[3];
            *hal.data->digital_in4 = bits_in[4];
            *hal.data->digital_in5 = bits_in[5];
            *hal.data->digital_in6 = bits_in[6];
            *hal.data->digital_in7 = bits_in[7];

            const std::bitset<8> bits_out{data[1]};
            *hal.data->digital_out0 = bits_out[0];
            *hal.data->digital_out1 = bits_out[1];
            *hal.data->digital_out2 = bits_out[2];
            *hal.data->digital_out3 = bits_out[3];
            *hal.data->digital_out4 = bits_out[4];
            *hal.data->digital_out5 = bits_out[5];
            return;
        }
        hal.data->modbus_errors++;
    }
}

void Lichuan_a4::update_internal_state()
{
    if (*hal.data->digital_out1) {
        read_error_code();
        print_error_message();
    } else {
        error_code = Error_code::no_error;
    }
}

void Lichuan_a4::read_error_code()
{
    for (int retries = 0; retries < modbus_retries; retries++) {
        const auto data = mb_ctx.read_registers(current_error_code_reg, single_register_count);

        if (data.size() == single_register_count) {
            *hal.data->error_code = data[0];
            return;
        }
        hal.data->modbus_errors++;
    }
}

void Lichuan_a4::print_error_message()
{
    auto current_error = static_cast<Error_code>(*hal.data->error_code);
    // Don't print error message multiple times.
    if (current_error == error_code)
        return;

    error_code = current_error;
    std::string_view message = get_error_message(error_code);
    if (message.empty())
        return;
    std::cerr << hal_name << ": ERROR: " << *hal.data->error_code << "\n\t" << message << "\n";
}
