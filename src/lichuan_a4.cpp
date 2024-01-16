/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (C) 2022-2024 Håvard F. Aasen <havard.f.aasen@pfft.no>
 */

#include "lichuan_a4.h"

#include <bitset>
#include <iostream>
#include <string>
#include <sstream>


Lichuan_a4::Lichuan_a4(std::string_view _hal_name, std::string_view _device, int _target, int _baud_rate, bool _verbose)
    : hal_name{_hal_name}
    , target{_target}
    , baud_rate{_baud_rate}
    , device{_device}
    , verbose{_verbose}
    , mb_ctx(device, baud_rate, data_bits, parity, stop_bits, target, verbose)
{
    hal_comp_id = hal_init(hal_name.c_str());
    if (hal_comp_id < 0) {
        std::ostringstream oss;
        oss << hal_name << ": ERROR: hal_init() failed\n";
        throw std::runtime_error(oss.str());
    }

    hal = static_cast<Hal_data*>(hal_malloc(sizeof(Hal_data)));
    if (!hal) {
        std::ostringstream oss;
        oss << hal_name << ": ERROR: Unable to allocate shared memory\n";
        throw std::runtime_error(oss.str());
    }

    if (create_hal_pins()) {
        hal_exit(hal_comp_id);
        std::ostringstream oss;
        oss << hal_name << ": ERROR: create_hal_pins() failed\n";
        throw std::runtime_error(oss.str());
    }

    Hal_data::initialize_data(hal);
    hal_ready(hal_comp_id);
}

Lichuan_a4::~Lichuan_a4()
{
    int ret = hal_exit(hal_comp_id);
    if (ret < 0) {
        std::cerr << hal_name << ": ERROR: hal_exit() failed with code " << ret << "\n";
    }
}

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

void Lichuan_a4::update_internal_state()
{
    if (*hal->digital_out1) {
        read_error_code();
        print_error_message();
    } else {
        error_code = Error_code::no_error;
    }
}

void Lichuan_a4::read_error_code()
{
    for (int retries = 0; retries < modbus_retries; retries++) {
        const auto data = mb_ctx.readRegisters(current_error_code_reg, single_register_count);

        if (data.size() == single_register_count) {
            *hal->error_code = data[0];
            return;
        }
        hal->modbus_errors++;
    }
}

void Lichuan_a4::print_error_message()
{
    auto current_error = static_cast<Error_code>(*hal->error_code);
    // Don't print error message multiple times.
    if (current_error == error_code)
        return;

    error_code = current_error;
    std::string_view message = get_error_message(error_code);
    if (message.empty())
        return;
    std::cerr << hal_name << ": ERROR: " << *hal->error_code << "\n\t" << message << "\n";
}

int Lichuan_a4::create_hal_pins() const noexcept
{
    const char *name = this->hal_name.c_str();

    if (hal_pin_float_newf(HAL_OUT, &hal->commanded_speed, hal_comp_id, "%s.commanded-speed", name) != 0) return -1;
    if (hal_pin_float_newf(HAL_OUT, &hal->feedback_speed, hal_comp_id, "%s.feedback-speed", name) != 0) return -1;
    if (hal_pin_float_newf(HAL_OUT, &hal->deviation_speed, hal_comp_id, "%s.deviation-speed", name) != 0) return -1;
    if (hal_pin_float_newf(HAL_OUT, &hal->commanded_torque, hal_comp_id, "%s.commanded-torque", name) != 0) return -1;
    if (hal_pin_float_newf(HAL_OUT, &hal->feedback_torque, hal_comp_id, "%s.feedback-torque", name) != 0) return -1;
    if (hal_pin_float_newf(HAL_OUT, &hal->deviation_torque, hal_comp_id, "%s.deviation-torque", name) != 0) return -1;
    if (hal_pin_float_newf(HAL_OUT, &hal->dc_bus_volt, hal_comp_id, "%s.dc-bus-volt", name) != 0) return -1;
    if (hal_pin_float_newf(HAL_OUT, &hal->torque_load, hal_comp_id, "%s.torque-load", name) != 0) return -1;
    if (hal_pin_float_newf(HAL_OUT, &hal->res_braking, hal_comp_id, "%s.res-braking", name) != 0) return -1;
    if (hal_pin_float_newf(HAL_OUT, &hal->torque_overload, hal_comp_id, "%s.torque-overload", name) != 0) return -1;
    if (hal_pin_s32_newf(HAL_OUT, &hal->error_code, hal_comp_id, "%s.error-code", name) != 0) return -1;

    if (hal_pin_bit_newf(HAL_OUT, &hal->digital_in0, hal_comp_id, "%s.servo-enabling", name) != 0) return -1;
    if (hal_pin_bit_newf(HAL_OUT, &hal->digital_in1, hal_comp_id, "%s.clear-alarm", name) != 0) return -1;
    if (hal_pin_bit_newf(HAL_OUT, &hal->digital_in2, hal_comp_id, "%s.clockwise-stroke-limit", name) != 0) return -1;
    if (hal_pin_bit_newf(HAL_OUT, &hal->digital_in3, hal_comp_id, "%s.anticlockwise-stroke-limit", name) != 0) return -1;
    if (hal_pin_bit_newf(HAL_OUT, &hal->digital_in4, hal_comp_id, "%s.clear-deviation-counter", name) != 0) return -1;
    if (hal_pin_bit_newf(HAL_OUT, &hal->digital_in5, hal_comp_id, "%s.pulse-prohibition", name) != 0) return -1;
    if (hal_pin_bit_newf(HAL_OUT, &hal->digital_in6, hal_comp_id, "%s.torque-limit-switchover", name) != 0) return -1;
    if (hal_pin_bit_newf(HAL_OUT, &hal->digital_in7, hal_comp_id, "%s.homing", name) != 0) return -1;

    if (hal_pin_bit_newf(HAL_OUT, &hal->digital_out0, hal_comp_id, "%s.servo-ready", name) != 0) return -1;
    if (hal_pin_bit_newf(HAL_OUT, &hal->digital_out1, hal_comp_id, "%s.active-alarm", name) != 0) return -1;
    if (hal_pin_bit_newf(HAL_OUT, &hal->digital_out2, hal_comp_id, "%s.location-arrival", name) != 0) return -1;
    if (hal_pin_bit_newf(HAL_OUT, &hal->digital_out3, hal_comp_id, "%s.brake", name) != 0) return -1;
    if (hal_pin_bit_newf(HAL_OUT, &hal->digital_out4, hal_comp_id, "%s.zero-speed", name) != 0) return -1;
    if (hal_pin_bit_newf(HAL_OUT, &hal->digital_out5, hal_comp_id, "%s.torque-limiting", name) != 0) return -1;

    // FIXME: If multiple devices, the 'modbus_polling' pin should be shared between all devices.
    if (hal_param_float_newf(HAL_RW, &hal->modbus_polling, hal_comp_id, "%s.modbus-polling", name) != 0) return -1;
    if (hal_param_u32_newf(HAL_RO, &hal->modbus_errors, hal_comp_id, "%s.modbus-errors", name) != 0) return -1;

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
    *hal->error_code = 0;

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
