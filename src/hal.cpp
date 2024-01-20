/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (C) 2024 Håvard F. Aasen <havard.f.aasen@pfft.no>
 */

#include "hal.h"

#include <iostream>
#include <sstream>


HAL::HAL(std::string_view _hal_name) : hal_name{_hal_name}
{
    hal_comp_id = hal_init(hal_name.c_str());
    if (hal_comp_id < 0) {
        std::ostringstream oss;
        oss << hal_name << ": ERROR: hal_init() failed\n";
        throw std::runtime_error(oss.str());
    }

    data = static_cast<Data*>(hal_malloc(sizeof(Data)));
    if (!data) {
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

    initialize_data();
    hal_ready(hal_comp_id);
}

HAL::~HAL()
{
    int ret = hal_exit(hal_comp_id);
    if (ret < 0) {
        std::cerr << hal_name << ": ERROR: hal_exit() failed with code " << ret << "\n";
    }
}

bool HAL::create_hal_pins() const noexcept
{
    const char *name = this->hal_name.c_str();

    if (hal_pin_float_newf(HAL_OUT, &data->commanded_speed, hal_comp_id, "%s.commanded-speed", name) != 0) return false;
    if (hal_pin_float_newf(HAL_OUT, &data->feedback_speed, hal_comp_id, "%s.feedback-speed", name) != 0) return false;
    if (hal_pin_float_newf(HAL_OUT, &data->deviation_speed, hal_comp_id, "%s.deviation-speed", name) != 0) return false;
    if (hal_pin_float_newf(HAL_OUT, &data->commanded_torque, hal_comp_id, "%s.commanded-torque", name) != 0) return false;
    if (hal_pin_float_newf(HAL_OUT, &data->feedback_torque, hal_comp_id, "%s.feedback-torque", name) != 0) return false;
    if (hal_pin_float_newf(HAL_OUT, &data->deviation_torque, hal_comp_id, "%s.deviation-torque", name) != 0) return false;
    if (hal_pin_float_newf(HAL_OUT, &data->dc_bus_volt, hal_comp_id, "%s.dc-bus-volt", name) != 0) return false;
    if (hal_pin_float_newf(HAL_OUT, &data->torque_load, hal_comp_id, "%s.torque-load", name) != 0) return false;
    if (hal_pin_float_newf(HAL_OUT, &data->res_braking, hal_comp_id, "%s.res-braking", name) != 0) return false;
    if (hal_pin_float_newf(HAL_OUT, &data->torque_overload, hal_comp_id, "%s.torque-overload", name) != 0) return false;
    if (hal_pin_s32_newf(HAL_OUT, &data->error_code, hal_comp_id, "%s.error-code", name) != 0) return false;

    if (hal_pin_bit_newf(HAL_OUT, &data->digital_in0, hal_comp_id, "%s.servo-enabling", name) != 0) return false;
    if (hal_pin_bit_newf(HAL_OUT, &data->digital_in1, hal_comp_id, "%s.clear-alarm", name) != 0) return false;
    if (hal_pin_bit_newf(HAL_OUT, &data->digital_in2, hal_comp_id, "%s.clockwise-stroke-limit", name) != 0) return false;
    if (hal_pin_bit_newf(HAL_OUT, &data->digital_in3, hal_comp_id, "%s.anticlockwise-stroke-limit", name) != 0) return false;
    if (hal_pin_bit_newf(HAL_OUT, &data->digital_in4, hal_comp_id, "%s.clear-deviation-counter", name) != 0) return false;
    if (hal_pin_bit_newf(HAL_OUT, &data->digital_in5, hal_comp_id, "%s.pulse-prohibition", name) != 0) return false;
    if (hal_pin_bit_newf(HAL_OUT, &data->digital_in6, hal_comp_id, "%s.torque-limit-switchover", name) != 0) return false;
    if (hal_pin_bit_newf(HAL_OUT, &data->digital_in7, hal_comp_id, "%s.homing", name) != 0) return false;

    if (hal_pin_bit_newf(HAL_OUT, &data->digital_out0, hal_comp_id, "%s.servo-ready", name) != 0) return false;
    if (hal_pin_bit_newf(HAL_OUT, &data->digital_out1, hal_comp_id, "%s.active-alarm", name) != 0) return false;
    if (hal_pin_bit_newf(HAL_OUT, &data->digital_out2, hal_comp_id, "%s.location-arrival", name) != 0) return false;
    if (hal_pin_bit_newf(HAL_OUT, &data->digital_out3, hal_comp_id, "%s.brake", name) != 0) return false;
    if (hal_pin_bit_newf(HAL_OUT, &data->digital_out4, hal_comp_id, "%s.zero-speed", name) != 0) return false;
    if (hal_pin_bit_newf(HAL_OUT, &data->digital_out5, hal_comp_id, "%s.torque-limiting", name) != 0) return false;

    // FIXME: If multiple devices, the 'modbus_polling' pin should be shared between all devices.
    if (hal_param_float_newf(HAL_RW, &data->modbus_polling, hal_comp_id, "%s.modbus-polling", name) != 0) return false;
    if (hal_param_u32_newf(HAL_RO, &data->modbus_errors, hal_comp_id, "%s.modbus-errors", name) != 0) return false;

    return true;
}

constexpr void HAL::initialize_data() noexcept
{
    *data->commanded_speed = 0;
    *data->feedback_speed = 0;
    *data->deviation_speed = 0;
    *data->commanded_torque = 0;
    *data->feedback_torque = 0;
    *data->deviation_torque = 0;
    *data->dc_bus_volt = 0;
    *data->torque_load = 0;
    *data->res_braking = 0;
    *data->torque_overload = 0;
    *data->error_code = 0;

    *data->digital_in0 = false;
    *data->digital_in1 = false;
    *data->digital_in2 = false;
    *data->digital_in3 = false;
    *data->digital_in4 = false;
    *data->digital_in5 = false;
    *data->digital_in6 = false;
    *data->digital_in7 = false;
    *data->digital_out0 = false;
    *data->digital_out1 = false;
    *data->digital_out2 = false;
    *data->digital_out3 = false;
    *data->digital_out4 = false;
    *data->digital_out5 = false;

    data->modbus_polling = 1.0;
    data->modbus_errors = 0;
}
