/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (C) 2022-2024 Håvard F. Aasen <havard.f.aasen@pfft.no>
 */

#include "lichuan_a4.h"

#include <cerrno>
#include <iostream>
#include <string>
#include <sstream>


Lichuan_a4::Lichuan_a4(Target_data& _target) : target{ _target }
{
    std::cout << target.hal_name << ": device='" << target.device << "', baud=" << target.baud_rate
              << ", bits=" << data_bits << ", parity='" << parity << "', stopbits="
              << stop_bits << ", target=" << target.target << "\n";

    mb_ctx = modbus_new_rtu(target.device.c_str(), target.baud_rate, parity, data_bits, stop_bits);
    if (!mb_ctx) {
        std::ostringstream oss;
        oss << target.hal_name << ": ERROR: Can't open modbus serial device: "
            << modbus_strerror(errno);
        throw std::runtime_error(oss.str());
    }

    if (modbus_connect(mb_ctx) != 0) {
        modbus_free(mb_ctx);
        std::ostringstream oss;
        oss << target.hal_name << ": ERROR: Can't connect to serial device:"
            << modbus_strerror(errno) << "\n";
        throw std::runtime_error(oss.str());
    }

    modbus_set_debug(mb_ctx, target.verbose);
    modbus_set_slave(mb_ctx, target.target);

    /* Create HAL component */
    target.hal_comp_id = hal_init(target.hal_name.c_str());
    if (target.hal_comp_id < 0) {
        modbus_close(mb_ctx);
        modbus_free(mb_ctx);
        std::ostringstream oss;
        oss << target.hal_name << ": ERROR: hal_init() failed\n";
        throw std::runtime_error(oss.str());
    }

    hal = static_cast<struct Hal_data*>(hal_malloc(sizeof(struct Hal_data)));
    if (!hal) {
        modbus_close(mb_ctx);
        modbus_free(mb_ctx);
        std::ostringstream oss;
        oss << target.hal_name << ": ERROR: Unable to allocate shared memory\n";
        throw std::runtime_error(oss.str());
    }

    if (create_hal_pins()) {
        modbus_close(mb_ctx);
        modbus_free(mb_ctx);
        hal_exit(target.hal_comp_id);
        std::ostringstream oss;
        oss << target.hal_name << ": ERROR: create_hal_pins() failed\n";
        throw std::runtime_error(oss.str());
    }

    initialize_haldata();
    hal_ready(target.hal_comp_id);
}

Lichuan_a4::~Lichuan_a4()
{
    modbus_close(mb_ctx);
    modbus_free(mb_ctx);
    hal_exit(target.hal_comp_id);
}

int Lichuan_a4::read_data()
{
    uint16_t receive_data[MODBUS_MAX_READ_REGISTERS];

    for (int retries = 0; retries < modbus_retries; retries++) {
        int retval = modbus_read_registers(mb_ctx, target.read_reg_start,
                                           target.read_reg_count, receive_data);

        if (retval == target.read_reg_count) {
            *hal->commanded_speed = receive_data[0];
            *hal->feedback_speed = receive_data[1];
            *hal->deviation_speed = receive_data[2];
            *hal->dc_bus_volt = receive_data[9];
            *hal->torque_load = receive_data[10];
            *hal->res_braking = receive_data[11];
            *hal->torque_overload = receive_data[12];
            return 0;
        }
        std::cerr << target.hal_name << ": ERROR reading data for "
                  << target.read_reg_count << " registers, from register "
                  << target.read_reg_start << ": " << modbus_strerror(errno) << "\n";
        hal->modbus_errors++;
    }

    return -1;
}

int Lichuan_a4::create_hal_pins() const noexcept
{
    const int id = target.hal_comp_id;
    const char *hal_name = target.hal_name.c_str();

    int retval = hal_pin_float_newf(HAL_OUT, &hal->commanded_speed, id, "%s.commanded-speed", hal_name);
    if (retval != 0) return -1;
    retval = hal_pin_float_newf(HAL_OUT, &hal->feedback_speed, id, "%s.feedback-speed", hal_name);
    if (retval != 0) return -1;
    retval = hal_pin_float_newf(HAL_OUT, &hal->deviation_speed, id, "%s.deviation-speed", hal_name);
    if (retval != 0) return -1;
    retval = hal_pin_float_newf(HAL_OUT, &hal->dc_bus_volt, id, "%s.dc-bus-volt", hal_name);
    if (retval != 0) return -1;
    retval = hal_pin_float_newf(HAL_OUT, &hal->torque_load, id, "%s.torque-load", hal_name);
    if (retval != 0) return -1;
    retval = hal_pin_float_newf(HAL_OUT, &hal->res_braking, id, "%s.res-braking", hal_name);
    if (retval != 0) return -1;
    retval = hal_pin_float_newf(HAL_OUT, &hal->torque_overload, id, "%s.torque-overload", hal_name);
    if (retval != 0) return -1;

    // FIXME: The modbus_polling pin should be shared between all devices.
    retval = hal_param_float_newf(HAL_RW, &hal->modbus_polling, id, "%s.modbus-polling", hal_name);
    if (retval != 0) return -1;
    retval = hal_param_s32_newf(HAL_RO, &hal->modbus_errors, id, "%s.modbus-errors", hal_name);

    return retval;
}

constexpr void Lichuan_a4::initialize_haldata() const noexcept
{
    *hal->commanded_speed = 0;
    *hal->feedback_speed = 0;
    *hal->deviation_speed = 0;
    *hal->dc_bus_volt = 0;
    *hal->torque_load = 0;
    *hal->res_braking = 0;
    *hal->torque_overload = 0;

    hal->modbus_polling = 1.0;
    hal->modbus_errors = 0;
}
