/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (C) 2022-2024 Håvard F. Aasen <havard.f.aasen@pfft.no>
 */

/**
 *  @file
 *  @brief A userspace program that interfaces the Lichuan A4 servo drive
 *         to the LinuxCNC HAL, using RS485 Modbus RTU.
 */


#ifndef LICHUAN_A4_H
#define LICHUAN_A4_H

#include "modbus.h"
#include "hal.h"

#include <string>

enum class Error_code {
    no_error = 0,
    system_error,
    DI_configuration_error,
    communication_error,
    control_power_is_off,
    FPGA_internal_error,
    zeroing_timeout,
    overvoltage = 12,
    undervoltage,
    overcurrent_and_grounding_errors,
    over_heating,
    excessive_load,
    regen_discharge_resistance_overload = 18,
    encoder_error = 21,
    excessive_position_deviation = 24,
    overspeed = 26,
    command_pulse_division_frequency,
    deviation_counter_overflow = 29,
    EEPROM_parameter_error = 36,
    stroke_limit_input_signal = 38,
    analog_command_overvoltage,
};


class Lichuan_a4 {
public:
    Lichuan_a4(std::string_view _hal_name, std::string_view _device, int _target, int _baud_rate = 19200, bool _verbose = false);

    void read_data();
    [[nodiscard]] Error_code get_current_error() const noexcept;
    [[nodiscard]] static constexpr std::string_view get_error_message(Error_code code) noexcept;

private:
    std::string hal_name;
    Error_code error_code{Error_code::no_error};
    int target; /*!< address of Modbus device to read from */
    int baud_rate;
    bool verbose;
    std::string device;
    HAL hal;
    Modbus mb_ctx;

    /** If a modbus transaction fails, retry this many times before giving up. */
    static constexpr int modbus_retries {5};

    // Modbus settings, hard-coded in servo driver
    static constexpr int data_bits {8};
    static constexpr int stop_bits {1};
    static constexpr char parity {'E'};

    static constexpr int current_error_code_reg {457};
    static constexpr int single_register_count {1};
    static constexpr int digital_IO_start_reg {466};
    static constexpr int digital_IO_reg_count {2};
    static constexpr int speed_start_reg {448};
    static constexpr int speed_reg_count {3};
    static constexpr int torque_start_reg {451};
    static constexpr int torque_reg_count {3};

    void read_speed_data();
    void read_torque_data();
    void read_digital_IO();
    void update_internal_state();
    void read_error_code();
    void print_error_message();
};

#endif // LICHUAN_A4_H
