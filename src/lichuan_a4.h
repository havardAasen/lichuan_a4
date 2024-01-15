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

#include <hal.h>

#include <string>


/** Target and registers to read from. */
struct Target_data {
    std::string hal_name{};
    int hal_comp_id{};

    // Modbus values
    std::string device{};
    int target{};               /*!< address of device to read from */
    int baud_rate{19200};
    bool verbose{false};
};

class Hal_data;


class Lichuan_a4 {
public:
    explicit Lichuan_a4(Target_data _target);
    ~Lichuan_a4();
    Lichuan_a4(const Lichuan_a4 &) = delete;
    Lichuan_a4 &operator=(const Lichuan_a4 &) = delete;

    void read_data();

private:
    Hal_data* hal{};
    Target_data target;
    Modbus mb_ctx;

    /** If a modbus transaction fails, retry this many times before giving up. */
    static constexpr int modbus_retries {5};

    // Modbus settings, hard-coded in servo driver
    static constexpr int data_bits {8};
    static constexpr int stop_bits {1};
    static constexpr char parity {'E'};

    static constexpr int digital_IO_start_reg {466};
    static constexpr int digital_IO_reg_count {2};
    static constexpr int speed_start_reg {448};
    static constexpr int speed_reg_count {3};

    /**
     * @brief Create HAL pins.
     * @return 0 on success, non-zero otherwise.
     */
    [[nodiscard]] int create_hal_pins() const noexcept;

    void read_speed_data();
    void read_digital_IO();
};

/** Signals, pins and parameters from LinuxCNC and HAL */
class Hal_data {
private:
    Hal_data() = default;
    friend class Lichuan_a4;

    static constexpr void initialize_data(Hal_data *hal) noexcept;

    // Info from driver
    hal_float_t     *commanded_speed{}; /*!< commanded speed [RPM] */
    hal_float_t     *feedback_speed{};  /*!< feedback speed [RPM] */
    hal_float_t     *deviation_speed{}; /*!< deviation between command and
                                             feedback speed [RPM] */
    hal_float_t     *dc_bus_volt{};     /*!< DC bus voltage [V] */
    hal_float_t     *torque_load{};     /*!< torque load ratio [%] */
    hal_float_t     *res_braking{};     /*!< resistance braking rate [%] */
    hal_float_t     *torque_overload{}; /*!< torque overload ratio [%] */

    // Digital IO is configurable from driver
    hal_bit_t       *digital_in0{};
    hal_bit_t       *digital_in1{};
    hal_bit_t       *digital_in2{};
    hal_bit_t       *digital_in3{};
    hal_bit_t       *digital_in4{};
    hal_bit_t       *digital_in5{};
    hal_bit_t       *digital_in6{};
    hal_bit_t       *digital_in7{};
    hal_bit_t       *digital_out0{};
    hal_bit_t       *digital_out1{};
    hal_bit_t       *digital_out2{};
    hal_bit_t       *digital_out3{};
    hal_bit_t       *digital_out4{};
    hal_bit_t       *digital_out5{};

    // Parameters
    hal_float_t  modbus_polling{};      /*!< Modbus polling frequency [s] */
    hal_u32_t    modbus_errors{};       /*!< Modbus error count */
};

#endif // LICHUAN_A4_H
