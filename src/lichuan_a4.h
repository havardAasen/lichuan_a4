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

#include <hal.h>
#include <modbus.h>

#include <string>


inline constexpr int start_register_r { 0x01C0 };
inline constexpr int num_register_r { 13 };

/** Target and registers to read from. */
struct Target_data {
    std::string hal_name{};
    int hal_comp_id{};

    // Modbus values
    std::string device{};
    int target{};               /*!< address of device to read from */
    int baud_rate{ 19200 };
    bool verbose{ false };

    int read_reg_start{ start_register_r };  /*!< register to start reading from */
    int read_reg_count{ num_register_r };    /*!< how many registers to read */
};

/** Signals, pins and parameters from LinuxCNC and HAL */
struct Hal_data {
    // Info from driver
    hal_float_t     *commanded_speed;   /*!< commanded speed [RPM] */
    hal_float_t     *feedback_speed;    /*!< feedback speed [RPM] */
    hal_float_t     *deviation_speed;   /*!< speed deviation [RPM] */
    hal_float_t     *dc_bus_volt;       /*!< DC bus voltage [V] */
    hal_float_t     *torque_load;       /*!< torque load ratio [%] */
    hal_float_t     *res_braking;       /*!< resistance braking rate [%] */
    hal_float_t     *torque_overload;   /*!< torque overload ratio [%] */

    // Parameters
    hal_float_t  modbus_polling;     /*!< Modbus polling frequency [s] */
    hal_s32_t    modbus_errors;      /*!< Modbus error count */
};


class Lichuan_a4 {
public:
    explicit Lichuan_a4(Target_data& _target);
    ~Lichuan_a4();

    int read_data();

    Lichuan_a4(const Lichuan_a4 &) = delete;
    Lichuan_a4 &operator=(const Lichuan_a4 &) = delete;

private:
    Hal_data* hal{};
    Target_data target;
    modbus_t* mb_ctx{};

    /** If a modbus transaction fails, retry this many times before giving up. */
    static constexpr int modbus_retries {5};

    // Modbus settings, hard-coded in servo driver
    static constexpr int data_bits { 8 };
    static constexpr int stop_bits { 1 };
    static constexpr char parity { 'E' };

    /**
     * @brief Create HAL pins.
     * @return 0 on success, non-zero otherwise.
     */
    [[nodiscard]] int create_hal_pins() const noexcept;
    constexpr void initialize_haldata() const noexcept;
};

#endif // LICHUAN_A4_H
