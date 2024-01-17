/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (C) 2024 Håvard F. Aasen <havard.f.aasen@pfft.no>
 */

/**
 * @file
 * @brief LinuxCNC HAL interface for the Lichuan A4 servo drive.
 *
 * Handles pins and parameters from LinuxCNC and HAL.
 */
#ifndef LICHUAN_A4_HAL_H
#define LICHUAN_A4_HAL_H

#include <hal.h>

#include <string>


class HAL {
public:
    explicit HAL(std::string_view _hal_name);
    ~HAL();

    /** Signals, pins and parameters from LinuxCNC and HAL */
    struct Data {
        // Info from driver
        hal_float_t     *commanded_speed{};     /*!< commanded speed [RPM] */
        hal_float_t     *feedback_speed{};      /*!< feedback speed [RPM] */
        hal_float_t     *deviation_speed{};     /*!< deviation between command and
                                                     feedback speed [RPM] */
        hal_float_t     *commanded_torque{};    /*!< commanded torque [0.1%] */
        hal_float_t     *feedback_torque{};     /*!< feedback torque [0.1%] */
        hal_float_t     *deviation_torque{};    /*!< deviation between command and
                                                     feedback torque [0.1%] */
        hal_float_t     *dc_bus_volt{};         /*!< DC bus voltage [V] */
        hal_float_t     *torque_load{};         /*!< torque load ratio [%] */
        hal_float_t     *res_braking{};         /*!< resistance braking rate [%] */
        hal_float_t     *torque_overload{};     /*!< torque overload ratio [%] */
        hal_s32_t       *error_code{};

        // Digital IO is configurable from driver, but we assume default settings
        hal_bit_t       *digital_in0{};
        hal_bit_t       *digital_in1{};
        hal_bit_t       *digital_in2{};
        hal_bit_t       *digital_in3{};
        hal_bit_t       *digital_in4{};
        hal_bit_t       *digital_in5{};
        hal_bit_t       *digital_in6{};
        hal_bit_t       *digital_in7{};
        hal_bit_t       *digital_out0{};    /*!< servo ready */
        hal_bit_t       *digital_out1{};    /*!< servo alarm */
        hal_bit_t       *digital_out2{};    /*!< location arrival */
        hal_bit_t       *digital_out3{};    /*!< brake release */
        hal_bit_t       *digital_out4{};    /*!< zero speed detection */
        hal_bit_t       *digital_out5{};    /*!< torque limiting */

        // Parameters
        hal_float_t  modbus_polling{};      /*!< Modbus polling frequency [s] */
        hal_u32_t    modbus_errors{};       /*!< Modbus error count */
    };
    Data *data;

private:
    std::string hal_name{};
    int hal_comp_id{};

    /**
     * @brief Create HAL pins.
     * @return 0 on success, non-zero otherwise.
     */
    [[nodiscard]] int create_hal_pins() const noexcept;
    constexpr void initialize_data() noexcept;

};

#endif // LICHUAN_A4_HAL_H
