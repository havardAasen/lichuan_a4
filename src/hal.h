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
    HAL(const HAL&) = delete;
    HAL& operator=(const HAL&) = delete;
    HAL(HAL&& rhs) noexcept;
    HAL& operator=(HAL&& rhs) noexcept;
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
        hal_s32_t       *error_code{};          /*!< servo driver error code */

        // Digital IO is configurable from driver, we assume default settings
        hal_bit_t       *digital_in0{};     /*!< servo enabling */
        hal_bit_t       *digital_in1{};     /*!< clear alarm */
        hal_bit_t       *digital_in2{};     /*!< clockwise stroke limit */
        hal_bit_t       *digital_in3{};     /*!< anticlockwise stroke limit */
        hal_bit_t       *digital_in4{};     /*!< clear deviation counter to 0 */
        hal_bit_t       *digital_in5{};     /*!< command pulse prohibition */
        hal_bit_t       *digital_in6{};     /*!< torque limit switchover */
        hal_bit_t       *digital_in7{};     /*!< start position of "back to zero" */
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
     * @return @c true if all pins are created, @c false otherwise.
     */
    constexpr void initialize_data() noexcept;
    [[nodiscard]] bool create_hal_pins() const noexcept;

};

#endif // LICHUAN_A4_HAL_H
