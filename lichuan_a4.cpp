/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (C) 2022-2024 Håvard F. Aasen <havard.f.aasen@pfft.no>
 */

/**
 *  @file
 *  @brief A userspace program that interfaces the Lichuan A4 servo drive
 *         to the LinuxCNC HAL, using RS485 Modbus RTU.
 */


#include <cerrno>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <getopt.h>
//#include <time.h>

#include <iostream>
#include <string>
#include <thread>

#include <modbus.h>
#include <sstream>
#include <list>

#include "hal.h"


/** If a modbus transaction fails, retry this many times before giving up. */
constexpr int modbus_retries =  5;

constexpr int start_register_r = 0x01C0;
constexpr int num_register_r = 13;

/** Target and registers to read from. */
struct Target_data {
    int target{};                            /*!< address of device to read from */
    int read_reg_start{ start_register_r };  /*!< register to start reading from */
    int read_reg_count{ num_register_r };    /*!< how many registers to read */
};

/** Signals, pins and parameters from LinuxCNC and HAL */
struct Hal_data {
    /* Info from driver */
    hal_float_t     *commanded_speed;   /*!< commanded speed [RPM] */
    hal_float_t     *feedback_speed;    /*!< feedback speed [RPM] */
    hal_float_t     *deviation_speed;   /*!< speed deviation [RPM] */
    hal_float_t     *dc_bus_volt;       /*!< DC bus voltage [V] */
    hal_float_t     *torque_load;       /*!< torque load ratio [%] */
    hal_float_t     *res_braking;       /*!< resistance braking rate [%] */
    hal_float_t     *torque_overload;   /*!< torque overload ratio [%] */

    /* Parameter */
    hal_float_t     period;             /*!< How often Modbus is polled */
    hal_s32_t       modbus_errors;      /*!< Amount of Modbus errors */
};

static int done = 0;
static std::string modname = "lichuan_a4";

static int read_data(modbus_t* mb_ctx, const Target_data* targetdata, Hal_data* haldata)
{
    uint16_t receive_data[MODBUS_MAX_READ_REGISTERS];

    for (int retries = 0; retries < modbus_retries; retries++) {
        int retval = modbus_read_registers(mb_ctx, targetdata->read_reg_start,
                                           targetdata->read_reg_count, receive_data);

        if (retval == targetdata->read_reg_count) {
            *haldata->commanded_speed = receive_data[0];
            *haldata->feedback_speed = receive_data[1];
            *haldata->deviation_speed = receive_data[2];
            *haldata->dc_bus_volt = receive_data[9];
            *haldata->torque_load = receive_data[10];
            *haldata->res_braking = receive_data[11];
            *haldata->torque_overload = receive_data[12];
            return 0;
        }
        std::cerr << modname << ": ERROR reading data for "
                  << targetdata->read_reg_count << " registers, from register "
                  << targetdata->read_reg_start << ": " << modbus_strerror(errno) << "\n";
        haldata->modbus_errors++;
    }

    return -1;
}

/* Command-line options */
static struct option long_options[] = {
    {"device",  required_argument,  nullptr, 'd'},
    {"name",    required_argument,  nullptr, 'n'},
    {"rate",    required_argument,  nullptr, 'r'},
    {"verbose", no_argument,        nullptr, 'v'},
    {"target",  required_argument,  nullptr, 't'},
    {"help",    no_argument,        nullptr, 'h'},
    {nullptr,   0,                  nullptr, 0}
};

static const char* option_string = "d:n:r:vt:h";

static const char* rate_strings[] = {"2400", "4800", "9600", "19200", "38400",
                                     "57600", "115200", nullptr};

static void quit([[maybe_unused]] int sig)
{
    done = 1;
}

int match_string(const char* string, const char* matches[])
{
    if ((!matches) || (!string)) {
        return -1;
    }

    int which = 0;
    int match = -1;
    const std::size_t len = strlen(string);
    while (!matches[which]) {
        if ((!strncmp(string, matches[which], len)) && (len <= strlen(matches[which]))) {
            /* Multiple matches */
            if (match >= 0) {
                return -1;
            }
            match = which;
        }
        ++which;
    }
    return match;
}

void usage(char *argv[])
{
    std::cout << "Usage: " << argv[0] << " [ARGUMENTS]\n"
              << "\n"
              << "This program interfaces the Lichuan A4 servo driver to the LinuxCNC HAL.\n"
              << "   Currently this only monitor the Lichuan servo driver.\n"
              << "\n"
              << "Optional arguments:\n"
              << "   -d, --device <path> (default: '/dev/ttyUSB0')\n"
              << "       Set the name of the serial device to use\n"
              << "   -n, --name <strings> (default: 'lichuan_a4')\n"
              << "       Set the name of the HAL module. The HAL comp name will be set to <string>, and all pin\n"
              << "       and parameter names will begin with <string>. If multiple names is given, multiple\n"
              << "       HAL modules will be created.\n"
              << "   -r, --rate <n> (default: 19200)\n"
              << "       Set baud rate to <n>. It is an error if the rate is not one of the following:\n"
              << "       [2400, 4800, 9600, 19200, 38400, 57600, 115200]\n"
              << "   -t, --target <integers> (default: 1)\n"
              << "       Set Modbus target number. This must match the device\n"
              << "       number you set on the Lichuan servo driver.\n"
              << "   -v, --verbose\n"
              << "       Turn on verbose mode.\n"
              << "   -h, --help\n"
              << "       Show this help.\n";
}

#define PIN(x)\
        do {\
            status = (x);\
            if (status != 0) return status;\
        } while (0)

/**
 * @brief Create HAL pins.
 * @param id Component ID created by HAL.
 * @param h Information to and from, LinuxCNC.
 * @return 0 on success, non-zero otherwise.
 */
static int hal_setup(const int id, Hal_data *h)
{
    int status;

    PIN(hal_pin_float_newf(HAL_OUT, &h->commanded_speed, id, "%s.commanded-speed", modname.c_str()));
    PIN(hal_pin_float_newf(HAL_OUT, &h->feedback_speed, id, "%s.feedback-speed", modname.c_str()));
    PIN(hal_pin_float_newf(HAL_OUT, &h->deviation_speed, id, "%s.deviation-speed", modname.c_str()));
    PIN(hal_pin_float_newf(HAL_OUT, &h->dc_bus_volt, id, "%s.dc-bus-volt", modname.c_str()));
    PIN(hal_pin_float_newf(HAL_OUT, &h->torque_load, id, "%s.torque-load", modname.c_str()));
    PIN(hal_pin_float_newf(HAL_OUT, &h->res_braking, id, "%s.res-braking", modname.c_str()));
    PIN(hal_pin_float_newf(HAL_OUT, &h->torque_overload, id, "%s.torque-overload", modname.c_str()));

    PIN(hal_param_float_newf(HAL_RW, &h->period, id, "%s.period-seconds", modname.c_str()));
    PIN(hal_param_s32_newf(HAL_RO, &h->modbus_errors, id, "%s.modbus-errors", modname.c_str()));

    return 0;
}
#undef PIN

/**
 * @brief Initialize HAL data variables.
 * @param hal Information to and from, LinuxCNC.
 * @return 0 on success
 */
static int set_haldata_defaults(Hal_data *hal)
{
    *hal->commanded_speed = 0;
    *hal->feedback_speed = 0;
    *hal->deviation_speed = 0;
    *hal->dc_bus_volt = 0;
    *hal->torque_load = 0;
    *hal->res_braking = 0;
    *hal->torque_overload = 0;

    hal->period = 2.0;
    hal->modbus_errors = 0;

    return 0;
}

static bool try_parse(const std::string& str, int& result) {
    std::istringstream iss(str);
    return (iss >> result) && iss.eof();
}

static std::string trim(const std::string& str) {
    std::size_t first = str.find_first_not_of(" \t\n\r");
    std::size_t last = str.find_last_not_of(" \t\n\r");
    return (first != std::string::npos && last != std::string::npos) ? str.substr(first, last - first + 1) : "";
}

template<typename T>
static std::list<T> parse_arguments(const std::string& input) {
    std::list<T> values;
    std::istringstream iss(input);
    std::string token;
    while (std::getline(iss, token, ',')) {
        if constexpr (std::is_same_v<T, std::string>)
            values.push_back(trim(token));

        if constexpr (std::is_same_v<T, int>) {
            int value;
            if (try_parse(token, value)) {
                values.push_back(value);
            } else {
                std::cerr << "ERROR: Invalid input in 'target' option: " << token << "\n";
                break;
            }
        }
    }
    return values;
}

int main(int argc, char *argv[])
{
    Hal_data* haldata;
    Target_data targetdata;
    std::list<std::string> hal_names { "lichuan_a4" };
    std::list<int> targets { 1 };
    //struct timespec period_timespec;

    /* Default Modbus values */
    std::string device = "/dev/ttyUSB0";
    int baud = 19200;
    const int bits = 8;
    const char parity = 'E';
    const int stopbits = 1;
    bool verbose = false;
    int target = 1;

    int opt;
    while ((opt = getopt_long(argc, argv, option_string, long_options, nullptr)) != -1) {
        switch (opt) {
            case 'd': /* Device name */
                if (strlen(optarg) > FILENAME_MAX) {
                    std::cerr << "ERROR: Device name to long: " << optarg << "\n";
                    exit(-1);
                }
                device = optarg;
                break;
            case 'n': /* Module base name */
                hal_names = parse_arguments<std::string>(optarg);
                break;
            case 'r': /* Baud rate */
                {
                    const int arg_index = match_string(optarg, rate_strings);
                    if (arg_index < 0) {
                        std::cerr << "ERROR: invalid baud rate: " << optarg << "\n";
                        exit(-1);
                    }
                    baud = atoi(rate_strings[arg_index]);
                }
                break;
            case 't': /* Target number */
                targets = parse_arguments<int>(optarg);
                break;
            case 'v':
                verbose = true;
                break;
            case 'h':
                usage(argv);
                exit(0);
            default:
                usage(argv);
                exit(1);
        }
    }

    if (hal_names.size() != targets.size()) {
        std::cerr << "ERROR: 'name' and 'target' must have the same number of arguments\n";
        exit(-1);
    }

    std::cout << modname << ": device='" << device << "', baud=" << baud
              << ", bits=" << bits << ", parity='" << parity << "', stopbits="
              << stopbits << ", target=" << target << "\n";

    /*
     * Point TERM and INT signals at our quit function.
     * If a signal is received between here and the main loop, it should
     * prevent some initialization from happening.
     */
    signal(SIGINT, quit);
    signal(SIGTERM, quit);

    /* Assume 19200 bps 8-E-1 serial setting, device 1 */
    modbus_t* mb_ctx = modbus_new_rtu(device.c_str(), baud, parity, bits, stopbits);
    if (!mb_ctx) {
        std::cerr << modname << ": ERROR: Couldn't open modbus serial device: "
                  << modbus_strerror(errno) << "\n";
        exit(-1);
    }

    int retval = modbus_connect(mb_ctx);
    if (retval != 0) {
        std::cerr << modname << ": ERROR: Couldn't open serial device: "
                  << modbus_strerror(errno) << "\n";
        modbus_free(mb_ctx);
        exit(-1);
    }

    modbus_set_debug(mb_ctx, verbose);
    modbus_set_slave(mb_ctx, target);

    /* Create HAL component */
    const int hal_comp_id = hal_init(modname.c_str());
    if (hal_comp_id < 0) {
        std::cerr << modname << ": ERROR: hal_init failed\n";
        retval = hal_comp_id;
        goto out_close;
    }

    haldata = static_cast<struct Hal_data *>(hal_malloc(sizeof(struct Hal_data)));
    if (!haldata) {
        std::cerr << modname << ": ERROR: unable to allocate shared memory\n";
        retval = -1;
        goto out_closeHAL;
    }

    if (hal_setup(hal_comp_id, haldata)) {
        goto out_closeHAL;
    }

    set_haldata_defaults(haldata);

    /* Activate HAL component */
    hal_ready(hal_comp_id);

    while (done == 0) {
        /* Don't scan to fast, and not delay more than a few seconds */
        if (haldata->period < 0.001) haldata->period = 0.001;
        if (haldata->period > 2.0) haldata->period = 2.0;
        std::chrono::duration<double> period_duration(haldata->period);
        std::chrono::nanoseconds period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(period_duration);
        std::this_thread::sleep_for(period_ns);

        //period_timespec.tv_sec = (time_t)(haldata->period);
        //period_timespec.tv_nsec = (long)((haldata->period - period_timespec.tv_sec) * 1000000000l);
        //nanosleep(&period_timespec, NULL);

        read_data(mb_ctx, &targetdata, haldata);

        /* Debug strings */
        printf("Commanded speed: %f\n", *haldata->commanded_speed);
        printf("Feedback speed: %f\n", *haldata->feedback_speed);
        printf("Deviation speed: %f\n", *haldata->deviation_speed);
        printf("Modbus-errors: %d\n", haldata->modbus_errors);
        printf("\n");
    }

out_closeHAL:
    hal_exit(hal_comp_id);
out_close:
    modbus_close(mb_ctx);
    modbus_free(mb_ctx);
    return retval;
}
