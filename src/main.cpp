/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (C) 2022-2024 Håvard F. Aasen <havard.f.aasen@pfft.no>
 */

#include "lichuan_a4.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <getopt.h>
#include <iostream>
#include <list>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <thread>


static int done = 0;

static const char* option_string = "d:n:r:vt:h";
static struct option long_options[] = {
        {"device",  required_argument,  nullptr, 'd'},
        {"name",    required_argument,  nullptr, 'n'},
        {"rate",    required_argument,  nullptr, 'r'},
        {"verbose", no_argument,        nullptr, 'v'},
        {"target",  required_argument,  nullptr, 't'},
        {"help",    no_argument,        nullptr, 'h'},
        {nullptr,   0,                  nullptr, 0}
};

static void quit([[maybe_unused]] int sig)
{
    done = 1;
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
        if constexpr (std::is_same_v<T, std::string>) {
            if (token.size() >= HAL_NAME_LEN) {
                std::cerr << "ERROR: HAL component name to long, max size: " << HAL_NAME_LEN << "\n";
                break;
            }
            values.push_back(trim(token));
        }

        if constexpr (std::is_same_v<T, int>) {
            int value = std::atoi(token.c_str());
            if (value < 1 || value > 32) {
                std::cerr << "ERROR: Invalid input in 'target' option: [" << token << "]\n";
                break;
            }
            values.push_back(value);
        }
    }
    return values;
}

int main(int argc, char *argv[])
{
    std::set<int> baud_rates { 2400, 4800, 9600, 19200, 38400, 57600, 115200 };
    std::list<std::string> hal_names { "lichuan_a4" };
    std::list<int> targets { 1 };
    std::string device = "/dev/ttyUSB0";
    int baud = 19200;
    bool verbose = false;

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
                baud = std::atoi(optarg);
                if (baud_rates.find(baud) == baud_rates.end()) {
                    std::cerr << "ERROR: Invalid baud rate: [" << baud << "]\n";
                    exit(-1);
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

    if (hal_names.empty() || targets.empty()) {
        std::cerr << "ERROR: One or both of 'name' and 'target' is empty\n";
        exit(-1);
    }

    /*
     * Point TERM and INT signals at our quit function.
     * If a signal is received between here and the main loop, it should
     * prevent some initialization from happening.
     */
    signal(SIGINT, quit);
    signal(SIGTERM, quit);

    std::list<Lichuan_a4> devices;
    for (const auto& name : hal_names) {
        Target_data servo_data;
        servo_data.device = device;
        servo_data.baud_rate = baud;
        servo_data.hal_name = name;
        servo_data.verbose = verbose;
        servo_data.target = targets.front();
        targets.pop_front();
        try {
            devices.emplace_back(servo_data);
        } catch (std::runtime_error& error) {
            std::cerr << error.what();
            exit(-1);
        }
    }

    while (done == 0) {
        /* Don't scan to fast, and not delay more than a few seconds */
        //if (haldata->period < 0.001) haldata->period = 0.001;
        //if (haldata->period > 2.0) haldata->period = 2.0;
        //std::chrono::duration<double> period_duration(haldata->period);
        //std::chrono::nanoseconds period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(period_duration);
        std::this_thread::sleep_for(std::chrono::seconds{1});

        //for (auto& servo : devices) {
        //    std::cout << servo.target.hal_name << ": Commanded speed: " << servo.hal->commanded_speed << "\n";
        //}

        /* Debug strings */
        //printf("Feedback speed: %f\n", *haldata->feedback_speed);
        //printf("Deviation speed: %f\n", *haldata->deviation_speed);
        //printf("Modbus-errors: %d\n", haldata->modbus_errors);
        //printf("\n");
        done = -1;
    }

    return 0;
}
