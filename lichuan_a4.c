/**
 *  @file lichuan_a4.c
 *  @brief A userspace program that interfaces the Lichuan A4 servo drive
 *         to the LinuxCNC HAL, using RS485 ModBus RTU.
 */


#include <errno.h>
#include <getopt.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include <modbus.h>

#include "hal.h"


/** If a modbus transaction fails, retry this many times before giving up. */
#define NUM_MODBUS_RETRIES 5

#define START_REGISTER_R        0x01C0
#define NUM_REGISTER_R          13

/** Target and registers to read from. */
struct targetdata {
    int target;             /*!< address of device to read from */
    int read_reg_start;     /*!< register to start reading from */
    int read_reg_count;     /*!< how many registers to read */
};

/** Signals, pins and parameters from LinuxCNC and HAL */
struct haldata {
    /* Info from driver */
    hal_float_t     *commanded_speed;   /**< commanded speed (RPM) */
    hal_float_t     *feedback_speed;    /**< feedback speed (RPM) */
    hal_float_t     *deviation_speed;   /**< speed deviation (RPM) */
    hal_float_t     *dc_bus_volt;       /**< DC bus voltage (V) */
    hal_float_t     *torque_load;       /**< torque load ratio (%) */
    hal_float_t     *res_braking;       /**< resistance braking rate (%) */
    hal_float_t     *torque_overload;   /**< torque overload ratio (%) */

    /* Parameter */
    hal_float_t     period;             /**< How often Modbus is polled */
    hal_s32_t       modbus_errors;      /**< Amount of Modbus errors */
};

int hal_comp_id;

static int done;
char *modname = "lichuan_a4";

static int read_data(modbus_t *mb_ctx, struct targetdata *targetdata,
              struct haldata *hal_data_block)
{
    uint16_t receive_data[MODBUS_MAX_READ_REGISTERS];
    int retval;

    /* Can't do anything with an empty datablock */
    if (hal_data_block == NULL)
        return -1;

    /* Signal error if parameter is null */
    if ((mb_ctx == NULL) || (targetdata == NULL))
    {
        hal_data_block->modbus_errors++;
        return -1;
    }

    retval = modbus_read_registers(mb_ctx, targetdata->read_reg_start,
                                   targetdata->read_reg_count, receive_data);

    if (retval == targetdata->read_reg_count) {
        retval = 0;
        *(hal_data_block->commanded_speed) = receive_data[0];
        *(hal_data_block->feedback_speed) = receive_data[1];
        *(hal_data_block->deviation_speed) = receive_data[2];
        *(hal_data_block->dc_bus_volt) = receive_data[9];
        *(hal_data_block->torque_load) = receive_data[10];
        *(hal_data_block->res_braking) = receive_data[11];
        *(hal_data_block->torque_overload) = receive_data[12];
    } else {
        hal_data_block->modbus_errors++;
        retval = -1;
    }
    return retval;
}

/* Command-line options */
static struct option long_options[] = {
    {"device", 1, 0, 'd'},
    {"name", 1, 0 , 'n'},
    {"rate", 1, 0, 'r'},
    {"verbose", 0, 0, 'v'},
    {"target", 1, 0, 't'},
    {"help", 0, 0, 'h'},
    {0,0,0,0}
};

static char *option_string = "d:n:r:vt:h";

static char *ratestrings[] = {"2400", "4800", "9600", "19200", "38400",
                              "57600", "115200", NULL};

static void quit(int sig)
{
    done = 1;
}

int match_string(char *string, char **matches)
{
    unsigned int len;
    int which, match;
    which = 0;
    match = -1;
    if ((matches == NULL) || (string == NULL)) {
        return -1;
    }
    len = strlen(string);
    while (matches[which] != NULL) {
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

void usage(char **argv)
{
    printf("Usage: %s [ARGUMENTS]\n", argv[0]);
    printf( "\n");
    printf("This program interfaces the Lichuan A4 servo driver to the LinuxCNC HAL.\n");
    printf("   Currently this only monitor the Lichuan servo driver.\n");
    printf("\n");
    printf("Optional arguments:\n");
    printf("   -d, --device <path> (default: /dev/ttyUSB0)\n");
    printf("       Set the name of the serial device to use\n");
    printf("   -n, --name <string> (default: lichuan_a4)\n");
    printf("       Set the name of the HAL module. The HAL comp name will be set to <string>, and all pin\n");
    printf("       and parameter names will begin with <string>.\n");
    printf("   -r, --rate <n> (default: 19200)\n");
    printf("       Set baud rate to <n>. It is an error if the rate is not one of the following:\n");
    printf("       2400, 4800, 9600, 19200, 38400, 57600, 115200\n");
    printf("   -t, --target <n> (default: 1)\n");
    printf("       Set Modbus target number. This must match the device\n");
    printf("       number you set on the Lichuan servo driver.\n");
    printf("   -v, --verbose\n");
    printf("       Turn on verbose mode.\n");
    printf("   -h, --help\n");
    printf("       Show this help.\n");
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
 * @param name Name of the HAL module.
 * @return 0 on success, non-zero otherwise.
 */
static int hal_setup(int id, struct haldata *h, const char *name)
{
    int status;

    PIN(hal_pin_float_newf(HAL_OUT, &(h->commanded_speed), id, "%s.commanded-speed", name));
    PIN(hal_pin_float_newf(HAL_OUT, &(h->feedback_speed), id, "%s.feedback-speed", name));
    PIN(hal_pin_float_newf(HAL_OUT, &(h->deviation_speed), id, "%s.deviation-speed", name));
    PIN(hal_pin_float_newf(HAL_OUT, &(h->dc_bus_volt), id, "%s.dc-bus-volt", name));
    PIN(hal_pin_float_newf(HAL_OUT, &(h->torque_load), id, "%s.torque-load", name));
    PIN(hal_pin_float_newf(HAL_OUT, &(h->res_braking), id, "%s.res-braking", name));
    PIN(hal_pin_float_newf(HAL_OUT, &(h->torque_overload), id, "%s.torque-overload", name));

    PIN(hal_param_float_newf(HAL_RW, &(h->period), id, "%s.period-seconds", name));
    PIN(hal_param_s32_newf(HAL_RO, &(h->modbus_errors), id, "%s.modbus-errors", name));

    return 0;
}
#undef PIN

/**
 * @brief Initialize HAL data variables.
 * @param haldata Information to and from, LinuxCNC.
 * @return 0 on success
 */
static int set_haldata_defaults(struct haldata *haldata)
{
    *haldata->commanded_speed = 0;
    *haldata->feedback_speed = 0;
    *haldata->deviation_speed = 0;
    *haldata->dc_bus_volt = 0;
    *haldata->torque_load = 0;
    *haldata->res_braking = 0;
    *haldata->torque_overload = 0;

    haldata->period = 2.0;
    haldata->modbus_errors = 0;

    return 0;
}

int main(int argc, char **argv)
{
    struct haldata *haldata;
    struct targetdata targetdata;
    struct timespec period_timespec;

    char *device;
    int baud;
    int bits;
    char parity;
    int stopbits;
    int verbose;

    int retval = 0;
    modbus_t *mb_ctx;
    int target;
    char *endarg;
    int opt;
    int argindex, argvalue;

    done = 0;

    /* Assume that nothing is specified on the command line */
    device = "/dev/ttyUSB0";
    baud = 19200;
    bits = 8;
    parity = 'E';
    stopbits = 1;

    verbose = 0;
    target = 1;

    targetdata.read_reg_start = START_REGISTER_R;
    targetdata.read_reg_count = NUM_REGISTER_R;

    /* Process command line options */
    while ((opt = getopt_long(argc, argv, option_string, long_options, NULL)) != -1) {
        switch (opt) {
            /* Device name, default /dev/ttyUSB0 */
            case 'd':
                /*
                 * Could check the device name here, but we'll leave it
                 * to the library open
                 */
                if (strlen(optarg) > FILENAME_MAX) {
                    fprintf(stderr, "ERROR: device node name is to long: %s\n",
                            optarg);
                    retval = -1;
                    goto out_noclose;
                }
                device = strdup(optarg);
                break;

            /* Module base name */
            case 'n':
                if (strlen(optarg) > HAL_NAME_LEN - 20) {
                    fprintf(stderr, "ERROR: HAL module name to long: %s\n",
                            optarg);
                    retval = -1;
                    goto out_noclose;
                }
                modname = strdup(optarg);
                break;

            /* Baud rate, defaults to 19200 */
            case 'r':
                argindex = match_string(optarg, ratestrings);
                if (argindex < 0) {
                    fprintf(stderr, "ERROR: invalid baud rate: %s\n", optarg);
                    retval = -1;
                    goto out_noclose;
                }
                baud = atoi(ratestrings[argindex]);
                break;

            /* Target number (MODBUS ID), default 1 */
            case 't':
                argvalue = strtol(optarg, &endarg, 10);
                if ((*endarg != '\0') || (argvalue < 1) || (argvalue > 31)) {
                    fprintf(stderr, "ERROR: invalid target number: %s\n",
                            optarg);
                    retval = -1;
                    goto out_noclose;
                }
                target = argvalue;
                break;

            case 'v':
                verbose = 1;
                break;

            case 'h':
                usage(argv);
                exit(0);
                break;

            default:
                usage(argv);
                exit(1);
                break;
        }
    }

    printf("%s: device='%s', baud=%d, bits=%d, parity='%c', stopbits=%d, address=%d\n",
           modname, device, baud, bits, parity, stopbits, target);

    /*
     * Point TERM and INT signals at our quit function.
     * If a signal is recieved between here and the main loop, it should
     * prevent some initialization from happening.
     */
    signal(SIGINT, quit);
    signal(SIGTERM, quit);

    /* Assume 19200 bps 8-E-1 serial setting, device 1 */
    mb_ctx = modbus_new_rtu(device, baud, parity, bits, stopbits);
    if (mb_ctx == NULL) {
        fprintf(stderr, "%s: ERROR: Couldn't open modbus serial device: %s\n",
                modname, modbus_strerror(errno));
        goto out_noclose;
    }

    retval = modbus_connect(mb_ctx);
    if (retval != 0) {
        fprintf(stderr, "%s: ERROR: Couldn't open serial device: %s\n",
                modname, modbus_strerror(errno));
        goto out_noclose;
    }

    modbus_set_debug(mb_ctx, verbose);

    modbus_set_slave(mb_ctx, target);

    /* Create HAL component */
    hal_comp_id = hal_init(modname);
    if (hal_comp_id <0) {
        fprintf(stderr, "%s: ERROR: hal_init failed\n", modname);
        retval = hal_comp_id;
        goto out_close;
    }

    haldata = (struct haldata *)hal_malloc(sizeof(struct haldata));
    if (haldata == NULL) {
        fprintf(stderr, "%s: ERROR: unable to allocate shared memory\n",
                modname);
        retval = -1;
        goto out_closeHAL;
    }

    if (hal_setup(hal_comp_id, haldata, modname)) {
        goto out_closeHAL;
    }

    set_haldata_defaults(haldata);

    /* Activate HAL component */
    hal_ready(hal_comp_id);

    while (done == 0) {
        /* Don't scan to fast, and not delay more than a few seconds */
        if (haldata->period < 0.001) haldata->period = 0.001;
        if (haldata->period > 2.0) haldata->period = 2.0;
        period_timespec.tv_sec = (time_t)(haldata->period);
        period_timespec.tv_nsec = (long)((haldata->period - period_timespec.tv_sec) * 1000000000l);
        nanosleep(&period_timespec, NULL);

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
out_noclose:
    return retval;
}
