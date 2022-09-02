// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022 Andreas Böhler <dev@aboehler.at>
 * Copyright (c) 2020 Sartura Ltd.
 *
 * User-space driver for the TI TPS23861 PoE PSE.
 * Heavily based on the Linux Kernel TPS23861 HWMON driver.
 *
 * Author: Andreas Böhler <dev@aboehler.at>
 *         Robert Marko <robert.marko@sartura.hr>
 */

#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <endian.h>

#include "tps23861.h"

#define BUS_CHANGED     2
#define BUS_OK          1
#define BUS_FAILURE     -1
#define ADDR_CHANGED    2
#define ADDR_OK         1
#define ADDR_FAILURE    -1

#define TPS23861_DEVICE_ID_REG                          0x43
#define TPS23861_DEVICE_ID_MASK                         0xe0
#define TPS23861_OPERATING_MODE_REG                     0x12
#define TPS23861_OPERATING_MODE_ALL_AUTO                0xff
#define TPS23861_DISCONNECT_ENABLE_REG                  0x13
#define TPS23861_DISCONNECT_ENABLE_ALL_ON               0x0f
#define TPS23861_DETECT_CLASS_ENABLE_REG                0x14
#define TPS23861_DETECT_CLASS_ENABLE_ALL_ON             0xff
#define TPS23861_DETECT_CLASS_RESTART_REG               0x18
#define TPS23861_DETECT_CLASS_RESTART_ALL_ON            0xff
#define TPS23861_POE_PLUS_REG                           0x40
#define TPS23861_PORT_POWER_REG                         0x19
#define TPS23861_PORT_1_STATUS_REG                      0x0c
#define TPS23861_SECOND_CLASSIFICATION_REG              0x21
#define TPS23861_SECOND_CLASSIFICATION_ALL_ON           0x55
#define TPS23861_PORT_1_CURRENT_REG                     0x30
#define TPS23861_PORT_1_VOLTAGE_REG                     0x32
#define TPS23861_GENERAL_MASK_1_REG                     0x17

#define TPS23861_DETECT_STATE_UNKNOWN                   0x00
#define TPS23861_DETECT_STATE_SHORT_CIRCUIT             0x01
#define TPS23861_DETECT_STATE_RESISTANCE_TOO_LOW        0x03
#define TPS23861_DETECT_STATE_RESISTANCE_VALID          0x04
#define TPS23861_DETECT_STATE_RESISTANCE_TOO_HIGH       0x05
#define TPS23861_DETECT_STATE_OPEN_CIRCUIT              0x06
#define TPS23861_DETECT_STATE_MOSFET_FAULT              0x08
#define TPS23861_DETECT_STATE_LEGACY_DETECT             0x09
#define TPS23861_DETECT_STATE_BEYOUND_CLAMP             0x0a
#define TPS23861_DETECT_STATE_INSUFFICIENT_DELTA_V      0x0b
#define TPS23861_DETECT_STATE_CAP_OUTSIDE_LEGACY_RANGE  0x0c

#define TPS23861_CLASS_STATE_UNKNOWN                    0x00
#define TPS23861_CLASS_STATE_CLASS_1                    0x01
#define TPS23861_CLASS_STATE_CLASS_2                    0x02
#define TPS23861_CLASS_STATE_CLASS_3                    0x03
#define TPS23861_CLASS_STATE_CLASS_4                    0x04
#define TPS23861_CLASS_STATE_RESERVED_CLASS_0           0x05
#define TPS23861_CLASS_STATE_CLASS_0                    0x06
#define TPS23861_CLASS_STATE_OVERCURRENT                0x07
#define TPS23861_CLASS_STATE_CLASS_MISMATCH             0x08

#define TPS23861_VOLTAGE_LSB                            3662

static int current_addr = -1;
static int current_bus_number = -1;
static int fd = -1;

static int check_bus(struct tps23861_dev *dev) {
    uint8_t bus = dev->bus_number;
    char filename[20];

    if(bus != current_bus_number) {
        if(fd >= 0)
            close(fd);

        snprintf(filename, 19, "/dev/i2c-%d", bus);
        fd = open(filename, O_RDWR);
        if(fd < 0)
            return BUS_FAILURE;
        current_bus_number = bus;
        return BUS_CHANGED;
    }

    if(fd >= 0)
        return BUS_OK;

    return BUS_FAILURE;
}

static int check_addr(struct tps23861_dev *dev, int bus_res) {
    uint8_t addr = dev->addr;

    if(bus_res == BUS_CHANGED || addr != current_addr) {
        if(fd >= 0) {
            if(ioctl(fd, I2C_SLAVE, addr) < 0)
                return ADDR_FAILURE;
            return ADDR_CHANGED;
        } else {
            return ADDR_FAILURE;
        }
    }
    return ADDR_OK;
}

static int tps23861_read_reg(struct tps23861_dev *dev, int reg) {
    int res;

    res = check_bus(dev);
    if(res != BUS_FAILURE)
        res = check_addr(dev, res);

    if(res == ADDR_FAILURE)
        return -1;

    return i2c_smbus_read_byte_data(fd, reg);
}

static int tps23861_write_reg(struct tps23861_dev *dev, int reg, uint8_t value) {
    int res;

    res = check_bus(dev);
    if(res != BUS_FAILURE)
        res = check_addr(dev, res);

    if(res == ADDR_FAILURE)
        return -1;

    return i2c_smbus_write_byte_data(fd, reg, value);
}

int tps23861_init(struct tps23861_dev *dev) {
    uint8_t val;

    // Read device ID
    val = tps23861_read_reg(dev, TPS23861_DEVICE_ID_REG);
    if((val & TPS23861_DEVICE_ID_MASK) != TPS23861_DEVICE_ID_MASK)
        return 1;

    // Set operating mode to "AUTO" on all ports
    if(tps23861_write_reg(dev, TPS23861_OPERATING_MODE_REG, TPS23861_OPERATING_MODE_ALL_AUTO) < 0)
        return 1;

    // Set Disconnect Enable on all ports
    if(tps23861_write_reg(dev, TPS23861_DISCONNECT_ENABLE_REG, TPS23861_DISCONNECT_ENABLE_ALL_ON) < 0)
        return 1;

    // Perform second classification if Class 4 device is detected
    if(tps23861_write_reg(dev, TPS23861_SECOND_CLASSIFICATION_REG, TPS23861_SECOND_CLASSIFICATION_ALL_ON) < 0)
        return 1;

    // Set current sense register value
    val = tps23861_read_reg(dev, TPS23861_GENERAL_MASK_1_REG);
    if(val < 0)
        return 1;

    if(dev->shunt_resistor == 255000)
        val &= ~(1 << 0);
    else
        val |= (1 << 0);

    if(tps23861_write_reg(dev, TPS23861_GENERAL_MASK_1_REG, val) < 0)
        return 1;

    return 0;
}

int tps23861_get_voltage(struct tps23861_dev *dev, int channel, long *val) {
    uint16_t tmp;
    int lsb;
    int msb;

    lsb = tps23861_read_reg(dev, TPS23861_PORT_1_VOLTAGE_REG + (channel * 4));     // LSB
    msb = (tps23861_read_reg(dev, TPS23861_PORT_1_VOLTAGE_REG + (channel * 4) + 1)); // MSB

    if(lsb < 0 || msb < 0)
        return 1;

    tmp = (msb | (lsb << 8));

    *val = (le16toh(tmp) * TPS23861_VOLTAGE_LSB) / 1000;

    return 0;
}

int tps23861_get_temperature(struct tps23861_dev *dev, long *val) {
    return 0;
}

int tps23861_get_current(struct tps23861_dev *dev, int channel, long *val) {
    uint16_t tmp;
    int lsb;
    int msb;
    int base_value;

    lsb = tps23861_read_reg(dev, TPS23861_PORT_1_CURRENT_REG + (channel * 4));     // LSB
    msb = (tps23861_read_reg(dev, TPS23861_PORT_1_CURRENT_REG + (channel * 4) + 1)); // MSB

    if(lsb < 0 || msb < 0)
        return 1;

    tmp = (msb | (lsb << 8));

    if(dev->shunt_resistor == 255000)
        base_value = 61039;
    else
        base_value = 62260;

    *val = (le16toh(tmp) * base_value) / 1000000;

    return 0;
}

int tps23861_set_port_power(struct tps23861_dev *dev, int channel, int enable) {
    uint8_t val;
    int reg;

    if(enable) {
        val = (1 << channel) | (1 << (channel + 4));
        reg = TPS23861_DETECT_CLASS_RESTART_REG;
    } else {
        val = (1 << (channel + 4));
        reg = TPS23861_PORT_POWER_REG;
    }

    if(tps23861_write_reg(dev, reg, val) < 0)
        return 1;

    return 0;
}

int tps23861_set_poe_plus_mode(struct tps23861_dev *dev, int channel, int enable) {
    uint8_t val;

    val = tps23861_read_reg(dev, TPS23861_POE_PLUS_REG);
    if(val < 0)
        return 1;

    if(enable)
        val |= (1 << (channel + 4));
    else
        val &= ~(1 << (channel + 4));

    if(tps23861_write_reg(dev, TPS23861_POE_PLUS_REG, val) < 0)
        return 1;

    return 0;
}

char *tps23861_get_detect_status(struct tps23861_dev *dev, int channel) {
    int val;

    val = tps23861_read_reg(dev, TPS23861_PORT_1_STATUS_REG + channel);
    val = (val & 0x0f);

    switch(val) {
    case TPS23861_DETECT_STATE_UNKNOWN:
        return "Unknown";
    case TPS23861_DETECT_STATE_SHORT_CIRCUIT:
        return "Short circuit";
    case TPS23861_DETECT_STATE_RESISTANCE_TOO_LOW:
        return "Resistance too low";
    case TPS23861_DETECT_STATE_RESISTANCE_VALID:
        return "Resistance valid";
    case TPS23861_DETECT_STATE_RESISTANCE_TOO_HIGH:
        return "Resistance too high";
    case TPS23861_DETECT_STATE_OPEN_CIRCUIT:
        return "Open Circuit";
    case TPS23861_DETECT_STATE_MOSFET_FAULT:
        return "MOSFET Fault";
    case TPS23861_DETECT_STATE_LEGACY_DETECT:
        return "Legacy Detect";
    case TPS23861_DETECT_STATE_BEYOUND_CLAMP:
        return "Voltage beyound clamp";
    case TPS23861_DETECT_STATE_INSUFFICIENT_DELTA_V:
        return "Insufficient dV";
    case TPS23861_DETECT_STATE_CAP_OUTSIDE_LEGACY_RANGE:
        return "Capacitance outside of legacy range";
    }
    return "Error reading detection status";
}

char *tps23861_get_class_status(struct tps23861_dev *dev, int channel) {
    int val;

    val = tps23861_read_reg(dev, TPS23861_PORT_1_STATUS_REG + channel);
    val = ((val & 0xf0) >> 4);

    switch(val) {
    case TPS23861_CLASS_STATE_UNKNOWN:
        return "Unknown";
    case TPS23861_CLASS_STATE_CLASS_1:
        return "Class 1";
    case TPS23861_CLASS_STATE_CLASS_2:
        return "Class 2";
    case TPS23861_CLASS_STATE_CLASS_3:
        return "Class 3";
    case TPS23861_CLASS_STATE_CLASS_4:
        return "Class 4";
    case TPS23861_CLASS_STATE_RESERVED_CLASS_0:
    case TPS23861_CLASS_STATE_CLASS_0:
        return "Class 0";
    case TPS23861_CLASS_STATE_OVERCURRENT:
        return "Overcurrent";
    case TPS23861_CLASS_STATE_CLASS_MISMATCH:
        return "Class mismatch";
    };
    return "Error reading classification status.";
}
