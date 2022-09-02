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
#ifndef _TPS23861_H
#define _TPS23861_H

#include <stdint.h>

struct tps23861_dev {
    uint8_t addr;
    uint32_t shunt_resistor;
    uint8_t bus_number;
};

int tps23861_init(struct tps23861_dev *dev);
int tps23861_get_voltage(struct tps23861_dev *dev, int channel, long *val);
int tps23861_get_temperature(struct tps23861_dev *dev, long *val);
int tps23861_get_current(struct tps23861_dev *dev, int channel, long *val);
int tps23861_set_port_power(struct tps23861_dev *dev, int channel, int enable);
int tps23861_set_poe_plus_mode(struct tps23861_dev *dev, int channel, int enable);
char *tps23861_get_detect_status(struct tps23861_dev *dev, int channel);
char *tps23861_get_class_status(struct tps23861_dev *dev, int channel);

#endif //_TPS23861_H
