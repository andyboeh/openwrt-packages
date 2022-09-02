// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022 Andreas Böhler <dev@aboehler.at>
 *
 * TP-Link SG2452P ports to TPS23861 address and channel mapping file
 *
 * Author: Andreas Böhler <dev@aboehler.at>
 */

#ifndef _TPS23861_MAPPINGS_H
#define _TPS23861_MAPPINGS_H

const unsigned char SG2452P_BUS_NUMBER = 0;
const unsigned int SG2452P_NUM_PORTS = 48;
const unsigned int SG2452P_NUM_CHIPS = 12;
const unsigned int SG2452P_SHUNT_RESISTOR = 255000;

const unsigned char SG2452P_CHANNEL_MAP[] = {
                                              0, 1, 2, 3, 
                                              0, 1, 2, 3, 
                                              0, 1, 2, 3, 
                                              0, 1, 2, 3, 
                                              0, 1, 2, 3, 
                                              0, 1, 2, 3, 
                                              0, 1, 2, 3, 
                                              0, 1, 2, 3, 
                                              0, 1, 2, 3, 
                                              0, 1, 2, 3, 
                                              0, 1, 2, 3, 
                                              0, 1, 2, 3
                                            };

const unsigned char SG2452P_DEV_LIST[] = {0x05, 0x06, 0x09, 0x0a, 0x14, 0x24, 
                                          0x25, 0x26, 0x29, 0x2c, 0x48, 0x49};

const unsigned char SG2452P_DEV_MAP[] = {
                                              0x14, 0x14, 0x14, 0x14,
                                              0x48, 0x48, 0x48, 0x48,
                                              0x05, 0x05, 0x05, 0x05,
                                              0x29, 0x29, 0x29, 0x29,
                                              0x06, 0x06, 0x06, 0x06,
                                              0x49, 0x49, 0x49, 0x49,
                                              0x24, 0x24, 0x24, 0x24,
                                              0x2c, 0x2c, 0x2c, 0x2c,
                                              0x25, 0x25, 0x25, 0x25,
                                              0x0a, 0x0a, 0x0a, 0x0a,
                                              0x26, 0x26, 0x26, 0x26,
                                              0x09, 0x09, 0x09, 0x09
                                          };

#endif //_TPS23861_MAPPINGS_H
