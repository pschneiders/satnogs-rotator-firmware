/*
 * watchdog.h
 *
 *  Created on: Nov 12, 2017
 *      Author: azisi
 */

#ifndef WATCHDOG_H_
#define WATCHDOG_H_

#include <avr/wdt.h>

void watchdog_init(uint8_t);

inline void watchdog_reset() {
    wdt_reset();
}

#endif /* WATCHDOG_H_ */
