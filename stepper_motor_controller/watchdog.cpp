/*
 * watchdog.c
 *
 *  Created on: Nov 12, 2017
 *      Author: azisi
 */

#include "watchdog.h"
#include <Arduino.h>

void watchdog_init(uint8_t delay) {
    // We enable the watchdog timer, but only for the interrupt.
    // Take care, as this requires the correct order of operation,
    // with interrupts disabled. See the datasheet of any AVR chip for details.

    wdt_reset();
    _WD_CONTROL_REG = _BV(_WD_CHANGE_BIT) | _BV(WDE);
    _WD_CONTROL_REG = _BV(WDIE) | delay;
}

ISR( WDT_vect) {
    Serial.println("WDT TIME OUT");
    while (1)
        Serial.println("WDT TIME OUT");    //wait for user
}
