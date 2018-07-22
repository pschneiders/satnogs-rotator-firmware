/*
 * watchdog.h
 *
 *  Created on: Nov 12, 2017
 *      Author: azisi
 */

#ifndef WATCHDOG_H_
#define WATCHDOG_H_

#include <avr/wdt.h>
#include "globals.h"
#include "easycomm.h"
#include "rotator_pins.h"

class wdt_timer{
public:

    void watchdog_init() {
        cli();
        wdt_reset();
        WDTCSR |= _BV(WDCE) | _BV(WDE);
        WDTCSR = _BV(WDIE) | _BV(WDE) | _BV(WDP3) | _BV(WDP2) | _BV(WDP1);
        sei();
    }

    void watchdog_reset() {
        wdt_reset();
    }
};

ISR(WDT_vect) {
    /* Disable motors */
    digitalWrite(MOTOR_EN, LOW);
    /* Set error */
    rotator.rotator_error = wdt_error;
    rotator.rotator_status = error;
    sei();

    while (1) {
        wdt_reset();
        /*Serial*/
        char buffer[BUFFER_SIZE];
        char incomingByte;
        static uint16_t BufferCnt = 0;
        String str1, str2, str3, str4, str5, str6;

        /*Read from serial*/
        while (rs485.available() > 0) {
            incomingByte = rs485.read();

            /*new data*/
            if (incomingByte == '\n') {
                buffer[BufferCnt] = 0;
                if (buffer[0] == 'G' && buffer[1] == 'S') {
                    str1 = String("GS");
                    str2 = String(rotator.rotator_status, DEC);
                    str3 = String("\n");
                    rs485.print(str1 + str2 + str3);
                } else if (buffer[0] == 'G' && buffer[1] == 'E') {
                    str1 = String("GE");
                    str2 = String(rotator.rotator_error, DEC);
                    str3 = String("\n");
                    rs485.print(str1 + str2 + str3);
                } else if (buffer[0] == 'R' && buffer[1] == 'B') {
                    while(1);
                }
                BufferCnt = 0;
                rs485.flush();
            } else {
                /*Fill the buffer with incoming data*/
                buffer[BufferCnt] = incomingByte;
                BufferCnt++;
            }
        }
        wdt_reset();
    }
}

#endif /* WATCHDOG_H_ */
