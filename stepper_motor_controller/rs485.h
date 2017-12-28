/*
 * rs485.h
 *
 *  Created on: Dec 9, 2017
 *      Author: azisi
 */

#ifndef RS485_H_
#define RS485_H_

class rs485 {
public:

    rs485(uint8_t pin_dir, uint16_t tx_time) {
        _pin_dir = pin_dir;
        _tx_time = tx_time;
    }

    void begin(uint16_t baudrate) {
        pinMode(_pin_dir, OUTPUT);
        Serial.begin(baudrate);
    }

    void print(String str) {
        digitalWrite(_pin_dir, HIGH);
        Serial.print(str);
        delay(_tx_time);
        digitalWrite(_pin_dir, LOW);
    }

    uint8_t read() {
        return Serial.read();
    }

    uint8_t available(void) {
        return Serial.available();
    }

    void flush() {
        Serial.flush();
    }

    void end() {
        Serial.end();
    }

private:
    uint8_t _pin_dir;
    uint16_t _tx_time;
};

#endif /* RS485_H_ */
