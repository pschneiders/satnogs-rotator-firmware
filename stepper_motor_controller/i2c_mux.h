/*
 * i2c_mux.h
 *
 *  Created on: Nov 12, 2017
 *      Author: azisi
 */

#ifndef I2C_MUX_H_
#define I2C_MUX_H_

#include <Wire.h>

class i2c_mux {
public:

    i2c_mux(uint8_t id, uint8_t ch0, uint8_t ch1) {
        _id = id;
        _ch0 = ch0;
        _ch1 = ch1;
    }

    void init() {
        Wire.begin();
        Wire.setClock(I2C_FREQ);
    }

    void set_channel(uint8_t ch) {
        if (ch == _ch0) {
            Wire.beginTransmission(_id);
            Wire.write(_ch0);
            Wire.endTransmission();
        } else if (ch == _ch1) {
            Wire.beginTransmission(_id);
            Wire.write(_ch1);
            Wire.endTransmission();
        }
    }

private:
    uint8_t _id, _ch0, _ch1;
};

#endif /* I2C_MUX_H_ */
