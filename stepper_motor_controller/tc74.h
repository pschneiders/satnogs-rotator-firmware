/*
 * tc74.h
 *
 *  Created on: Dec 8, 2017
 *      Author: azisi
 */

#ifndef TC74_H_
#define TC74_H_

#include <Wire.h>

#define TC74_TEMPERATURE_REGISTER 0x00
#define TC74_CONFIGURATION_REGISTER 0x01
#define TC74_STANDBY_COMMAND 0x80
#define TC74_AWAKE_COMMAND 0x00
#define TC74_DATA_READY_FLAG 0x40

class tc74 {
public:

    tc74(uint8_t id) {
        _id = id;
    }

    void init() {
        Wire.begin();
        Wire.setClock(I2C_FREQ);
    }

    int8_t get_temp() {
        Wire.beginTransmission(_id);
        Wire.write(TC74_TEMPERATURE_REGISTER);
        Wire.endTransmission();
        Wire.requestFrom(_id, 1);
        while (Wire.available() == 0)
            ;
        return Wire.read();
    }

    int8_t get_status() {
        Wire.beginTransmission(_id);
        Wire.write(TC74_CONFIGURATION_REGISTER);
        Wire.endTransmission();
        Wire.beginTransmission(_id);
        Wire.write(TC74_AWAKE_COMMAND);
        Wire.endTransmission();
        Wire.requestFrom(_id, 1);
        while (Wire.available() == 0)
            ;
        return Wire.read();
    }

    int8_t wake_up() {
        Wire.beginTransmission(_id);
        Wire.write(TC74_CONFIGURATION_REGISTER);
        Wire.endTransmission();
        Wire.beginTransmission(_id);
        Wire.write(TC74_AWAKE_COMMAND);
        Wire.endTransmission();
        return get_status();
    }

    int8_t sleep() {
        Wire.beginTransmission(_id);
        Wire.write(TC74_CONFIGURATION_REGISTER);
        Wire.endTransmission();
        Wire.beginTransmission(_id);
        Wire.write(TC74_STANDBY_COMMAND);
        Wire.endTransmission();
        return get_status();
    }

private:
    int _id;
};

#endif /* TC74_H_ */
