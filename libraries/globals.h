/*
 * globals.h
 *
 *  Created on: Jul 18, 2018
 *      Author: azisi
 */

#ifndef LIBRARIES_GLOBALS_H_
#define LIBRARIES_GLOBALS_H_

#include <Arduino.h>

/******************************************************************************/
enum _rotator_status {
    idle = 1, moving = 2, pointing = 4, error = 8
};
enum _rotator_error {
    no_error = 1, sensor_error = 2, homing_error = 4, motor_error = 8,
    over_temperature = 12, wdt_error = 16
};
enum _control_mode {
    position = 0, speed = 1
};
struct _control{
    double input;
    double input_prv;
    double speed;
    double setpoint;
    double setpoint_speed;
    uint16_t load;
    double u;
    double p, i, d;
};
struct _rotator{
    volatile enum _rotator_status rotator_status;
    volatile enum _rotator_error rotator_error;
    enum _control_mode control_mode;
    bool homing_flag;
    int8_t inside_temperature;
    double park_az, park_el;
    uint8_t fault_az, fault_el;
    bool switch_az, switch_el;
};
/******************************************************************************/
_control control_az = { .input = 0, .input_prv = 0, .speed=0, .setpoint = 0,
                        .setpoint_speed = 0, .load = 0, .u = 0, .p = 8.0,
                        .i = 0.0, .d = 0.5 };
_control control_el = { .input = 0, .input_prv = 0, .speed=0, .setpoint = 0,
                        .setpoint_speed = 0, .load = 0, .u = 0, .p = 10.0,
                        .i = 0.0, .d = 0.3 };
_rotator rotator = { .rotator_status = idle, .rotator_error = no_error,
                     .control_mode = position, .homing_flag = false,
                     .inside_temperature = 0, .park_az = 0, .park_el = 0,
                     .fault_az = LOW, .fault_el = LOW , .switch_az = false,
                     .switch_el = false};
/******************************************************************************/

#endif /* LIBRARIES_GLOBALS_H_ */
