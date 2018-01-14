/*
 * rotator_config.h
 *
 *  Created on: Mar 3, 2017
 *      Author: azisi
 */

#ifndef ROTATOR_CONFIG_H_
#define ROTATOR_CONFIG_H_

#define SAMPLE_TIME 1000 /* in ms */
#define WDT_TIMEOUT WDTO_2S
#define RATIO 54 /*Gear ratio*/

/* I2C Multiplexer */
#define I2C_FREQ 100000
#define PCA9540_ID 0x70
#define PCA9540_CH0 0x04
#define PCA9540_CH1 0x05
/* Temperature Sensor */
#define TC74_ID 0x4D
/* Encoder AS5601 */
#define ENC_RATIO 2

/* DC Motors */
#define MAX_PWM 255
#define MIN_PWM 35

/* Stepper Motors */
#define MIN_PULSE_WIDTH 20 /*in microsecond*/
#define MAX_SPEED 400
#define MAX_ACCELERATION 100
#define SPR 200 /*Step Per Revolution*/

/* Homing Functions */
#define MIN_M1_ANGLE 0 /*Maximum Angle of Azimuth for homing scanning*/
#define MAX_M1_ANGLE 365 /*Maximum Angle of Azimuth for homing scanning*/
#define MIN_M2_ANGLE 0
#define MAX_M2_ANGLE 180 /*Maximum Angle of Elevation for homing scanning*/
#define DEFAULT_HOME_STATE HIGH /*Change to LOW according to Home sensor*/
#define HOME_DELAY 10000 /*Time for homing Decceleration in millisecond*/

/* Communication interface */
#define BufferSize 256
#define BaudRate 19200
#define RS485_TX_TIME 9 // ms

#endif /* ROTATOR_CONFIG_H_ */
