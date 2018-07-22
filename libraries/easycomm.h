/*
 * easycomm.h
 *
 *  Created on: Jul 18, 2018
 *      Author: azisi
 */

#ifndef LIBRARIES_EASYCOMM_H_
#define LIBRARIES_EASYCOMM_H_

#include <Arduino.h>
#include <WString.h>
#include <avr/wdt.h>
#include "rs485.h"
#include "rotator_pins.h"
#include "globals.h"

/* Communication interface */
#define RS485_TX_TIME 9 // Wait t ms to write in serial
#define BUFFER_SIZE 256
#define BAUDRATE 19200

rs485 rs485(RS485_DIR, RS485_TX_TIME);

class easycomm {
public:
    void easycomm_init() {
        rs485.begin(BAUDRATE);
    }

    /*EasyComm 3 Protocol & Calculate the steps*/
    void easycomm_proc() {
        /*Serial*/
        char buffer[BUFFER_SIZE];
        char incomingByte;
        char *Data = buffer;
        char *rawData;
        static uint16_t BufferCnt = 0;
        char data[100];
        String str1, str2, str3, str4, str5, str6;

        /*Read from serial*/
        while (rs485.available() > 0) {
            incomingByte = rs485.read();

            /*new data*/
            if (incomingByte == '\n') {
                buffer[BufferCnt] = 0;
                if (buffer[0] == 'A' && buffer[1] == 'Z') {
                    if (buffer[2] == ' ' && buffer[3] == 'E' && buffer[4] == 'L') {
                        /*Get position*/
                        str1 = String("AZ");
                        str2 = String(control_az.input, 1);
                        str3 = String(" EL");
                        str4 = String(control_el.input, 1);
                        str5 = String("\n");
                        rs485.print(str1 + str2 + str3 + str4 + str5);
                    } else {
                        /*Get the absolute value of angle*/
                        rotator.control_mode = position;
                        rawData = strtok_r(Data, " ", &Data);
                        strncpy(data, rawData + 2, 10);
                        if (isNumber(data)) {
                            /*Calculate the steps*/
                            control_az.setpoint = atof(data);
                        }
                        /*Get the absolute value of angle*/
                        rawData = strtok_r(Data, " ", &Data);
                        if (rawData[0] == 'E' && rawData[1] == 'L') {
                            strncpy(data, rawData + 2, 10);
                            if (isNumber(data)) {
                                /*Calculate the steps*/
                                control_el.setpoint = atof(data);
                            }
                        }
                    }
                } else if (buffer[0] == 'V' && buffer[1] == 'U') {
                    /* Elevation increase speed */
                    rotator.control_mode = speed;
                    strncpy(data, Data + 2, 10);
                    if (isNumber(data)) {
                        /*Calculate the steps*/
                        control_el.setpoint_speed = atof(data) / 1000;
                    }
                } else if (buffer[0] == 'V' && buffer[1] == 'D') {
                    /* Elevation decrease speed */
                    rotator.control_mode = speed;
                    strncpy(data, Data + 2, 10);
                    if (isNumber(data)) {
                        /*Calculate the steps*/
                        control_el.setpoint_speed = - atof(data) / 1000;
                    }
                } else if (buffer[0] == 'V' && buffer[1] == 'L') {
                    /* Azimuth increase speed */
                    rotator.control_mode = speed;
                    strncpy(data, Data + 2, 10);
                    if (isNumber(data)) {
                        /*Calculate the steps*/
                        control_az.setpoint_speed = atof(data) / 1000;
                    }
                } else if (buffer[0] == 'V' && buffer[1] == 'R') {
                    /* Azimuth decrease speed */
                    rotator.control_mode = speed;
                    strncpy(data, Data + 2, 10);
                    if (isNumber(data)) {
                        /*Calculate the steps*/
                        control_az.setpoint_speed = - atof(data) / 1000;
                    }
                } else if (buffer[0] == 'S' && buffer[1] == 'A' && buffer[2] == ' '
                        && buffer[3] == 'S' && buffer[4] == 'E') {
                    /* Stop Moving */
                    rotator.control_mode = position;
                    str1 = String("AZ");
                    str2 = String(control_az.input, 1);
                    str3 = String(" EL");
                    str4 = String(control_el.input, 1);
                    str5 = String("\n");
                    rs485.print(str1 + str2 + str3 + str4 + str5);
                    control_az.setpoint = control_az.input;
                    control_el.setpoint = control_el.input;
                } else if (buffer[0] == 'R' && buffer[1] == 'E' && buffer[2] == 'S'
                        && buffer[3] == 'E' && buffer[4] == 'T') {
                    /* Reset the rotator */
                    str1 = String("AZ");
                    str2 = String(control_az.input, 1);
                    str3 = String(" EL");
                    str4 = String(control_el.input, 1);
                    str5 = String("\n");
                    rs485.print(str1 + str2 + str3 + str4 + str5);
                    rotator.homing_flag = false;
                } else if (buffer[0] == 'P' && buffer[1] == 'A' && buffer[2] == 'R'
                        && buffer[3] == 'K' ) {
                    /* Park the rotator */
                    str1 = String("AZ");
                    str2 = String(control_az.input, 1);
                    str3 = String(" EL");
                    str4 = String(control_el.input, 1);
                    str5 = String("\n");
                    rs485.print(str1 + str2 + str3 + str4 + str5);
                    control_az.setpoint = rotator.park_az;
                    control_el.setpoint = rotator.park_el;
                } else if (buffer[0] == 'V' && buffer[1] == 'E') {
                    str1 = String("VE");
                    str2 = String("SatNOGS-v2");
                    str3 = String("\n");
                    rs485.print(str1 + str2 + str3);
                } else if (buffer[0] == 'I' && buffer[1] == 'P' && buffer[2] == '0') {
                    str1 = String("IP0,");
                    str2 = String(rotator.inside_temperature, DEC);
                    str3 = String("\n");
                    rs485.print(str1 + str2 + str3);
                } else if (buffer[0] == 'I' && buffer[1] == 'P' && buffer[2] == '1') {
                    str1 = String("IP1,");
                    str2 = String(rotator.switch_az, DEC);
                    str3 = String("\n");
                    rs485.print(str1 + str2 + str3);
                } else if (buffer[0] == 'I' && buffer[1] == 'P' && buffer[2] == '2') {
                    str1 = String("IP2,");
                    str2 = String(rotator.switch_el, DEC);
                    str3 = String("\n");
                    rs485.print(str1 + str2 + str3);
                } else if (buffer[0] == 'I' && buffer[1] == 'P' && buffer[2] == '3') {
                    str1 = String("IP3,");
                    str2 = String(control_az.input, 2);
                    str3 = String("\n");
                    rs485.print(str1 + str2 + str3);
                } else if (buffer[0] == 'I' && buffer[1] == 'P' && buffer[2] == '4') {
                    str1 = String("IP4,");
                    str2 = String(control_el.input, 2);
                    str3 = String("\n");
                    rs485.print(str1 + str2 + str3);
                } else if (buffer[0] == 'I' && buffer[1] == 'P' && buffer[2] == '5') {
                    str1 = String("IP5,");
                    str2 = String(control_az.load, DEC); // Get load for motor driver
                    str3 = String("\n");
                    rs485.print(str1 + str2 + str3);
                } else if (buffer[0] == 'I' && buffer[1] == 'P' && buffer[2] == '6') {
                    str1 = String("IP6,");
                    str2 = String(control_el.load, DEC); // Get load for motor driver
                    str3 = String("\n");
                    rs485.print(str1 + str2 + str3);
                } else if (buffer[0] == 'I' && buffer[1] == 'P' && buffer[2] == '7') {
                    str1 = String("IP7,");
                    str2 = String(control_az.speed, 2);
                    str3 = String("\n");
                    rs485.print(str1 + str2 + str3);
                } else if (buffer[0] == 'I' && buffer[1] == 'P' && buffer[2] == '8') {
                    str1 = String("IP8,");
                    str2 = String(control_el.speed, 2);
                    str3 = String("\n");
                    rs485.print(str1 + str2 + str3);
                } else if (buffer[0] == 'G' && buffer[1] == 'S') {
                    str1 = String("GS");
                    str2 = String(rotator.rotator_status, DEC);
                    str3 = String("\n");
                    rs485.print(str1 + str2 + str3);
                } else if (buffer[0] == 'G' && buffer[1] == 'E') {
                    str1 = String("GE");
                    str2 = String(rotator.rotator_error, DEC);
                    str3 = String("\n");
                    rs485.print(str1 + str2 + str3);
                } else if(buffer[0] == 'C' && buffer[1] == 'R') {
                    /* Get Config
                     * 1 get Kp Azimuth gain
                     * 2 get Ki Azimuth gain
                     * 3 get Kd Azimuth gain
                     * 4 get Kp Elevation gain
                     * 5 get Ki Elevation gain
                     * 6 get Kd Elevation gain
                     * 7 get Azimuth park position
                     * 8 get Elevation park position
                     * 9 control mode */
                    if (buffer[3] == '1') {
                        /* Get the Kp gain */
                        str1 = String("1,");
                        str2 = String(control_az.p, 2);
                        str3 = String("\n");
                        rs485.print(str1 + str2 + str3);
                    } else if (buffer[3] == '2') {
                        /* Get the Ki gain */
                        str1 = String("2,");
                         str2 = String(control_az.i, 2);
                         str3 = String("\n");
                         rs485.print(str1 + str2 + str3);
                    } else if (buffer[3] == '3') {
                        /* Get the Kd gain */
                        str1 = String("3,");
                         str2 = String(control_az.d, 2);
                         str3 = String("\n");
                         rs485.print(str1 + str2 + str3);
                    } else if (buffer[3] == '4') {
                        /* Get the Kp gain */
                        str1 = String("4,");
                         str2 = String(control_el.p, 2);
                         str3 = String("\n");
                         rs485.print(str1 + str2 + str3);
                    } else if (buffer[3] == '5') {
                        /* Get the Ki gain */
                        str1 = String("5,");
                        str2 = String(control_el.i, 2);
                        str3 = String("\n");
                        rs485.print(str1 + str2 + str3);
                    } else if (buffer[3] == '6') {
                        /* Get the Kd gain */
                        str1 = String("6,");
                        str2 = String(control_el.d, 2);
                        str3 = String("\n");
                        rs485.print(str1 + str2 + str3);
                    } else if (buffer[3] == '7') {
                        /* Get the Kd gain */
                        str1 = String("7,");
                        str2 = String(rotator.park_az, 2);
                        str3 = String("\n");
                        rs485.print(str1 + str2 + str3);
                    } else if (buffer[3] == '8') {
                        /* Get the Kd gain */
                        str1 = String("8,");
                        str2 = String(rotator.park_el, 2);
                        str3 = String("\n");
                        rs485.print(str1 + str2 + str3);
                    } else if (buffer[3] == '9') {
                        /* Get control mode */
                        str1 = String("9,");
                        str2 = String(rotator.control_mode);
                        str3 = String("\n");
                        rs485.print(str1 + str2 + str3);
                    }
                } else if (buffer[0] == 'C' && buffer[1] == 'W') {
                    /* Set Config
                     * 1 set Kp Azimuth gain
                     * 2 set Ki Azimuth gain
                     * 3 set Kd Azimuth gain
                     * 4 set Kp Elevation gain
                     * 5 set Ki Elevation gain
                     * 6 set Kd Elevation gain
                     * 7 set Azimuth park position
                     * 8 set Elevation park position
                     * 9 control mode is set by cmd Vx
                     * 10 reset error in EEPROM */
                    if (buffer[2] == '1') {
                        /* Set the Kp gain */
                        rawData = strtok_r(Data, ",", &Data);
                        strncpy(data, rawData + 4, 10);
                        if (isNumber(data)) {
                            control_az.p = atof(data);
                        }
                    } else if (buffer[2] == '2') {
                        /* Get the Ki gain */
                        rawData = strtok_r(Data, ",", &Data);
                        strncpy(data, rawData + 4, 10);
                        if (isNumber(data)) {
                            control_az.i = atof(data);
                        }
                    } else if (buffer[2] == '3') {
                        /* Set the Kd gain */
                        rawData = strtok_r(Data, ",", &Data);
                        strncpy(data, rawData + 4, 10);
                        if (isNumber(data)) {
                            control_az.d = atof(data);
                        }
                    } else if (buffer[2] == '4') {
                        /* Set the Kp gain */
                        rawData = strtok_r(Data, ",", &Data);
                        strncpy(data, rawData + 4, 10);
                        if (isNumber(data)) {
                            control_el.p = atof(data);
                        }
                    } else if (buffer[2] == '5') {
                        /* Set the Ki gain */
                        rawData = strtok_r(Data, ",", &Data);
                        strncpy(data, rawData + 4, 10);
                        if (isNumber(data)) {
                            control_el.i = atof(data);
                        }
                    } else if (buffer[2] == '6') {
                        /* Set the Kd gain */
                        rawData = strtok_r(Data, ",", &Data);
                        strncpy(data, rawData + 4, 10);
                        if (isNumber(data)) {
                            control_el.d = atof(data);
                        }
                    }  else if (buffer[2] == '7') {
                        /* Set the Azimuth park position */
                        rawData = strtok_r(Data, ",", &Data);
                        strncpy(data, rawData + 4, 10);
                        if (isNumber(data)) {
                            rotator.park_az = atof(data);
                        }
                    } else if (buffer[2] == '8') {
                        /* Set the Elevation park position */
                        rawData = strtok_r(Data, ",", &Data);
                        strncpy(data, rawData + 4, 10);
                        if (isNumber(data)) {
                            rotator.park_el = atof(data);
                        }
                    }
                } else if (buffer[0] == 'R' && buffer[1] == 'S'
                        && buffer[2] == 'T') {
                    while(1)
                        ;
                } else if (buffer[0] == 'R' && buffer[1] == 'B') {
                    wdt_enable(WDTO_2S);
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
    }

private:
    /*Check if is argument in number*/
    bool isNumber(char *input) {
        for (uint16_t i = 0; input[i] != '\0'; i++) {
            if (isalpha(input[i]))
                return false;
        }
        return true;
    }
};

#endif /* LIBRARIES_EASYCOMM_H_ */
