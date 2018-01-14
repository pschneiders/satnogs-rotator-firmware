#include <AccelStepper.h>
#include <Wire.h>
#include "rotator_pins.h"
#include "rotator_config.h"
#include "rs485.h"
#include "endstop.h"
#include "watchdog.h"
#include "i2c_mux.h"
#include "tc74.h"

/******************************************************************************/
enum _rotator_status {
    idle = 1, moving = 2, pointing = 4, error = 8
};
enum _rotator_error {
    no_error = 1, sensor_error = 2, homing_error = 4, motor_error = 8
};

/******************************************************************************/
enum _rotator_error homing(int32_t seek_az, int32_t seek_el);
void cmd_proc(int32_t &setpoint_az, int32_t &setpoit_el);
int32_t deg2step(float deg);
float step2deg(int32_t step);
bool isNumber(char *input);

/******************************************************************************/
AccelStepper stepper_az(1, M1IN1, M1IN2), stepper_el(1, M2IN1, M2IN2);
rs485 rs485(RS485_DIR, RS485_TX_TIME);
endstop switch_az(SW1, DEFAULT_HOME_STATE), switch_el(SW2, DEFAULT_HOME_STATE);
i2c_mux pca9540(PCA9540_ID, PCA9540_CH0, PCA9540_CH1);
tc74 temp_sensor(TC74_ID);

volatile enum _rotator_status rotator_status = idle;
volatile enum _rotator_error rotator_error = no_error;
volatile bool homing_flag = true;
static int32_t step_az = 0, step_el = 0;
/******************************************************************************/

void setup() {
    /* Stepper Motor setup */
    stepper_az.setEnablePin(MOTOR_EN);
    stepper_az.setPinsInverted(false, false, true);
    stepper_az.enableOutputs();
    stepper_az.setMaxSpeed(MAX_SPEED);
    stepper_az.setAcceleration(MAX_ACCELERATION);
    stepper_az.setMinPulseWidth(MIN_PULSE_WIDTH);
    stepper_el.setPinsInverted(false, false, true);
    stepper_el.enableOutputs();
    stepper_el.setMaxSpeed(MAX_SPEED);
    stepper_el.setAcceleration(MAX_ACCELERATION);
    stepper_el.setMinPulseWidth(MIN_PULSE_WIDTH);

    /* Homing switch */
    switch_az.init();
    switch_el.init();

    /* Initialize I2C MUX */
    pca9540.init();

    /* Serial Communication */
    rs485.begin(BaudRate);

    /* Initialize WDT */
    watchdog_init(WDT_TIMEOUT);
}

void loop() {
    /* Update WDT */
    watchdog_reset();

    /*Read the steps from serial*/
    cmd_proc(step_az, step_el);

    if (rotator_status != error) {
        if (homing_flag == false) {
            rotator_error = homing(deg2step(-MAX_M1_ANGLE), deg2step(-MAX_M2_ANGLE));
            if (rotator_error == no_error) {
                /*Zero the steps*/
                step_az = 0;
                step_el = 0;
                rotator_status = idle;
                homing_flag = true;
            } else {
                rotator_status = error;
            }
        } else {
            /*Move the Azimuth & Elevation Motor*/
            stepper_az.moveTo(step_az);
            stepper_el.moveTo(step_el);
            stepper_az.run();
            stepper_el.run();
            rotator_status = pointing;
            if (stepper_az.distanceToGo() == 0 && stepper_el.distanceToGo() == 0) {
                rotator_status = idle;
            }
        }
    } else {
        /* error handler */
        stepper_az.stop();
        stepper_az.disableOutputs();
        stepper_el.stop();
        stepper_el.disableOutputs();
    }
}

/*Homing Function*/
enum _rotator_error homing(int32_t seek_az, int32_t seek_el) {
    bool isHome_az = false;
    bool isHome_el = false;

    stepper_az.moveTo(seek_az);
    stepper_el.moveTo(seek_el);

    /* Homing loop */
    while (isHome_az == false || isHome_el == false) {
        /* Update WDT */
        watchdog_reset();
        /* Homing routine */
        if (switch_az.get_state() == true) {
            stepper_az.moveTo(stepper_az.currentPosition());
            isHome_az = true;
        }
        if (switch_el.get_state() == true) {
            stepper_el.moveTo(stepper_el.currentPosition());
            isHome_el = true;
        }
        if (stepper_az.distanceToGo() == 0 && !isHome_az) {
            return homing_error;
        }
        if (stepper_el.distanceToGo() == 0 && !isHome_el) {
            return homing_error;
        }
        stepper_az.run();
        stepper_el.run();
    }
    /* Delay to Deccelerate and homing */
    uint32_t time = millis();
    while (millis() - time < HOME_DELAY) {
        watchdog_reset();
        stepper_az.run();
        stepper_el.run();
    }
    /*Reset the steps*/
    stepper_az.setCurrentPosition(0);
    stepper_el.setCurrentPosition(0);

    return no_error;
}

/*EasyComm 2 Protocol & Calculate the steps*/
void cmd_proc(int32_t &setpoint_az, int32_t &setpoint_el) {
    /*Serial*/
    char buffer[BufferSize];
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
                    str2 = String(step2deg(stepper_az.currentPosition()), 1);
                    str3 = String(" EL");
                    str4 = String(step2deg(stepper_el.currentPosition()), 1);
                    str5 = String("\n");
                    rs485.print(str1 + str2 + str3 + str4 + str5);
                } else {
                    /*Get the absolute value of angle*/
                    rawData = strtok_r(Data, " ", &Data);
                    strncpy(data, rawData + 2, 10);
                    if (isNumber(data)) {
                        /*Calculate the steps*/
                        setpoint_az = deg2step(atof(data));
                    }
                    /*Get the absolute value of angle*/
                    rawData = strtok_r(Data, " ", &Data);
                    if (rawData[0] == 'E' && rawData[1] == 'L') {
                        strncpy(data, rawData + 2, 10);
                        if (isNumber(data)) {
                            /*Calculate the steps*/
                            setpoint_el = deg2step(atof(data));
                        }
                    }
                }
            }
            /*Stop Moving*/
            else if (buffer[0] == 'S' && buffer[1] == 'A' && buffer[2] == ' '
                    && buffer[3] == 'S' && buffer[4] == 'E') {
                /*Get position*/
                str1 = String("AZ");
                str2 = String(step2deg(stepper_az.currentPosition()), 1);
                str3 = String(" EL");
                str4 = String(step2deg(stepper_el.currentPosition()), 1);
                str5 = String("\n");
                rs485.print(str1 + str2 + str3 + str4 + str5);
                setpoint_az = stepper_az.currentPosition();
                setpoint_el = stepper_el.currentPosition();
            }
            /*Reset the rotator*/
            else if (buffer[0] == 'R' && buffer[1] == 'E' && buffer[2] == 'S'
                    && buffer[3] == 'E' && buffer[4] == 'T') {
                /*Get position*/
                str1 = String("AZ");
                str2 = String(step2deg(stepper_az.currentPosition()), 1);
                str3 = String(" EL");
                str4 = String(step2deg(stepper_el.currentPosition()), 1);
                str5 = String("\n");
                rs485.print(str1 + str2 + str3 + str4 + str5);
                homing_flag = false;
            } else if (buffer[0] == 'V' && buffer[1] == 'E') {
                str1 = String("VE");
                str2 = String("SatNOGS-v2");
                str3 = String("\n");
                rs485.print(str1 + str2 + str3);
            } else if (buffer[0] == 'I' && buffer[1] == 'P' && buffer[2] == '0') {
                pca9540.set_channel(PCA9540_CH1);
                temp_sensor.wake_up();
                str1 = String("IP0,");
                str2 = String(temp_sensor.get_temp(), DEC);
                str3 = String("\n");
                rs485.print(str1 + str2 + str3);
                temp_sensor.sleep();
            } else if (buffer[0] == 'I' && buffer[1] == 'P' && buffer[2] == '1') {
                str1 = String("IP1,");
                str2 = String(switch_az.get_state(), DEC);
                str3 = String("\n");
                rs485.print(str1 + str2 + str3);
            } else if (buffer[0] == 'I' && buffer[1] == 'P' && buffer[2] == '2') {
                str1 = String("IP2,");
                str2 = String(switch_el.get_state(), DEC);
                str3 = String("\n");
                rs485.print(str1 + str2 + str3);
            } else if (buffer[0] == 'G' && buffer[1] == 'S') {
                str1 = String("GS");
                str2 = String(rotator_status, DEC);
                str3 = String("\n");
                rs485.print(str1 + str2 + str3);
            } else if (buffer[0] == 'G' && buffer[1] == 'E') {
                str1 = String("GE");
                str2 = String(rotator_error, DEC);
                str3 = String("\n");
                rs485.print(str1 + str2 + str3);
            }
            BufferCnt = 0;
            rs485.flush();
        }
        /*Fill the buffer with incoming data*/
        else {
            buffer[BufferCnt] = incomingByte;
            BufferCnt++;
        }
    }
}

/*Convert degrees to steps*/
int32_t deg2step(float deg) {
    return (RATIO * SPR * deg / 360);
}

/*Convert steps to degrees*/
float step2deg(int32_t step) {
    return (360.00 * step / (SPR * RATIO));
}

/*Check if is argument in number*/
bool isNumber(char *input) {
    for (uint16_t i = 0; input[i] != '\0'; i++) {
        if (isalpha(input[i]))
            return false;
    }
    return true;
}
