#include <AccelStepper.h>
#include "rotator_pins.h"
#include "rotator_config.h"
#include "rs485.h"
#include "endstop.h"
#include "watchdog.h"

AccelStepper stepper_az(1, M1IN1, M1IN2), stepper_el(1, M2IN1, M2IN2);
rs485 rs485(RS485_DIR, RS485_TX_TIME);
endstop switch_az(SW1, DEFAULT_HOME_STATE), switch_el(SW2, DEFAULT_HOME_STATE);

void Homing(int32_t AZsteps, int32_t ELsteps);
void cmd_proc(int &stepAz, int &stepEl);
void error(int num_error);
int deg2step(double deg);
double step2deg(int Step);
bool isNumber(char *input);

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

    /* Serial Communication */
    rs485.begin(BaudRate);

    /* Initial Homing */
    Homing(deg2step(-MAX_M1_ANGLE), deg2step(-MAX_M2_ANGLE));

    /* Initialize WDT */
    watchdog_init(WDT_TIMEOUT);
}

void loop() {
    /* Update WDT */
    watchdog_reset();

    /*Define the steps*/
    static int AZstep = 0, ELstep = 0;

    /*Read the steps from serial*/
    cmd_proc(AZstep, ELstep);
    /*Move the Azimuth & Elevation Motor*/
    stepper_az.moveTo(AZstep);
    stepper_el.moveTo(ELstep);
    stepper_az.run();
    stepper_el.run();
}

/*Homing Function*/
void Homing(int32_t AZsteps, int32_t ELsteps) {
    bool value_Home_AZ = false;
    bool value_Home_EL = false;
    bool isHome_AZ = false;
    bool isHome_EL = false;

    stepper_az.moveTo(AZsteps);
    stepper_el.moveTo(ELsteps);

    /* Homing loop */
    while (isHome_AZ == false || isHome_EL == false) {
        value_Home_AZ = switch_az.get_state();
        value_Home_EL = switch_el.get_state();
        if (value_Home_AZ == true) {
            stepper_az.moveTo(stepper_az.currentPosition());
            isHome_AZ = true;
        }
        if (value_Home_EL == true) {
            stepper_el.moveTo(stepper_el.currentPosition());
            isHome_EL = true;
        }
        if (stepper_az.distanceToGo() == 0 && !isHome_AZ) {
            error(0);
            break;
        }
        if (stepper_el.distanceToGo() == 0 && !isHome_EL) {
            error(1);
            break;
        }
        stepper_az.run();
        stepper_el.run();
    }

    /* Delay to Deccelerate and homing */
    uint32_t time = millis();
    while (millis() - time < HOME_DELAY) {
        stepper_az.run();
        stepper_el.run();
    }

    /*Reset the steps*/
    stepper_az.setCurrentPosition(0);
    stepper_el.setCurrentPosition(0);
}

/*EasyComm 2 Protocol & Calculate the steps*/
void cmd_proc(int &stepAz, int &stepEl) {
    /*Serial*/
    char buffer[BufferSize];
    char incomingByte;
    char *Data = buffer;
    char *rawData;
    static int BufferCnt = 0;
    char data[100];

    String str1, str2, str3, str4, str5;

    double angleAz, angleEl;

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
                    str2 = String(step2deg(stepper_az.currentPosition()), 2);
                    str3 = String(" EL");
                    str4 = String(step2deg(stepper_el.currentPosition()), 2);
                    str5 = String("\n");
                    rs485.print(str1 + str2 + str3 + str4 + str5);
                } else {
                    /*Get the absolute value of angle*/
                    rawData = strtok_r(Data, " ", &Data);
                    strncpy(data, rawData + 2, 10);
                    if (isNumber(data)) {
                        angleAz = atof(data);
                        /*Calculate the steps*/
                        stepAz = deg2step(angleAz);
                    }
                    /*Get the absolute value of angle*/
                    rawData = strtok_r(Data, " ", &Data);
                    if (rawData[0] == 'E' && rawData[1] == 'L') {
                        strncpy(data, rawData + 2, 10);
                        if (isNumber(data)) {
                            angleEl = atof(data);
                            /*Calculate the steps*/
                            stepEl = deg2step(angleEl);
                        }
                    }
                }
            }
            /*Stop Moving*/
            else if (buffer[0] == 'S' && buffer[1] == 'A' && buffer[2] == ' '
                    && buffer[3] == 'S' && buffer[4] == 'E') {
                /*Get position*/
                str1 = String("AZ");
                str2 = String(step2deg(stepper_az.currentPosition()), 2);
                str3 = String(" EL");
                str4 = String(step2deg(stepper_el.currentPosition()), 2);
                str5 = String("\n");
                rs485.print(str1 + str2 + str3 + str4 + str5);

                stepAz = stepper_az.currentPosition();
                stepEl = stepper_el.currentPosition();
            }
            /*Reset the rotator*/
            else if (buffer[0] == 'R' && buffer[1] == 'E' && buffer[2] == 'S'
                    && buffer[3] == 'E' && buffer[4] == 'T') {
                /*Get position*/
                str1 = String("AZ");
                str2 = String(step2deg(stepper_az.currentPosition()), 2);
                str3 = String(" EL");
                str4 = String(step2deg(stepper_el.currentPosition()), 2);
                str5 = String("\n");
                rs485.print(str1 + str2 + str3 + str4 + str5);

                /*Move the steppers to initial position*/
                Homing(deg2step(-MAX_M1_ANGLE), deg2step(-MAX_M2_ANGLE));
                /*Zero the steps*/
                stepAz = 0;
                stepEl = 0;
            } else if (buffer[0] == 'V' && buffer[1] == 'E') {
                str1 = String("SatNOGS Controller v2");
                str2 = String("\n");
                rs485.print(str1 + str2);
            }
            BufferCnt = 0;
        }
        /*Fill the buffer with incoming data*/
        else {
            buffer[BufferCnt] = incomingByte;
            BufferCnt++;
        }
    }
}

/*Error Handling*/
void error(int num_error) {
    switch (num_error) {
    /*Azimuth error*/
    case (0):
        while (1) {
            ;
        }
        /*Elevation error*/
    case (1):
        while (1) {
            ;
        }
    default:
        while (1) {
            ;
        }
    }
}

/*Convert degrees to steps*/
int deg2step(double deg) {
    return (RATIO * SPR * deg / 360);
}

/*Convert steps to degrees*/
double step2deg(int Step) {
    return (360.00 * Step / (SPR * RATIO));
}

/*Check if is argument in number*/
bool isNumber(char *input) {
    for (int i = 0; input[i] != '\0'; i++) {
        if (isalpha(input[i]))
            return false;
    }
    return true;
}
