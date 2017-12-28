#include <AccelStepper.h>
#include "rs485.h"

#define DIR_AZ 5 /*PIN for Azimuth Direction*/
#define STEP_AZ 6 /*PIN for Azimuth Steps*/
#define DIR_EL 3 /*PIN for Elevation Direction*/
#define STEP_EL 11 /*PIN for Elevation Steps*/

#define SPR 200 /*Step Per Revolution*/
#define RATIO 54 /*Gear ratio*/

#define HOME_AZ 9 /*Homing switch for Azimuth*/
#define HOME_EL 4 /*Homing switch for Elevation*/

#define MAX_AZ_ANGLE 365 /*Maximum Angle of Azimuth for homing scanning*/
#define MAX_EL_ANGLE 365 /*Maximum Angle of Elevation for homing scanning*/

#define MAX_SPEED 300
#define MAX_ACCELERATION 100

#define MIN_PULSE_WIDTH 20 /*in microsecond*/

#define DEFAULT_HOME_STATE HIGH /*Change to LOW according to Home sensor*/

#define HOME_DELAY 6000 /*Time for homing Decceleration in millisecond*/

#define BufferSize 256
#define BaudRate 19200

#define MOTOR_EN 8
AccelStepper stepper_az(1, STEP_AZ, DIR_AZ);
AccelStepper stepper_el(1, STEP_EL, DIR_EL);

#define RS485_DIR 2
#define RS485_TX_TIME 8 // ms
rs485 rs485(RS485_DIR, RS485_TX_TIME);

void Homing(int AZsteps, int ELsteps);
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

    /*Homing switch*/
    pinMode(HOME_AZ, INPUT_PULLUP);
    pinMode(HOME_EL, INPUT_PULLUP);
    /*Serial Communication*/
    rs485.begin(BaudRate);
    /*Initial Homing*/
    Homing(deg2step(-MAX_AZ_ANGLE), deg2step(-MAX_EL_ANGLE));
}

void loop() {
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
void Homing(int AZsteps, int ELsteps) {
    int value_Home_AZ = DEFAULT_HOME_STATE;
    int value_Home_EL = DEFAULT_HOME_STATE;
    bool isHome_AZ = false;
    bool isHome_EL = false;

    stepper_az.moveTo(AZsteps);
    stepper_el.moveTo(ELsteps);

    while (isHome_AZ == false || isHome_EL == false) {
        value_Home_AZ = digitalRead(HOME_AZ);
        value_Home_EL = digitalRead(HOME_EL);
        /*Change to LOW according to Home sensor*/
        if (value_Home_AZ == DEFAULT_HOME_STATE) {
            stepper_az.moveTo(stepper_az.currentPosition());
            isHome_AZ = true;
        }
        /*Change to LOW according to Home sensor*/
        if (value_Home_EL == DEFAULT_HOME_STATE) {
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
    /*Delay to Deccelerate*/
    long time = millis();
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
                Homing(deg2step(-MAX_AZ_ANGLE), deg2step(-MAX_EL_ANGLE));
                /*Zero the steps*/
                stepAz = 0;
                stepEl = 0;
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
