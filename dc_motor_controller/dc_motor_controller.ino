/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
#define SAMPLE_TIME        0.1  /* Control loop in s */
#define RATIO              54   /* Gear ratio */
#define MAX_PWM            180  /* Set max Speed */
#define MIN_PWM            5    /* Set min Speed */
#define POSITION_DEADZONE  0.2  /* Control dead zone */
#define PCA9540_ID         0x70 /* I2C Multiplexer ID */
#define PCA9540_CH0        0x04 /* I2C Multiplexer CHO */
#define PCA9540_CH1        0x05 /* I2C Multiplexer CH1 */
#define TC74_ID            0x48 /* Temperature Sensor ID */
#define OVER_TEMP          60   /* Over temperature limit */
#define ENC_RATIO          2    /* Encoder AS5601 gear ratio */
#define MIN_M1_ANGLE       0    /* Minimum angle of azimuth */
#define MAX_M1_ANGLE       360  /* Maximum angle of azimuth */
#define MIN_M2_ANGLE       0    /* Minimum angle of elevation */
#define MAX_M2_ANGLE       90   /* Maximum angle of elevation */
#define DEFAULT_HOME_STATE HIGH /* Change to LOW according to Home sensor*/
#define HOME_SPEED         100  /* Set speed to find home, duty cycle 255->100% */
#define HOME_DELAY         1000 /* Time for homing Deceleration in millisecond */
/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
#include <PID_v1.h>
#include <Wire.h>
#include "../libraries/globals.h"
#include "../libraries/easycomm.h"
#include "../libraries/rotator_pins.h"
#include "../libraries/endstop.h"
#include "../libraries/watchdog.h"
#include "../libraries/i2c_mux.h"
#include "../libraries/tc74.h"
#include "../libraries/motor.h"
#include "../libraries/as5601.h"
/******************************************************************************/
uint32_t t_run = 0;
easycomm comm;
motor motor_az(M1IN1, M1IN2, M1FB, MOTOR_EN, M1SF, MAX_PWM, MIN_PWM);
motor motor_el(M2IN1, M2IN2, M2FB, MOTOR_EN, M2SF, MAX_PWM, MIN_PWM);
PID pid_az(&control_az.input, &control_az.u, &control_az.setpoint, control_az.p,
           control_az.i, control_az.d, P_ON_E, DIRECT);
PID pid_el(&control_el.input, &control_el.u, &control_el.setpoint, control_el.p,
           control_el.i, control_el.d, P_ON_E, DIRECT);
endstop switch_az(SW1, DEFAULT_HOME_STATE), switch_el(SW2, DEFAULT_HOME_STATE);
i2c_mux pca9540(PCA9540_ID, PCA9540_CH0, PCA9540_CH1);
tc74 temp_sensor(TC74_ID);
AS5601 encoder_az, encoder_el;
wdt_timer wdt;
/******************************************************************************/
enum _rotator_error homing();
/******************************************************************************/

void setup() {
    /* Homing switch */
    switch_az.init();
    switch_el.init();

    /* Serial Communication */
    comm.easycomm_init();

    /* Initialize DC motors */
    motor_az.init_pin();
    motor_az.init_timer(1, 8);
    motor_az.enable();
    motor_el.init_pin();
    motor_el.init_timer(2, 8);
    motor_el.enable();

    /* Initialize I2C MUX */
    pca9540.init();
    /* Initialize rotary encoders */
    encoder_az.set_gear_ratio(ENC_RATIO);
    encoder_el.set_gear_ratio(ENC_RATIO);

    /* Initialize control parameters */
    pid_az.SetSampleTime(SAMPLE_TIME); // in seconds
    pid_az.SetOutputLimits(-MAX_PWM, MAX_PWM );
    pid_az.SetMode(AUTOMATIC);
    pid_el.SetSampleTime(SAMPLE_TIME); // in seconds
    pid_el.SetOutputLimits(-MAX_PWM, MAX_PWM);
    pid_el.SetMode(AUTOMATIC);

    /* Initialize WDT */
    wdt.watchdog_init();
}

void loop() {
    /* Update WDT */
    wdt.watchdog_reset();

    /* Get end stop status */
    rotator.switch_az = switch_az.get_state();
    rotator.switch_el = switch_el.get_state();

    /*Read the steps from serial*/
    comm.easycomm_proc();

    /* Get Motor driver status */
    rotator.fault_az = motor_az.get_fault();
    rotator.fault_el = motor_el.get_fault();
    if (rotator.fault_az == LOW || rotator.fault_el == LOW) {
        rotator.rotator_status = error;
        rotator.rotator_error = motor_error;
    }

    /* Get Temperature */
    pca9540.set_channel(PCA9540_CH1);
    temp_sensor.wake_up();
    rotator.inside_temperature = temp_sensor.get_temp();
    temp_sensor.sleep();
    if (rotator.inside_temperature > OVER_TEMP) {
        rotator.rotator_status = error;
        rotator.rotator_error = over_temperature;
    }
    /* Get encoders */
    pca9540.set_channel(PCA9540_CH0);
    encoder_az.get_pos(&control_az.input);
    pca9540.set_channel(PCA9540_CH1);
    encoder_el.get_pos(&control_el.input);

    /* Check rotator status */
    if (rotator.rotator_status != error) {
        if (rotator.homing_flag == false) {
            /* Homing */
            rotator.control_mode = position;
            rotator.rotator_error = homing();
            if (rotator.rotator_error == no_error) {
                rotator.rotator_status = idle;
                rotator.homing_flag = true;
            } else {
                rotator.rotator_status = error;
                rotator.rotator_error = homing_error;
            }
        } else {
            /* Control Loop */
            if (millis() - t_run > SAMPLE_TIME * 1000) {
                /* Update control gains */
                pid_az.SetTunings(control_az.p, control_az.i, control_az.d);
                pid_el.SetTunings(control_el.p, control_el.i, control_el.d);
                /* Move the Azimuth & Elevation Motor */
                if (rotator.control_mode ==  speed) {
                    control_az.setpoint += control_az.setpoint_speed
                                           * SAMPLE_TIME;
                    control_el.setpoint += control_el.setpoint_speed
                                           * SAMPLE_TIME;
                    rotator.rotator_status = moving;
                } else {
                    rotator.rotator_status = pointing;
                }
                pid_az.Compute();
                motor_az.move(control_az.u);
                pid_el.Compute();
                motor_el.move(control_el.u);
                control_az.speed = (control_az.input - control_az.input_prv)
                                   / SAMPLE_TIME;
                control_az.input_prv = control_az.input;
                control_el.speed = (control_el.input - control_el.input_prv)
                                   / SAMPLE_TIME;
                control_el.input_prv = control_el.input;
                t_run = millis();
                /* Idle rotator, dead-band */
                if ((abs(control_az.setpoint - control_az.input) <=
                    POSITION_DEADZONE || (control_az.speed == 0)) &&
                    (abs(control_el.setpoint - control_el.input) <=
                    POSITION_DEADZONE || (control_el.speed == 0))) {
                    rotator.rotator_status = idle;
                }
            }
        }
    } else {
        /* error handler */
        motor_az.stop();
        motor_az.disenable();
        motor_el.stop();
        motor_el.disenable();
        if (rotator.rotator_error != homing_error) {
            /* Reset Error flags */
            rotator.rotator_error = no_error;
            rotator.rotator_status = idle;
        }
    }
}

/* Homing Function */
enum _rotator_error homing() {
    bool isHome_az = false;
    bool isHome_el = false;

    /* Resest position */
    pca9540.set_channel(PCA9540_CH0);
    encoder_az.set_zero();
    pca9540.set_channel(PCA9540_CH1);
    encoder_el.set_zero();

    /* Move motors with ~constant speed */
    motor_az.move(-HOME_SPEED);
    motor_el.move(-HOME_SPEED);

    /* Homing loop */
    while (isHome_az == false || isHome_el == false) {
        /* Update WDT */
        wdt.watchdog_reset();
        if (switch_az.get_state() == true && !isHome_az) {
            /* Find azimuth home */
            motor_az.stop();
            isHome_az = true;
        }
        if (switch_el.get_state() == true && !isHome_el) {
            /* Find elevation home */
            motor_el.stop();
            isHome_el = true;
        }
        /* Get current position */
        pca9540.set_channel(PCA9540_CH0);
        encoder_az.get_pos(&control_az.input);
        pca9540.set_channel(PCA9540_CH1);
        encoder_el.get_pos(&control_el.input);
        if ((abs(control_az.input) > MAX_M1_ANGLE && !isHome_az)
         || (abs(control_el.input) > MAX_M2_ANGLE && !isHome_el)) {
            return homing_error;
        }
    }

    /* Resest position */
    pca9540.set_channel(PCA9540_CH0);
    encoder_az.init_zero();
    encoder_az.set_zero();
    control_az.setpoint = 0;
    pca9540.set_channel(PCA9540_CH1);
    encoder_el.init_zero();
    encoder_el.set_zero();
    control_el.setpoint = 0;

    return no_error;
}
