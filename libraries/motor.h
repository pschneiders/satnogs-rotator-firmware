#ifndef MOTOR_H_
#define MOTOR_H_

class motor {

public:
    motor(uint8_t pwm_pin1, uint8_t pwm_pin2, uint8_t fb_pin, uint8_t en_pin,
            uint8_t sf_pin, uint16_t maxSpeed, uint16_t minSpeed) {
        _pwm_pin1 = pwm_pin1;
        _pwm_pin2 = pwm_pin2;
        _maxSpeed = maxSpeed;
        _minSpeed = minSpeed;
        _fb_pin = fb_pin;
        _en_pin = en_pin;
        _sf_pin = sf_pin;
        stop();
    }

    void init_pin() {
        pinMode(_pwm_pin1, OUTPUT);
        pinMode(_pwm_pin2, OUTPUT);
        /* Feedback and sense */
        pinMode(_fb_pin, INPUT);
        pinMode(_sf_pin,INPUT);
        /* Enable Motors */
        pinMode(_en_pin, OUTPUT);
    }

    void init_timer(uint8_t timer, uint16_t divisor) {
        // Set PWM frequency for D5 & D6
        // set timer 0 divisor to     1 for PWM frequency of 62500.00 Hz
        // set timer 0 divisor to     8 for PWM frequency of  7812.50 Hz
        // set timer 0 divisor to    64 for PWM frequency of   976.56 Hz (The DEFAULT)
        // set timer 0 divisor to   256 for PWM frequency of   244.14 Hz
        // set timer 0 divisor to  1024 for PWM frequency of    61.04 Hz
        if (timer == 0) {
            if (divisor == 1) {
                TCCR0B = (TCCR0B & B11111000) | B00000001;
            } else if (divisor == 8) {
                TCCR0B = (TCCR0B & B11111000) | B00000010;
            } else if (divisor == 64) {
                TCCR0B = (TCCR0B & B11111000) | B00000011;
            } else if (divisor == 256) {
                TCCR0B = (TCCR0B & B11111000) | B00000100;
            } else if (divisor == 1024) {
                TCCR0B = (TCCR0B & B11111000) | B00000101;
            }
        }
        // Set PWM frequency for D9 & D10
        // set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz
        // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
        // set timer 1 divisor to    64 for PWM frequency of   490.20 Hz (The DEFAULT)
        // set timer 1 divisor to   256 for PWM frequency of   122.55 Hz
        // set timer 1 divisor to  1024 for PWM frequency of    30.64 Hz
        if (timer == 1) {
            if (divisor == 1) {
                TCCR1B = (TCCR1B & B11111000) | B00000001;
            } else if (divisor == 8) {
                TCCR1B = (TCCR1B & B11111000) | B00000010;
            } else if (divisor == 64) {
                TCCR1B = (TCCR1B & B11111000) | B00000011;
            } else if (divisor == 256) {
                TCCR1B = (TCCR1B & B11111000) | B00000100;
            } else if (divisor == 1024) {
                TCCR1B = (TCCR1B & B11111000) | B00000101;
            }
        }
        // Set PWM frequency for D3 & D11
        // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz
        // set timer 2 divisor to     8 for PWM frequency of  3921.16 Hz
        // set timer 2 divisor to    32 for PWM frequency of   980.39 Hz
        // set timer 2 divisor to    64 for PWM frequency of   490.20 Hz (The DEFAULT)
        // set timer 2 divisor to   128 for PWM frequency of   245.10 Hz
        // set timer 2 divisor to   256 for PWM frequency of   122.55 Hz
        // set timer 2 divisor to  1024 for PWM frequency of    30.64 Hz
        if (timer == 2) {
            if (divisor == 1) {
                TCCR2B = (TCCR2B & B11111000) | B00000001;
            } else if (divisor == 8) {
                TCCR2B = (TCCR2B & B11111000) | B00000010;
            } else if (divisor == 32){
                TCCR2B = (TCCR2B & B11111000) | B00000011;
            } else if (divisor == 64) {
                TCCR2B = (TCCR2B & B11111000) | B00000100;
            } else if (divisor == 128) {
                TCCR2B = (TCCR2B & B11111000) | B00000101;
            } else if (divisor == 256) {
                TCCR2B = (TCCR2B & B11111000) | B00000110;
            } else if (divisor == 1024) {
                TCCR2B = (TCCR2B & B11111000) | B00000111;
            }
        }
    }

    void enable() {
        digitalWrite(_en_pin, HIGH);
    }

    void disenable() {
        digitalWrite(_en_pin, LOW);
    }

    uint16_t get_load() {
        return analogRead(_fb_pin);
    }

    uint8_t get_fault()
    {
      return digitalRead(_sf_pin);
    }

    void move(int16_t speed) {
        if (speed == 0) {
            stop();
            return;
        }

        if (speed >= 0) {
            speed = speed + _minSpeed;
            if (speed > _maxSpeed)
                speed = _maxSpeed;
            analogWrite(_pwm_pin1, 0);
            analogWrite(_pwm_pin2, speed);
        } else {
            speed = -speed;
            speed = speed + _minSpeed;
            if (speed > _maxSpeed)
                speed = _maxSpeed;
            analogWrite(_pwm_pin1, speed);
            analogWrite(_pwm_pin2, 0);
        }
    }

    void stop() {
        analogWrite(_pwm_pin1, 0);
        analogWrite(_pwm_pin2, 0);
    }

    void set_min(uint16_t min) {
        _maxSpeed = min;
    }

    void set_max(uint16_t max) {
        _maxSpeed = max;
    }

private:
    uint8_t _pwm_pin1, _pwm_pin2, _fb_pin, _en_pin, _sf_pin;
    int16_t _maxSpeed, _minSpeed;
};

#endif /* MOTOR_H_ */
