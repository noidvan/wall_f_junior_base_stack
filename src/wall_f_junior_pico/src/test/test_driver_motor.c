/*

Copyright © 2025 Yifan Lin

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/

#include <stdio.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/pwm.h"

uint8_t IN1 = 0;
uint8_t IN2 = 1;
uint8_t ENA = 2;
uint8_t IN3 = 3;
uint8_t IN4 = 4;
uint8_t ENB = 5;

float MIN_THROTTLE = -1.0f;
uint16_t ZERO_THROTTLE = 0;
float MAX_THROTTLE = 1.0f;
float PWM_CLK_DIVIDER = 1.0f;
float UINT6_SCALE = 65535.0f;

typedef struct Motor {
    uint8_t direction_pin_1;
    uint8_t direction_pin_2;
    uint8_t speed_pin;
} Motor;

typedef enum Direction {
    FORWARD,
    BACKWARD,
    STOP
} Direction;

typedef enum Level {
    LOW = 0,
    HIGH = 1
} Level;

float clamp(float d, float min, float max) {
    const float t = d < min ? min : d;
    return t > max ? max : t;
}

void speed_init_pin(const uint8_t pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint8_t slice_num = pwm_gpio_to_slice_num(pin);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, PWM_CLK_DIVIDER);
    pwm_init(slice_num, &config, true);
}

void direction_init_pin(const uint8_t pin) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
}

void init_motor(const Motor* motor) {
    direction_init_pin(motor->direction_pin_1);
    direction_init_pin(motor->direction_pin_2);
    speed_init_pin(motor->speed_pin);
}

void set_direction(const Direction direction, const uint8_t pin_1, const uint8_t pin_2) {
    switch (direction) {
        case FORWARD:
            gpio_put(pin_1, HIGH);
            gpio_put(pin_2, LOW);
            break;
        case BACKWARD:
            gpio_put(pin_1, LOW);
            gpio_put(pin_2, HIGH);
            break;
        case STOP:
        default:
            gpio_put(pin_1, LOW);
            gpio_put(pin_2, LOW);
            break;
    }
}

void set_motor(const Motor* motor, const bool armed, const float throttle) {
    if (armed) {
        float clipped_throttle = clamp(throttle, MIN_THROTTLE, MAX_THROTTLE);

        Direction direction;
        if (clipped_throttle > ZERO_THROTTLE) {
            direction = FORWARD;
        } else if (clipped_throttle < ZERO_THROTTLE) {
            direction = BACKWARD;
        } else {
            direction = STOP;
        }
    
        uint16_t pwm_level = (uint16_t)(fabs(clipped_throttle) * UINT6_SCALE);

        set_direction(direction, motor->direction_pin_1, motor->direction_pin_2);
        pwm_set_gpio_level(motor->speed_pin, pwm_level);
    } else {
        set_direction(STOP, motor->direction_pin_1, motor->direction_pin_2);
        pwm_set_gpio_level(motor->speed_pin, ZERO_THROTTLE);
    }

}

int main()
{
    // Pinout
    const Motor motor_1 = {IN1, IN2, ENA};
    const Motor motor_2 = {IN3, IN4, ENB};

    // Init motors
    init_motor(&motor_1);
    init_motor(&motor_2);

    const float frequency = 0.5f;
    const float amplitude = 1.0f;
    const float offset = 0.0f;
    const float period = 1.0f / frequency;
    const float step_time = 0.01f;
    const uint32_t step_delay_ms = (uint32_t)(step_time * 1000);

    float time = 0.0f;

    while (true)
    {
        float throttle_1 = amplitude * sinf(2.0f * M_PI * frequency * time) + offset;
        float throttle_2 = amplitude * sinf(2.0f * M_PI * frequency * time + M_PI);

        set_motor(&motor_1, true, throttle_1);
        set_motor(&motor_2, true, throttle_2);

        time += step_time;

        sleep_ms(step_delay_ms);
    }

    return 0;
}