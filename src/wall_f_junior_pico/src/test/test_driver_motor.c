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

float clamp(float d, float min, float max) {
    const float t = d < min ? min : d;
    return t > max ? max : t;
}

void speed_init_pin(const uint8_t pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint8_t slice_num = pwm_gpio_to_slice_num(pin);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 4.f);
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
            gpio_put(pin_1, 1);
            gpio_put(pin_2, 0);
            break;
        case BACKWARD:
            gpio_put(pin_1, 0);
            gpio_put(pin_2, 1);
            break;
        case STOP:
        default:
            gpio_put(pin_1, 0);
            gpio_put(pin_2, 0);
            break;
    }
}

void set_motor(const Motor* motor, const float throttle) {
    float clipped_throttle = clamp(throttle, -1.0f, 1.0f);

    Direction direction;
    if (clipped_throttle > 0) {
        direction = FORWARD;
    } else if (clipped_throttle < 0) {
        direction = BACKWARD;
    } else {
        direction = STOP;
    }

    set_direction(direction, motor->direction_pin_1, motor->direction_pin_2);

    uint16_t pwm_level = (uint16_t)(fabs(clipped_throttle) * 65535.0f);

    pwm_set_gpio_level(motor->speed_pin, pwm_level);
}

int main()
{
    // Pinout
    const Motor motor_1 = {0, 1, 2};
    const Motor motor_2 = {3, 4, 5};

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

        set_motor(&motor_1, throttle_1);
        set_motor(&motor_2, throttle_2);

        time += step_time;

        sleep_ms(step_delay_ms);
    }

    return 0;
}