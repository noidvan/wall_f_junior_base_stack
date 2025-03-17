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

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/bool.h>
#include <rmw_microros/rmw_microros.h>


#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "pico_uart_transports.h"

uint8_t IN1 = 6;
uint8_t IN2 = 7;
uint8_t ENA = 2;
uint8_t IN3 = 3;
uint8_t IN4 = 4;
uint8_t ENB = 5;
uint8_t LED_PIN = 25;

float MIN_THROTTLE = -1.0f;
uint16_t ZERO_THROTTLE = 0;
float MAX_THROTTLE = 1.0f;
float PWM_CLK_DIVIDER = 1.0f;
float UINT6_SCALE = 65535.0f;

int ROS_TIMEOUT_MS = 1000;
uint8_t ROS_ATTEMPTS = 120;
int ROS_NUM_HANDLES = 4;
char* NAMESPACE = "";
char* NODE_NAME = "motor_test";
char* MOTOR_1_SETPOINT_TOPIC = "motor_1_setpoint";
char* MOTOR_1_ARM_TOPIC = "motor_1_arm";
char* MOTOR_2_SETPOINT_TOPIC = "motor_2_setpoint";
char* MOTOR_2_ARM_TOPIC = "motor_2_arm";

typedef struct Motor {
    uint8_t direction_pin_1;
    uint8_t direction_pin_2; 
    uint8_t speed_pin;
    bool armed;
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
    if (armed && motor->armed) {
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

void setpoint_callback(const void * msgin, const Motor* motor) {
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
    if (msg == NULL) {
        set_motor(motor, false, (float)ZERO_THROTTLE);
    } else {
        set_motor(motor, motor->armed, msg->data);
    }
}

void arm_callback(const void * msgin, Motor* motor) {
    const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
    if (msg == NULL) {
        motor->armed = false;
    } else {
        motor->armed = msg->data;
    }
}

int main()
{
    // Pinout
    Motor motor_1 = {IN1, IN2, ENA, false};
    Motor motor_2 = {IN3, IN4, ENB, false};

    // Init motors
    init_motor(&motor_1);
    init_motor(&motor_2);

    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    rcl_subscription_t motor_1_setpoint_sub;
    rcl_subscription_t motor_2_setpoint_sub;
    
    rcl_subscription_t motor_1_arm_sub;
    rcl_subscription_t motor_2_arm_sub;

    const rosidl_message_type_support_t * float32_type_support = 
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32);
    const rosidl_message_type_support_t * bool_type_support = 
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool);

    std_msgs__msg__Float32 motor_1_setpoint_msg;
    std_msgs__msg__Bool motor_1_arm_msg;


    std_msgs__msg__Float32 motor_2_setpoint_msg;
    std_msgs__msg__Bool motor_2_arm_msg;

    allocator = rcl_get_default_allocator();

    rcl_ret_t ret = rmw_uros_ping_agent(ROS_TIMEOUT_MS, ROS_ATTEMPTS);

    if (ret != RCL_RET_OK)
    {
        return ret;
    }

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, NODE_NAME, NAMESPACE, &support);

    rclc_subscription_init_default(
        &motor_1_setpoint_sub,
        &node,
        float32_type_support,
        MOTOR_1_SETPOINT_TOPIC);

     rclc_subscription_init_default(
        &motor_1_arm_sub,
        &node,
        bool_type_support,
        MOTOR_1_ARM_TOPIC);

    rclc_subscription_init_default(
        &motor_2_setpoint_sub,
        &node,
        float32_type_support,
        MOTOR_2_SETPOINT_TOPIC);

     rclc_subscription_init_default(
        &motor_2_arm_sub,
        &node,
        bool_type_support,
        MOTOR_2_ARM_TOPIC);

    rclc_executor_init(&executor, &support.context, ROS_NUM_HANDLES, &allocator);

    rclc_executor_add_subscription_with_context(
        &executor,
        &motor_1_setpoint_sub,
        &motor_1_setpoint_msg,
        &setpoint_callback,
        &motor_1,
        ON_NEW_DATA
    );

    rclc_executor_add_subscription_with_context(
        &executor,
        &motor_1_arm_sub,
        &motor_1_arm_msg,
        &arm_callback,
        &motor_1,
        ON_NEW_DATA
    );

    rclc_executor_add_subscription_with_context(
        &executor,
        &motor_2_setpoint_sub,
        &motor_2_setpoint_msg,
        &setpoint_callback,
        &motor_2,
        ON_NEW_DATA
    );

    rclc_executor_add_subscription_with_context(
        &executor,
        &motor_2_arm_sub,
        &motor_2_arm_msg,
        &arm_callback,
        &motor_2,
        ON_NEW_DATA
    );
    
    gpio_put(LED_PIN, 1);

    rclc_executor_spin(&executor);

    return 0;
}