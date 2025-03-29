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
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/bool.h>
#include <rmw_microros/rmw_microros.h>


#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "bme68x_platform.h"
#include "pico_uart_transports.h"

static const uint8_t IN1 = 7;
static const uint8_t IN2 = 8;
static const uint8_t ENA = 6;
static const uint8_t IN3 = 9;
static const uint8_t IN4 = 10;
static const uint8_t ENB = 11;
static const uint8_t LED_PIN = 25;

static const uint8_t STEP_PIN = 12;
static const uint8_t DIR_PIN  = 13;

static const uint8_t ADC_PIN = 26;

float MIN_THROTTLE = -1.0f;
uint16_t ZERO_THROTTLE = 0;
float MAX_THROTTLE = 1.0f;
float PWM_CLK_DIVIDER = 1.0f;
float UINT6_SCALE = 65535.0f;

int ROS_TIMEOUT_MS = 1000;
uint8_t ROS_ATTEMPTS = 120;
int ROS_NUM_HANDLES = 8;
uint8_t ARGC = 0;
char* ARGV = NULL;
char* NAMESPACE = "";
char* NODE_NAME = "rear_agent";
char* MOTOR_1_SETPOINT_TOPIC = "motor_left_setpoint";
char* MOTOR_1_ARM_TOPIC = "rear_driver_motors_arm";
char* MOTOR_2_SETPOINT_TOPIC = "motor_right_setpoint";
char* MOTOR_2_ARM_TOPIC = "rear_driver_motors_arm";
static const char* STEPPER_TOPIC = "soil_stepper_motors";
static const char* ADC_TOPIC = "soil_moisture";
char* TEMPERATURE_TOPIC = "temperature";
char* PRESSURE_TOPIC = "pressure";
char* HUMIDITY_TOPIC = "humidity";
char* GAS_RES_TOPIC = "gas_resistance";

static const uint32_t STEP_TIMER_NS = 4000;  // Timer period in ms; sets stepping speed
static const uint32_t STEP_PULSE_NS = 2000;   // Pulse width for step pin (microseconds)

static const uint32_t ADC_TIMER_MS= 1000;
static const float ADC_CONVERSION_FACTOR = 1;

uint16_t HEATER_TEMP = 300;
uint16_t HEATER_DURATION = 100;
uint16_t READ_INTERVAL_MS = 1000;
uint16_t MS_TO_US = 1000;
float TEMPERATURE_SCALE = 0.01;
float PRESSURE_SCALE = 0.001;
float HUMIDITY_SCALE = 0.001;
uint8_t N_FIELDS_RESET_VAL = 0;

struct bme68x_dev bme;
struct bme68x_conf conf;
struct bme68x_heatr_conf heatr_conf;
struct bme68x_data data;
uint32_t del_period;
uint8_t n_fields;

std_msgs__msg__Float32 temp_msg;
std_msgs__msg__Float32 pressure_msg;
std_msgs__msg__Float32 humidity_msg;
std_msgs__msg__Float32 gas_res_msg;

rcl_publisher_t temp_pub;
rcl_publisher_t pressure_pub;
rcl_publisher_t humidity_pub;
rcl_publisher_t gas_res_pub;

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

static volatile int8_t direction_state = 0; // -1, 0, +1

static rcl_node_t node;
static rcl_allocator_t allocator;
static rclc_support_t support;
static rclc_executor_t executor;

static rcl_subscription_t motor_1_setpoint_sub;
static rcl_subscription_t motor_2_setpoint_sub;
static rcl_subscription_t sub_direction;
static rcl_subscription_t motor_1_arm_sub;
static rcl_subscription_t motor_2_arm_sub;
static rcl_timer_t stepper_timer;
static rcl_timer_t adc_timer;
static rcl_timer_t air_timer;

rcl_publisher_t adc_pub;

static std_msgs__msg__Float32 motor_1_setpoint_msg;
static std_msgs__msg__Bool motor_1_arm_msg;
static std_msgs__msg__Float32 motor_2_setpoint_msg;
static std_msgs__msg__Bool motor_2_arm_msg;
static std_msgs__msg__Int8 direction_msg; 
std_msgs__msg__Float32 adc_msg;

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

static void direction_callback(const void * msgin)
{
    const std_msgs__msg__Int8 * msg = (const std_msgs__msg__Int8 *)msgin;
    if (msg == NULL) {
        // This should never happen once you pass a valid message buffer
        direction_state = 0;
        return;
    }

    if (msg->data > 0) {
        direction_state = 1;   // clockwise
    } else if (msg->data < 0) {
        direction_state = -1;  // counterclockwise
    } else {
        direction_state = 0;   // stop
    }
}

static void step_timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
    (void) last_call_time;
    if (timer == NULL) {
        return;
    }

    if (direction_state != 0)
    {
        // Set DIR pin based on g_direction_state
        //  1 => DIR pin = HIGH
        // -1 => DIR pin = LOW
        bool dir_pin_level = (direction_state == 1) ? true : false;
        gpio_put(DIR_PIN, dir_pin_level);

        // Generate a single rising->falling pulse on STEP pin
        //  We use a short busy-wait for pulse width
        gpio_put(STEP_PIN, true);
        sleep_us(STEP_PULSE_NS);    // Wait high
        gpio_put(STEP_PIN, false);
        // The motor sees this as one step; frequency is determined by how often
        // step_timer_callback gets called (STEP_TIMER_MS).
    }
}

void adc_timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
    (void)timer;
    (void)last_call_time;

    // Select ADC input 0 => GPIO 26
    adc_select_input(0);

    // 12-bit read: range 0..4095
    uint16_t raw_value = adc_read();

    // Convert to voltage
    float voltage = raw_value * ADC_CONVERSION_FACTOR;

    // Assign to message and publish
    adc_msg.data = voltage;
    rcl_publish(&adc_pub, &adc_msg, NULL);
}

void read_timer_callback(rcl_timer_t* timer, int64_t last_call_time) {

  bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);

  del_period = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &bme) +
      (heatr_conf.heatr_dur * MS_TO_US);
  bme.delay_us(del_period, bme.intf_ptr);

  bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &bme);

  if (n_fields) {
      temp_msg.data = data.temperature * TEMPERATURE_SCALE;
      pressure_msg.data = data.pressure * PRESSURE_SCALE;
      humidity_msg.data = data.humidity * HUMIDITY_SCALE;
      gas_res_msg.data = data.gas_resistance;

      rcl_ret_t temp_pub_ret = rcl_publish(&temp_pub, &temp_msg, NULL);
      rcl_ret_t pres_pub_ret = rcl_publish(&pressure_pub, &pressure_msg, NULL);
      rcl_ret_t hum_pub_ret = rcl_publish(&humidity_pub, &humidity_msg, NULL);
      rcl_ret_t gas_pub_ret = rcl_publish(&gas_res_pub, &gas_res_msg, NULL);

      n_fields = N_FIELDS_RESET_VAL;
  }

}

int main()
{
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    // Pinout
    Motor motor_1 = {IN1, IN2, ENA, false};
    Motor motor_2 = {IN3, IN4, ENB, false};

    // Init motors
    init_motor(&motor_1);
    init_motor(&motor_2);

    // Set up STEP and DIR pins
    gpio_init(STEP_PIN);
    gpio_set_dir(STEP_PIN, GPIO_OUT);
    gpio_put(STEP_PIN, false);

    gpio_init(DIR_PIN);
    gpio_set_dir(DIR_PIN, GPIO_OUT);
    gpio_put(DIR_PIN, false);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    adc_init();
    adc_gpio_init(ADC_PIN);    // Connect GPIO 26 to ADC channel 0
    adc_select_input(0);

    const rosidl_message_type_support_t * float32_type_support = 
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32);
    const rosidl_message_type_support_t * bool_type_support = 
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool);

    const rosidl_message_type_support_t* int8_type_support =
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8);

    platform_interface_init(&bme);
    bme68x_init(&bme);

    conf.odr     = BME68X_ODR_NONE;
    conf.os_hum  = BME68X_OS_16X;
    conf.os_pres = BME68X_OS_1X;
    conf.os_temp = BME68X_OS_2X;
    bme68x_set_conf(&conf, &bme);

    heatr_conf.enable     = BME68X_ENABLE;
    heatr_conf.heatr_temp = HEATER_TEMP;
    heatr_conf.heatr_dur  = HEATER_DURATION;
    bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &bme);

    allocator = rcl_get_default_allocator();

    rcl_ret_t ret = rmw_uros_ping_agent(ROS_TIMEOUT_MS, ROS_ATTEMPTS);

    if (ret != RCL_RET_OK) {
        return ret;
    }

    rclc_support_init(&support, ARGC, ARGV, &allocator);

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

    rclc_subscription_init_default(
        &sub_direction,
        &node,
        int8_type_support,
        STEPPER_TOPIC
    );

    rclc_timer_init_default(
        &stepper_timer,
        &support,
        STEP_TIMER_NS,
        step_timer_callback
    );

    rclc_publisher_init_default(
        &adc_pub,
        &node,
        float32_type_support,
        ADC_TOPIC
    );

    // Create the timer (1 second)
    rclc_timer_init_default2(
        &adc_timer,
        &support,
        RCL_MS_TO_NS(ADC_TIMER_MS),
        adc_timer_callback,
        true
    );

    rclc_publisher_init_default(
        &temp_pub,
        &node,
        float32_type_support,
        TEMPERATURE_TOPIC);

    rclc_publisher_init_default(
        &pressure_pub,
        &node,
        float32_type_support,
        PRESSURE_TOPIC);

    rclc_publisher_init_default(
        &humidity_pub,
        &node,
        float32_type_support,
        HUMIDITY_TOPIC);

    rclc_publisher_init_default(
        &gas_res_pub,
        &node,
        float32_type_support,
        GAS_RES_TOPIC);

    rclc_timer_init_default2(
        &air_timer,
        &support,
        RCL_MS_TO_NS(READ_INTERVAL_MS),
        read_timer_callback,
        true);

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
    
    // Add the direction subscriber
    rclc_executor_add_subscription(
        &executor,
        &sub_direction,
        &direction_msg,
        &direction_callback,
        ON_NEW_DATA
    );

    // Add the step timer
    rclc_executor_add_timer(&executor, &stepper_timer);
    rclc_executor_add_timer(&executor, &adc_timer);
    rclc_executor_add_timer(&executor, &air_timer);
    gpio_put(LED_PIN, HIGH);

    rclc_executor_spin(&executor);

    return 0;
}