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

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "bme68x_platform.h"
#include "pico_uart_transports.h"

uint8_t LED_PIN = 25;

uint16_t HEATER_TEMP = 300;
uint16_t HEATER_DURATION = 100;
uint16_t READ_INTERVAL_MS = 1000;
uint16_t MS_TO_US = 1000;
float TEMPERATURE_SCALE = 0.01;
float PRESSURE_SCALE = 0.001;
float HUMIDITY_SCALE = 0.001;
uint8_t N_FIELDS_RESET_VAL = 0;

int ROS_TIMEOUT_MS = 1000;
uint8_t ROS_ATTEMPTS = 120;
int ROS_NUM_HANDLES = 1;
uint8_t ARGC = 0;
char* ARGV = NULL;
char* NAMESPACE = "";
char* NODE_NAME = "air_sensor_test";
char* TEMPERATURE_TOPIC = "temperature";
char* PRESSURE_TOPIC = "pressure";
char* HUMIDITY_TOPIC = "humidity";
char* GAS_RES_TOPIC = "gas_resistance";

typedef enum Level {
    LOW = 0,
    HIGH = 1
} Level;

// I absolutly hate to define objects globally
// but there's no other way to do it under the
// current rclc timer API.
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

int main() {

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

    const rosidl_message_type_support_t * float32_type_support = 
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32);

    platform_interface_init(&bme);
    bme68x_init(&bme);

    conf.filter  = BME68X_FILTER_OFF;
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
        &timer,
        &support,
        RCL_MS_TO_NS(READ_INTERVAL_MS),
        read_timer_callback,
        true);

    rclc_executor_init(&executor, &support.context, ROS_NUM_HANDLES, &allocator);

    rclc_executor_add_timer(&executor, &timer);

    gpio_put(LED_PIN, HIGH);

    rclc_executor_spin(&executor);

    return 0;
}