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
#include "hardware/adc.h"
#include "pico_uart_transports.h"

// -----------------------------------------------------------
// Configuration
// -----------------------------------------------------------

static const uint8_t LED_PIN = 25;
static const uint8_t ADC_PIN = 26;

// micro-ROS communication settings
static const int     ROS_TIMEOUT_MS    = 1000;
static const uint8_t ROS_ATTEMPTS      = 120;
static const int     ROS_NUM_HANDLES   = 1;   // We'll only use 1 timer
static const char*   NAMESPACE         = "";
static const char*   NODE_NAME         = "soil_test_node";
static const char*   ADC_TOPIC         = "soil_moisture";

// Timer period: 1 second
static const uint32_t ADC_TIMER_MS     = 1000;

// ADC conversion factor: 12-bit reading, 3.3 V reference
// static const float ADC_CONVERSION_FACTOR = 3.3f / (1 << 12);
static const float ADC_CONVERSION_FACTOR = 1;

// -----------------------------------------------------------
// Global micro-ROS handles
// -----------------------------------------------------------
rcl_node_t      g_node;
rcl_timer_t     g_timer;
rcl_publisher_t g_adc_pub;
rcl_allocator_t g_allocator;
rclc_support_t  g_support;
rclc_executor_t g_executor;

// We need a global (or static) message instance for micro-ROS
std_msgs__msg__Float32 g_adc_msg;

// -----------------------------------------------------------
// Timer Callback
// -----------------------------------------------------------
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
    g_adc_msg.data = voltage;
    rcl_publish(&g_adc_pub, &g_adc_msg, NULL);
}

// -----------------------------------------------------------
// main()
// -----------------------------------------------------------
int main(void)
{
    // ---------------------------
    // Basic Pico init
    // ---------------------------
    stdio_init_all();

    // Optional: LED to show we're running
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);

    // ---------------------------
    // ADC Setup on GPIO 26
    // ---------------------------
    adc_init();
    adc_gpio_init(ADC_PIN);    // Connect GPIO 26 to ADC channel 0
    adc_select_input(0);

    // ---------------------------
    // micro-ROS transport init
    // (Pico UART or custom transport)
    // ---------------------------
    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );

    // Attempt to contact the micro-ROS Agent
    rcl_ret_t ret = rmw_uros_ping_agent(ROS_TIMEOUT_MS, ROS_ATTEMPTS);
    if (ret != RCL_RET_OK) {
        // If we never connected, exit early
        return ret;
    }

    // ---------------------------
    // micro-ROS structures
    // ---------------------------
    g_allocator = rcl_get_default_allocator();

    // Initialize rclc support
    rclc_support_init(&g_support, 0, NULL, &g_allocator);

    // Create our node
    rclc_node_init_default(&g_node, NODE_NAME, NAMESPACE, &g_support);

    // Create the ADC publisher (Float32)
    const rosidl_message_type_support_t* float32_type_support =
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32);

    rclc_publisher_init_default(
        &g_adc_pub,
        &g_node,
        float32_type_support,
        ADC_TOPIC
    );

    // Create the timer (1 second)
    rclc_timer_init_default2(
        &g_timer,
        &g_support,
        RCL_MS_TO_NS(ADC_TIMER_MS),
        adc_timer_callback,
        true
    );

    // Create the executor
    rclc_executor_init(&g_executor, &g_support.context, ROS_NUM_HANDLES, &g_allocator);

    // Add our single timer to the executor
    rclc_executor_add_timer(&g_executor, &g_timer);

    // Turn on LED to show we are up
    gpio_put(LED_PIN, 1);

    // ---------------------------
    // Spin forever
    // ---------------------------
    rclc_executor_spin(&g_executor);

    // (In embedded apps we usually never leave main)
    return 0;
}
