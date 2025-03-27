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
#include <std_msgs/msg/int8.h>
#include <rmw_microros/rmw_microros.h>

// Pico SDK
#include "pico/stdlib.h"
#include "pico_uart_transports.h"

// ---------- Pin Definitions ----------
static const uint8_t STEP_PIN = 2;
static const uint8_t DIR_PIN  = 3;
static const uint8_t LED_PIN  = 25;

// ---------- micro-ROS Configuration ----------
static const int     ROS_TIMEOUT_MS = 1000;
static const uint8_t ROS_ATTEMPTS   = 120;
static const int     ROS_NUM_HANDLES = 2;  // 1 subscriber + 1 timer

static const char*   NAMESPACE        = "";
static const char*   NODE_NAME        = "stepper_node";
static const char*   STEPPER_TOPIC    = "stepper_direction"; // std_msgs/Int8

// ---------- Stepper Behavior Configuration ----------
/*
   The subscriber will update a global direction variable:
     +1 => clockwise
      0 => stop
     -1 => counterclockwise
   A timer will produce step pulses at a fixed frequency (no speed control).
*/
static const uint32_t STEP_TIMER_NS = 4000;  // Timer period in ms; sets stepping speed
static const uint32_t STEP_PULSE_NS = 2000;   // Pulse width for step pin (microseconds)

// ---------- Global State ----------
static volatile int8_t g_direction_state = 0; // -1, 0, +1

// ---------- micro-ROS Entities ----------
static rcl_node_t      g_node;
static rcl_subscription_t g_sub_direction;
static rcl_timer_t     g_timer;
static rcl_allocator_t g_allocator;
static rclc_support_t  g_support;
static rclc_executor_t g_executor;
static std_msgs__msg__Int8 direction_msg; 

// ---------- Callback: Subscriber for Direction ----------
static void direction_callback(const void * msgin)
{
    const std_msgs__msg__Int8 * msg = (const std_msgs__msg__Int8 *)msgin;
    if (msg == NULL) {
        // This should never happen once you pass a valid message buffer
        g_direction_state = 0;
        return;
    }

    if (msg->data > 0) {
        g_direction_state = 1;   // clockwise
    } else if (msg->data < 0) {
        g_direction_state = -1;  // counterclockwise
    } else {
        g_direction_state = 0;   // stop
    }
}

// ---------- Callback: Timer for Step Pulses ----------
static void step_timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
    (void) last_call_time;
    if (timer == NULL) {
        return;
    }

    if (g_direction_state != 0)
    {
        // Set DIR pin based on g_direction_state
        //  1 => DIR pin = HIGH
        // -1 => DIR pin = LOW
        bool dir_pin_level = (g_direction_state == 1) ? true : false;
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

// ---------- Main Application ----------
int main()
{
    // Setup default UART-based transport for micro-ROS
    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );

    // Initialize chosen GPIO pins
    stdio_init_all();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);

    // Set up STEP and DIR pins
    gpio_init(STEP_PIN);
    gpio_set_dir(STEP_PIN, GPIO_OUT);
    gpio_put(STEP_PIN, false);

    gpio_init(DIR_PIN);
    gpio_set_dir(DIR_PIN, GPIO_OUT);
    gpio_put(DIR_PIN, false);

    // Attempt to ping the micro-ROS Agent
    rcl_ret_t ret = rmw_uros_ping_agent(ROS_TIMEOUT_MS, ROS_ATTEMPTS);
    if (ret != RCL_RET_OK) {
        // If we cannot contact the agent, return early
        return ret;
    }

    g_allocator = rcl_get_default_allocator();

    // Initialize micro-ROS support structures
    rclc_support_init(&g_support, 0, NULL, &g_allocator);
    rclc_node_init_default(&g_node, NODE_NAME, NAMESPACE, &g_support);

    // Create subscriber for std_msgs/Int8
    const rosidl_message_type_support_t* int8_type_support =
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8);

    rclc_subscription_init_default(
        &g_sub_direction,
        &g_node,
        int8_type_support,
        STEPPER_TOPIC
    );

    // Create a timer for step pulses
    // The period sets how frequently we generate a STEP pulse (if direction != 0)
    rclc_timer_init_default(
        &g_timer,
        &g_support,
        STEP_TIMER_NS,
        step_timer_callback
    );

    // Create an executor to handle subscription + timer
    rclc_executor_init(&g_executor, &g_support.context, ROS_NUM_HANDLES, &g_allocator);

    // Add the direction subscriber
    rclc_executor_add_subscription(
        &g_executor,
        &g_sub_direction,
        &direction_msg,
        &direction_callback,
        ON_NEW_DATA
    );

    // Add the step timer
    rclc_executor_add_timer(&g_executor, &g_timer);

    // Light the LED to indicate we are up and running
    gpio_put(LED_PIN, 1);

    // Spin forever, processing callbacks (subscriber + timer)
    rclc_executor_spin(&g_executor);

    return 0;
}
