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
#include <sensor_msgs/msg/imu.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "pico_uart_transports.h"

static const uint8_t LEFT_STEP_PIN = 6;
static const uint8_t LEFT_DIR_PIN  = 7;
static const uint8_t RIGHT_STEP_PIN = 8;
static const uint8_t RIGHT_DIR_PIN  = 9;
static const uint8_t LED_PIN  = 25;

static const int     ROS_TIMEOUT_MS = 1000;
static const uint8_t ROS_ATTEMPTS   = 120;
static const int     ROS_NUM_HANDLES = 3;

static const char*   NAMESPACE        = "";
static const char*   NODE_NAME        = "front_agent";
static const char*   STEPPER_TOPIC    = "joint_stepper_motors"; // std_msgs/Int8

static const uint32_t STEP_TIMER_NS = 4000;  // Timer period in ms; sets stepping speed
static const uint32_t STEP_PULSE_NS = 2000;   // Pulse width for step pin (microseconds)

static volatile int8_t direction_state = 0; // -1, 0, +1

static i2c_inst_t* MPU6050_PORT = i2c1;
static const int MPU6050_SDA_PIN = 2;
static const int MPU6050_SCL_PIN = 3;
static const int MPU6050_ADDR = 0x68;

// Conversion factors (assuming default ±2g & ±250 deg/s ranges)
static const float ACCEL_SENS       = 16384.0f;  // LSB/g
static const float GYRO_SENS_DEG_S  = 131.0f;    // LSB/deg/s
static const float G_TO_M_S2        = 9.81f;
static const float DEG_TO_RAD       = (float)M_PI / 180.0f;

static const char*   IMU_TOPIC           = "imu";

static const uint32_t IMU_PUBLISH_PERIOD_MS = 10;

static rcl_node_t      node;
static rcl_allocator_t allocator;
static rclc_support_t  support;
static rclc_executor_t executor;
static std_msgs__msg__Int8 direction_msg; 
sensor_msgs__msg__Imu imu_msg;

static rcl_publisher_t pub_imu;
static rcl_timer_t     imu_timer;
static rcl_subscription_t sub_direction;
static rcl_timer_t     stepper_timer;

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
        gpio_put(LEFT_DIR_PIN, dir_pin_level);
        gpio_put(RIGHT_DIR_PIN, !dir_pin_level);

        // Generate a single rising->falling pulse on STEP pin
        //  We use a short busy-wait for pulse width
        gpio_put(LEFT_STEP_PIN, true);
        gpio_put(RIGHT_STEP_PIN, true);
        sleep_us(STEP_PULSE_NS);    // Wait high
        gpio_put(LEFT_STEP_PIN, false);
        gpio_put(RIGHT_STEP_PIN, false);
        // The motor sees this as one step; frequency is determined by how often
        // step_timer_callback gets called (STEP_TIMER_MS).
    }
}

static inline void mpu6050_write_reg(uint8_t reg, uint8_t value) {
  uint8_t buf[2];
  buf[0] = reg;
  buf[1] = value;
  i2c_write_blocking(MPU6050_PORT, MPU6050_ADDR, buf, 2, false);
}

static void mpu6050_reset() {
  // Two byte reset. First byte register, second byte data
  // There are a load more options to set up the device in different ways that could be added here
  uint8_t buf[] = {0x6B, 0x80};
  i2c_write_blocking(MPU6050_PORT, MPU6050_ADDR, buf, 2, false);
  sleep_ms(100); // Allow device to reset and stabilize

  // Clear sleep mode (0x6B register, 0x00 value)
  buf[1] = 0x00;  // Clear sleep mode by writing 0x00 to the 0x6B register
  i2c_write_blocking(MPU6050_PORT, MPU6050_ADDR, buf, 2, false); 
  sleep_ms(10); // Allow stabilization after waking up
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
  // For this particular device, we send the device the register we want to read
  // first, then subsequently read from the device. The register is auto incrementing
  // so we don't need to keep sending the register we want, just the first.

  uint8_t buffer[6];

  // Start reading acceleration registers from register 0x3B for 6 bytes
  uint8_t val = 0x3B;
  i2c_write_blocking(MPU6050_PORT, MPU6050_ADDR, &val, 1, true); // true to keep master control of bus
  i2c_read_blocking(MPU6050_PORT, MPU6050_ADDR, buffer, 6, false);

  for (int i = 0; i < 3; i++) {
      accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
  }

  // Now gyro data from reg 0x43 for 6 bytes
  // The register is auto incrementing on each read
  val = 0x43;
  i2c_write_blocking(MPU6050_PORT, MPU6050_ADDR, &val, 1, true);
  i2c_read_blocking(MPU6050_PORT, MPU6050_ADDR, buffer, 6, false);  // False - finished with bus

  for (int i = 0; i < 3; i++) {
      gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
  }

  // Now temperature from reg 0x41 for 2 bytes
  // The register is auto incrementing on each read
  val = 0x41;
  i2c_write_blocking(MPU6050_PORT, MPU6050_ADDR, &val, 1, true);
  i2c_read_blocking(MPU6050_PORT, MPU6050_ADDR, buffer, 2, false);  // False - finished with bus

  *temp = buffer[0] << 8 | buffer[1];
}

static void imu_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)timer;
  (void)last_call_time;

  // Read raw data
  int16_t accel_raw[3], gyro_raw[3], temp_raw;
  mpu6050_read_raw(accel_raw, gyro_raw, &temp_raw);

  // Convert to physical units

  // Acceleration in m/s^2
  // default ±2g => 1g = 16384 LSB. So raw / 16384 => g, then multiply by 9.81 => m/s^2
  float ax = (accel_raw[0] / ACCEL_SENS) * G_TO_M_S2;
  float ay = (accel_raw[1] / ACCEL_SENS) * G_TO_M_S2;
  float az = (accel_raw[2] / ACCEL_SENS) * G_TO_M_S2;

  // Gyro in rad/s
  // default ±250 deg/s => 1 deg/s = 131 LSB. raw / 131 => deg/s, multiply by pi/180 => rad/s
  float gx = (gyro_raw[0] / GYRO_SENS_DEG_S) * DEG_TO_RAD;
  float gy = (gyro_raw[1] / GYRO_SENS_DEG_S) * DEG_TO_RAD;
  float gz = (gyro_raw[2] / GYRO_SENS_DEG_S) * DEG_TO_RAD;

  // Fill the IMU message
  // 1) Header (we have no real time source, so just fill seq or leave as is)
  //    Typically you'd fill frame_id or attach a timestamp if you have one
  //    for demo, we ignore them
  imu_msg.header.stamp.sec  = 0;
  imu_msg.header.stamp.nanosec = 0;
  // "base_link" or "imu_link" might be typical
  // but for simplicity:
  memset(imu_msg.header.frame_id.data, 0, imu_msg.header.frame_id.capacity);
  snprintf(imu_msg.header.frame_id.data, imu_msg.header.frame_id.capacity, "imu_link");
  imu_msg.header.frame_id.size = strlen("imu_link");

  // 2) Orientation – we do not measure orientation with just an accelerometer & gyro.
  //    Some people put a placeholder (0,0,0,1) indicating no rotation from reference.
  imu_msg.orientation.x = 0.0;
  imu_msg.orientation.y = 0.0;
  imu_msg.orientation.z = 0.0;
  imu_msg.orientation.w = 1.0;

  // 3) Angular velocity
  imu_msg.angular_velocity.x = gx;
  imu_msg.angular_velocity.y = gy;
  imu_msg.angular_velocity.z = gz;

  // 4) Linear acceleration
  imu_msg.linear_acceleration.x = ax;
  imu_msg.linear_acceleration.y = ay;
  imu_msg.linear_acceleration.z = az;

  // We can set covariance arrays to 0 if unknown or if you'd like to specify them.
  // For demonstration, we set them to 0:
  for (int i = 0; i < 9; i++) {
      imu_msg.orientation_covariance[i] = 0.0;
      imu_msg.angular_velocity_covariance[i] = 0.0;
      imu_msg.linear_acceleration_covariance[i] = 0.0;
  }

  // Publish the IMU message
  rcl_ret_t rc = rcl_publish(&pub_imu, &imu_msg, NULL);
  (void)rc; // Typically check for RCL_RET_OK
}



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
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);

    // Set up STEP and DIR pins
    gpio_init(LEFT_STEP_PIN);
    gpio_set_dir(LEFT_STEP_PIN, GPIO_OUT);
    gpio_put(LEFT_STEP_PIN, false);

    gpio_init(LEFT_DIR_PIN);
    gpio_set_dir(LEFT_DIR_PIN, GPIO_OUT);
    gpio_put(LEFT_DIR_PIN, false);

    gpio_init(RIGHT_STEP_PIN);
    gpio_set_dir(RIGHT_STEP_PIN, GPIO_OUT);
    gpio_put(RIGHT_STEP_PIN, false);

    gpio_init(RIGHT_STEP_PIN);
    gpio_set_dir(RIGHT_STEP_PIN, GPIO_OUT);
    gpio_put(RIGHT_STEP_PIN, false);
    
    i2c_init(MPU6050_PORT, 400 * 1000);
    gpio_set_function(MPU6050_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(MPU6050_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(MPU6050_SDA_PIN);
    gpio_pull_up(MPU6050_SCL_PIN);
    bi_decl(bi_2pins_with_func(MPU6050_SDA_PIN,
                               MPU6050_SCL_PIN,
                               GPIO_FUNC_I2C));

    // Reset/Wake up MPU6050
    mpu6050_reset();

    // Attempt to ping the micro-ROS Agent
    rcl_ret_t ret = rmw_uros_ping_agent(ROS_TIMEOUT_MS, ROS_ATTEMPTS);
    if (ret != RCL_RET_OK) {
        // If we cannot contact the agent, return early
        return ret;
    }

    allocator = rcl_get_default_allocator();

    // Initialize micro-ROS support structures
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, NODE_NAME, NAMESPACE, &support);

    // Create subscriber for std_msgs/Int8
    const rosidl_message_type_support_t* int8_type_support =
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8);

    const rosidl_message_type_support_t* imu_msg_ts =
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu);

    rclc_subscription_init_default(
        &sub_direction,
        &node,
        int8_type_support,
        STEPPER_TOPIC
    );

    rclc_publisher_init_default(
        &pub_imu,
        &node,
        imu_msg_ts,
        IMU_TOPIC
    );

    imu_msg.header.frame_id.capacity = 64;
    imu_msg.header.frame_id.data = (char*) malloc(imu_msg.header.frame_id.capacity);
    imu_msg.header.frame_id.size = 0;

    // Create a timer for step pulses
    // The period sets how frequently we generate a STEP pulse (if direction != 0)
    rclc_timer_init_default2(
        &stepper_timer,
        &support,
        STEP_TIMER_NS,
        step_timer_callback,
        true
    );

    rclc_timer_init_default2(
        &imu_timer,
        &support,
        RCL_MS_TO_NS(IMU_PUBLISH_PERIOD_MS),
        imu_timer_callback,
        true
    );

    // Create an executor to handle subscription + timer
    rclc_executor_init(&executor, &support.context, ROS_NUM_HANDLES, &allocator);

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
    rclc_executor_add_timer(&executor, &imu_timer);

    // Light the LED to indicate we are up and running
    gpio_put(LED_PIN, 1);

    // Spin forever, processing callbacks (subscriber + timer)
    rclc_executor_spin(&executor);

    return 0;
}
