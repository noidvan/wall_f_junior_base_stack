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
#include "hardware/i2c.h"
#include "pico/binary_info.h"

// micro-ROS / ROS 2 includes
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/imu.h>

// If using the default micro-ROS Pico UART transport:
#include "pico_uart_transports.h"

// ------------------------------------------------------------------------------------
// MPU6050 definitions
// ------------------------------------------------------------------------------------
static const int MPU6050_ADDR = 0x68;

// Conversion factors (assuming default ±2g & ±250 deg/s ranges)
static const float ACCEL_SENS       = 16384.0f;  // LSB/g
static const float GYRO_SENS_DEG_S  = 131.0f;    // LSB/deg/s
static const float G_TO_M_S2        = 9.81f;
static const float DEG_TO_RAD       = (float)M_PI / 180.0f;

// ------------------------------------------------------------------------------------
// micro-ROS configuration
// ------------------------------------------------------------------------------------
static const uint8_t ROS_ATTEMPTS       = 120;
static const int     ROS_TIMEOUT_MS     = 1000;
static const int     ROS_EXECUTOR_HANDLES = 1;

// We want to publish at 100 Hz => every 10 ms
static const uint32_t IMU_PUBLISH_PERIOD_MS = 10;

// Node & topic names
static const char*   ROS_NAMESPACE       = "";
static const char*   ROS_NODE_NAME       = "imu_node";
static const char*   IMU_TOPIC           = "imu";

// ------------------------------------------------------------------------------------
// Global micro-ROS entities
// ------------------------------------------------------------------------------------
rcl_node_t      g_node;
rcl_publisher_t g_pub_imu;
rcl_timer_t     g_timer;
rcl_allocator_t g_allocator;
rclc_support_t  g_support;
rclc_executor_t g_executor;

// A global IMU message buffer needed for the publisher
sensor_msgs__msg__Imu g_imu_msg;

// ------------------------------------------------------------------------------------
// Helper: Write a register on MPU6050
// ------------------------------------------------------------------------------------
static inline void mpu6050_write_reg(uint8_t reg, uint8_t value) {
    uint8_t buf[2];
    buf[0] = reg;
    buf[1] = value;
    i2c_write_blocking(i2c_default, MPU6050_ADDR, buf, 2, false);
}

// ------------------------------------------------------------------------------------
// Reset / Wake the MPU6050
// ------------------------------------------------------------------------------------
static void mpu6050_reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x80};
    i2c_write_blocking(i2c_default, MPU6050_ADDR, buf, 2, false);
    sleep_ms(100); // Allow device to reset and stabilize

    // Clear sleep mode (0x6B register, 0x00 value)
    buf[1] = 0x00;  // Clear sleep mode by writing 0x00 to the 0x6B register
    i2c_write_blocking(i2c_default, MPU6050_ADDR, buf, 2, false); 
    sleep_ms(10); // Allow stabilization after waking up
}

// ------------------------------------------------------------------------------------
// Read raw acceleration (X,Y,Z), raw gyro (X,Y,Z), and temperature from the MPU6050
// ------------------------------------------------------------------------------------
static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, MPU6050_ADDR, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_default, MPU6050_ADDR, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(i2c_default, MPU6050_ADDR, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU6050_ADDR, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = 0x41;
    i2c_write_blocking(i2c_default, MPU6050_ADDR, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU6050_ADDR, buffer, 2, false);  // False - finished with bus

    *temp = buffer[0] << 8 | buffer[1];
}

// ------------------------------------------------------------------------------------
// Timer callback: Read MPU6050, convert to SI units, publish sensor_msgs/Imu
// ------------------------------------------------------------------------------------
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
    g_imu_msg.header.stamp.sec  = 0;
    g_imu_msg.header.stamp.nanosec = 0;
    // "base_link" or "imu_link" might be typical
    // but for simplicity:
    memset(g_imu_msg.header.frame_id.data, 0, g_imu_msg.header.frame_id.capacity);
    snprintf(g_imu_msg.header.frame_id.data, g_imu_msg.header.frame_id.capacity, "imu_link");
    g_imu_msg.header.frame_id.size = strlen("imu_link");

    // 2) Orientation – we do not measure orientation with just an accelerometer & gyro.
    //    Some people put a placeholder (0,0,0,1) indicating no rotation from reference.
    g_imu_msg.orientation.x = 0.0;
    g_imu_msg.orientation.y = 0.0;
    g_imu_msg.orientation.z = 0.0;
    g_imu_msg.orientation.w = 1.0;

    // 3) Angular velocity
    g_imu_msg.angular_velocity.x = gx;
    g_imu_msg.angular_velocity.y = gy;
    g_imu_msg.angular_velocity.z = gz;

    // 4) Linear acceleration
    g_imu_msg.linear_acceleration.x = ax;
    g_imu_msg.linear_acceleration.y = ay;
    g_imu_msg.linear_acceleration.z = az;

    // We can set covariance arrays to 0 if unknown or if you'd like to specify them.
    // For demonstration, we set them to 0:
    for (int i = 0; i < 9; i++) {
        g_imu_msg.orientation_covariance[i] = 0.0;
        g_imu_msg.angular_velocity_covariance[i] = 0.0;
        g_imu_msg.linear_acceleration_covariance[i] = 0.0;
    }

    // Publish the IMU message
    rcl_ret_t rc = rcl_publish(&g_pub_imu, &g_imu_msg, NULL);
    (void)rc; // Typically check for RCL_RET_OK
}

// ------------------------------------------------------------------------------------
// main()
// ------------------------------------------------------------------------------------
int main() {
    // Initialize stdio (for debugging, optional)
    stdio_init_all();

    // Initialize I2C at 400 kHz
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN,
                               PICO_DEFAULT_I2C_SCL_PIN,
                               GPIO_FUNC_I2C));

    // Reset/Wake up MPU6050
    mpu6050_reset();

    // -------------------------------------------------------------------
    // micro-ROS Transport Setup
    //  - For the typical UART-based approach:
    // -------------------------------------------------------------------
    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );

    // Attempt to ping micro-ROS Agent
    rcl_ret_t ret = rmw_uros_ping_agent(ROS_TIMEOUT_MS, ROS_ATTEMPTS);
    if (ret != RCL_RET_OK) {
        // If no agent found, we just give up here
        printf("No micro-ROS agent found. Exiting.\n");
        return ret;
    }

    // -------------------------------------------------------------------
    // micro-ROS Setup
    // -------------------------------------------------------------------
    g_allocator = rcl_get_default_allocator();

    // Create rclc_support
    rclc_support_init(&g_support, 0, NULL, &g_allocator);

    // Create node
    rclc_node_init_default(&g_node, ROS_NODE_NAME, ROS_NAMESPACE, &g_support);

    // Create IMU publisher
    const rosidl_message_type_support_t* imu_msg_ts =
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu);

    rclc_publisher_init_default(
        &g_pub_imu,
        &g_node,
        imu_msg_ts,
        IMU_TOPIC
    );

    // Initialize the IMU message's frame_id string so it's ready to hold data
    g_imu_msg.header.frame_id.capacity = 64;  // Just an example capacity
    g_imu_msg.header.frame_id.data = (char*) malloc(g_imu_msg.header.frame_id.capacity);
    g_imu_msg.header.frame_id.size = 0;       // We'll set it in the callback

    // Create timer at 100 Hz => 10 ms
    rclc_timer_init_default(
        &g_timer,
        &g_support,
        RCL_MS_TO_NS(IMU_PUBLISH_PERIOD_MS),
        imu_timer_callback
    );

    // Create executor
    rclc_executor_init(&g_executor, &g_support.context, ROS_EXECUTOR_HANDLES, &g_allocator);

    // Add timer callback
    rclc_executor_add_timer(&g_executor, &g_timer);

    // -------------------------------------------------------------------
    // Spin forever at 100 Hz
    // -------------------------------------------------------------------
    printf("Publishing IMU at 100 Hz on topic '%s'\n", IMU_TOPIC);
    rclc_executor_spin(&g_executor);

    // Normally never returns in embedded apps
    return 0;
}
