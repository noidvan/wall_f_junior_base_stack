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

#include "wall_f_junior_rpi/command_mixer.hpp"

CommandMixer::CommandMixer() : Node("command_mixer")
{
  // Create a subscription to the "/joy" topic with a queue size of 10.
  joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "/joy", 10,
    std::bind(&CommandMixer::joy_callback, this, std::placeholders::_1));

  // Create publishers for left and right motor setpoints.
  motor_left_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/motor_left_setpoint", 10);
  motor_right_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/motor_right_setpoint", 10);
}

float CommandMixer::clamp(float value, float min_val, float max_val)
{
  if (value < min_val) {
    return min_val;
  } else if (value > max_val) {
    return max_val;
  }
  return value;
}

void CommandMixer::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  // Ensure the message has at least 2 axes.
  if (msg->axes.size() < 2) {
    RCLCPP_WARN(this->get_logger(), "Received Joy message with insufficient axes.");
    return;
  }

  // Extract joystick commands:
  // - axes[0]: command yaw (cy)
  // - axes[1]: command throttle (ct)
  float cy = msg->axes[0];
  float ct = msg->axes[1];

  // Apply the thrust mixing matrix:
  // ml = -cy + ct
  // mr =  cy + ct
  float ml = (-1.0f * cy) + (1.0f * ct);
  float mr = (1.0f * cy) + (1.0f * ct);

  // Clamp the motor commands to the range [-1, 1]
  ml = clamp(ml, -1.0f, 1.0f);
  mr = clamp(mr, -1.0f, 1.0f);

  // Create and publish the left motor command.
  auto left_msg = std_msgs::msg::Float32();
  left_msg.data = ml;
  motor_left_publisher_->publish(left_msg);

  // Create and publish the right motor command.
  auto right_msg = std_msgs::msg::Float32();
  right_msg.data = mr;
  motor_right_publisher_->publish(right_msg);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CommandMixer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

