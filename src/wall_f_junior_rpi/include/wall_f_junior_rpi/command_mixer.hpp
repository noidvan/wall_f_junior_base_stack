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

#ifndef COMMAND_MIXER_HPP_
#define COMMAND_MIXER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32.hpp"

class CommandMixer : public rclcpp::Node
{
public:
  CommandMixer();

private:
  // Callback function for processing incoming joystick messages.
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

  // Helper function to clamp a value between a minimum and maximum.
  float clamp(float value, float min_val, float max_val);

  // Subscriber for the "/joy" topic.
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;

  // Publishers for motor setpoints.
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr motor_left_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr motor_right_publisher_;
};

#endif  // COMMAND_MIXER_HPP_
