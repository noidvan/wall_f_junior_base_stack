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

#ifndef TELEOP_PANEL_H
#define TELEOP_PANEL_H

# include <rclcpp/rclcpp.hpp>
# include <rviz_common/panel.hpp>

#include <stdio.h>

#include <QPainter>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTimer>
#include <QPushButton>
#include <QCheckBox>
#include <QFileDialog>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int8.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <rclcpp/time.hpp>

#include "wall_f_junior_base_station/drive_widget.h"

class QLineEdit;

namespace wall_f_junior_base_station
{

class DriveWidget;

class ControlPanel: public rviz_common::Panel
{
  Q_OBJECT
public:
  ControlPanel( QWidget* parent = 0 );

public Q_SLOTS:
  void setVel( float linear_velocity_, float angular_velocity_, bool mouse_pressed_ );

protected Q_SLOTS:
  // Arm buttons
  void press_rear_arm();
  void press_front_arm();

  // Joint control buttons: pressed and released
  void press_joint_up();
  void release_joint_up();
  void press_joint_down();
  void release_joint_down();

  // Soil sensor control buttons: pressed and released
  void press_soil_up();
  void release_soil_up();
  void press_soil_down();
  void release_soil_down();

  void sendVel();

protected:
  DriveWidget* drive_widget_;

  // ROS2 publishers and node
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr velocity_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr rear_driver_motors_arm_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr front_driver_motors_arm_pub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr joint_stepper_motors_pub;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr soil_stepper_motorspub;
  rclcpp::Node::SharedPtr node_;

  // UI buttons
  QPushButton *rear_arm_button_;
  QPushButton *front_arm_button_;
  QPushButton *joint_up_button_;
  QPushButton *joint_down_button_;
  QPushButton *soil_up_button_;
  QPushButton *soil_down_button_;

  // Internal state flags
  bool rear_armed;
  bool front_armed;

  float linear_velocity_;
  float angular_velocity_;
  bool mouse_pressed_;
  bool mouse_pressed_sent_;
};

} // namespace wall_f_junior_base_station

#endif // TELEOP_PANEL_H
