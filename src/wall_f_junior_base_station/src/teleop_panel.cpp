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

#include "wall_f_junior_base_station/teleop_panel.h"

namespace wall_f_junior_base_station
{

ControlPanel::ControlPanel( QWidget* parent )
  : rviz_common::Panel( parent )
  , linear_velocity_( 0 )
  , angular_velocity_( 0 )
  , mouse_pressed_( false )
  , mouse_pressed_sent_( false )
  , rear_armed(false)
  , front_armed(false)
{
  node_ = rclcpp::Node::make_shared("teleop_panel_node");

  // Initialize all publishers.
  velocity_publisher_ = node_->create_publisher<sensor_msgs::msg::Joy>("/joy", 5);
  rear_driver_motors_arm_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/rear_driver_motors_arm", 5);
  front_driver_motors_arm_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/front_driver_motors_arm", 5);
  joint_stepper_motors_pub = node_->create_publisher<std_msgs::msg::Int8>("/joint_stepper_motors", 5);
  soil_stepper_motorspub = node_->create_publisher<std_msgs::msg::Int8>("/soil_stepper_motors", 5);

  // Build UI layout.
  QVBoxLayout* layout = new QVBoxLayout;

  // Arm buttons (full width)
  rear_arm_button_ = new QPushButton( "Arm Rear Motors", this );
  layout->addWidget( rear_arm_button_ );
  
  front_arm_button_ = new QPushButton( "Arm Front Motors", this );
  layout->addWidget( front_arm_button_ );

  // Joint control buttons (half width on same line)
  QHBoxLayout* jointLayout = new QHBoxLayout;
  joint_up_button_ = new QPushButton( "Joint Up", this );
  joint_down_button_ = new QPushButton( "Joint Down", this );
  jointLayout->addWidget( joint_down_button_ );
  jointLayout->addWidget( joint_up_button_ );
  layout->addLayout( jointLayout );

  // Soil sensor control buttons (half width on same line)
  QHBoxLayout* soilLayout = new QHBoxLayout;
  soil_up_button_ = new QPushButton( "Soil Sensor Up", this );
  soil_down_button_ = new QPushButton( "Soil Sensor Down", this );
  soilLayout->addWidget( soil_down_button_ );
  soilLayout->addWidget( soil_up_button_ );
  layout->addLayout( soilLayout );

  // Add drive widget.
  drive_widget_ = new DriveWidget;
  layout->addWidget( drive_widget_ );

  setLayout( layout );

  // Timer for sending velocity messages.
  QTimer* output_timer = new QTimer( this );
  connect(output_timer, &QTimer::timeout, this, &ControlPanel::sendVel);
  output_timer->start(100);
  
  // Connect arm buttons to their slots.
  connect(rear_arm_button_, &QPushButton::clicked, this, &ControlPanel::press_rear_arm);
  connect(front_arm_button_, &QPushButton::clicked, this, &ControlPanel::press_front_arm);
  
  // Connect joint control buttons for press and release events.
  connect(joint_up_button_, &QPushButton::pressed, this, &ControlPanel::press_joint_up);
  connect(joint_up_button_, &QPushButton::released, this, &ControlPanel::release_joint_up);
  connect(joint_down_button_, &QPushButton::pressed, this, &ControlPanel::press_joint_down);
  connect(joint_down_button_, &QPushButton::released, this, &ControlPanel::release_joint_down);
  
  // Connect soil sensor control buttons for press and release events.
  connect(soil_up_button_, &QPushButton::pressed, this, &ControlPanel::press_soil_up);
  connect(soil_up_button_, &QPushButton::released, this, &ControlPanel::release_soil_up);
  connect(soil_down_button_, &QPushButton::pressed, this, &ControlPanel::press_soil_down);
  connect(soil_down_button_, &QPushButton::released, this, &ControlPanel::release_soil_down);
  
  // Connect drive widget velocity signal.
  connect(drive_widget_, &DriveWidget::outputVelocity, this, &ControlPanel::setVel);
  
}

void ControlPanel::press_rear_arm()
{
  std_msgs::msg::Bool msg;
  if (!rear_armed)
  {
    rear_armed = true;
    msg.data = true;
    rear_arm_button_->setText("Disarm Rear Motors");
  }
  else
  {
    rear_armed = false;
    msg.data = false;
    rear_arm_button_->setText("Arm Rear Motors");
  }
  rear_driver_motors_arm_pub_->publish(msg);
}

void ControlPanel::press_front_arm()
{
  std_msgs::msg::Bool msg;
  if (!front_armed)
  {
    front_armed = true;
    msg.data = true;
    front_arm_button_->setText("Disarm Front Motors");
  }
  else
  {
    front_armed = false;
    msg.data = false;
    front_arm_button_->setText("Arm Front Motors");
  }
  front_driver_motors_arm_pub_->publish(msg);
}

void ControlPanel::press_joint_up()
{
  std_msgs::msg::Int8 msg;
  msg.data = 1;
  joint_stepper_motors_pub->publish(msg);
}

void ControlPanel::release_joint_up()
{
  std_msgs::msg::Int8 msg;
  msg.data = 0;
  joint_stepper_motors_pub->publish(msg);
}

void ControlPanel::press_joint_down()
{
  std_msgs::msg::Int8 msg;
  msg.data = -1;
  joint_stepper_motors_pub->publish(msg);
}

void ControlPanel::release_joint_down()
{
  std_msgs::msg::Int8 msg;
  msg.data = 0;
  joint_stepper_motors_pub->publish(msg);
}

void ControlPanel::press_soil_up()
{
  std_msgs::msg::Int8 msg;
  msg.data = 1;
  soil_stepper_motorspub->publish(msg);
}

void ControlPanel::release_soil_up()
{
  std_msgs::msg::Int8 msg;
  msg.data = 0;
  soil_stepper_motorspub->publish(msg);
}

void ControlPanel::press_soil_down()
{
  std_msgs::msg::Int8 msg;
  msg.data = -1;
  soil_stepper_motorspub->publish(msg);
}

void ControlPanel::release_soil_down()
{
  std_msgs::msg::Int8 msg;
  msg.data = 0;
  soil_stepper_motorspub->publish(msg);
}

void ControlPanel::setVel(float lin, float ang, bool pre)
{
  linear_velocity_ = lin;
  angular_velocity_ = ang;
  mouse_pressed_ = pre;
}

void ControlPanel::sendVel()
{
  if (rclcpp::ok() && velocity_publisher_->get_subscription_count() > 0 && (mouse_pressed_ || mouse_pressed_sent_))
  {
    sensor_msgs::msg::Joy joy;

    joy.axes.push_back( angular_velocity_ );
    joy.axes.push_back( linear_velocity_ );

    joy.header.stamp = node_->now();
    joy.header.frame_id = "teleop_panel";
    velocity_publisher_->publish(joy);

    mouse_pressed_sent_ = mouse_pressed_;
  }
}

} // end namespace wall_f_junior_base_station

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(wall_f_junior_base_station::ControlPanel, rviz_common::Panel)
