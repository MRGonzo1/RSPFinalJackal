// Copyright 2022 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>

#include <control_msgs/msg/dynamic_joint_state.hpp>

#include <memory>

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node =
    std::make_shared<rclcpp::Node>("diff_drive_test_node");

  auto publisher = node->create_publisher<geometry_msgs::msg::Twist>(
    "/diff_drive_base_controller/cmd_vel_unstamped", 10);

  auto publisher1 = node->create_publisher<control_msgs::msg::DynamicJointState>(
    "/dynamic_joint_states", 10);

  // auto publisher1 = node->create_publisher<geometry_msgs::msg::Twist>(
  //   "/dynamic_joint_states", 10);

  RCLCPP_INFO(node->get_logger(), "node created");

  geometry_msgs::msg::Twist command;
  
  command.linear.x = 0.02;
  command.linear.y = 0.0;
  command.linear.z = 0.0;

  command.angular.x = 0.0;
  command.angular.y = 0.0;
  command.angular.z = 0.0;

  // control_msgs::msg::DynamicJointState command1;
  // std::cout << "0" << std::endl;
  // command1.joint_names[0] = "left_wheel_joint";
  // command1.joint_names[0] = "right_wheel_joint";
  // std::cout << "1" << std::endl;
  // command1.interface_values[0].interface_names[0] = "position";
  // command1.interface_values[0].values[0] = 0.0;
  // command1.interface_values[0].interface_names[1] = "velocity";
  // command1.interface_values[0].values[1] = -0.02;
  // std::cout << "2" << std::endl;
  // command1.interface_values[1].interface_names[0] = "position";
  // command1.interface_values[1].values[0] = 0.0;
  // command1.interface_values[1].interface_names[1] = "velocity";
  // command1.interface_values[1].values[1] = -0.02;

  // std::cout << "3" << std::endl;


  while (1) {
    publisher->publish(command);
    // publisher1->publish(command1);
    std::this_thread::sleep_for(50ms);
    rclcpp::spin_some(node);
  }
  rclcpp::shutdown();

  return 0;
}
