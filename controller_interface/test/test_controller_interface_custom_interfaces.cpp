// Copyright 2024 Stogl Robotics Consulting UG (haftungsbeschränkt)
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

#include "test_controller_interface.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "rclcpp/node.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

TEST_F(TestControllerInterface, custom_interfaces_validation)
{
  std::string urdf =
    R"(
    <robot name="robot" xmlns="http://www.ros.org">
      <ros2_control name="test_robot_with_custom_interfaces" type="system">
        <hardware>
          <plugin>test_hardware</plugin>
        </hardware>
        <joint name="joint1">
          <command_interface name="command1" data_type="bool"/>
          <command_interface name="command2" data_type="int32"/>
          <state_interface name="state1" data_type="bool"/>
          <state_interface name="state2" data_type="int32"/>
        </joint>
      </ros2_control>
    </robot>
    )";

  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("test_controller_interface");
  hardware_interface::ResourceManager rm(
    urdf, node->get_clock(), node->get_logger(), true /*activate_all*/);

  auto controller = std::make_shared<TestController>();
  controller->get_node()->set_parameter(
    "command_interfaces", std::vector<std::string>{"joint1/command1", "joint1/command2"});
  controller->get_node()->set_parameter(
    "state_interfaces", std::vector<std::string>{"joint1/state1", "joint1/state2"});

  controller->assign_interfaces(
    rm.claim_command_interfaces(
      controller->get_node()->get_parameter("command_interfaces").as_string_array()),
    rm.claim_state_interfaces(
      controller->get_node()->get_parameter("state_interfaces").as_string_array()));

  EXPECT_EQ(controller->command_interface_types_["joint1/command1"], "bool");
  EXPECT_EQ(controller->command_interface_types_["joint1/command2"], "int32");
  EXPECT_EQ(controller->state_interface_types_["joint1/state1"], "bool");
  EXPECT_EQ(controller->state_interface_types_["joint1/state2"], "int32");
}
