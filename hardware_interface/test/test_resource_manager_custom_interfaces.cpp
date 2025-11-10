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

#include "test_resource_manager.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/resource_manager.hpp"
#include "rclcpp/node.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

TEST_F(TestResourceManager, custom_interfaces_validation)
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
        <sensor name="sensor1">
          <state_interface name="state1" data_type="uint64"/>
        </sensor>
      </ros2_control>
    </robot>
    )";

  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("test_resource_manager");
  hardware_interface::ResourceManager rm(
    urdf, node->get_clock(), node->get_logger(), true /*activate_all*/);

  EXPECT_EQ(rm.get_command_interface_data_type("joint1/command1"), "bool");
  EXPECT_EQ(rm.get_command_interface_data_type("joint1/command2"), "int32");
  EXPECT_EQ(rm.get_state_interface_data_type("joint1/state1"), "bool");
  EXPECT_EQ(rm.get_state_interface_data_type("joint1/state2"), "int32");
  EXPECT_EQ(rm.get_state_interface_data_type("sensor1/state1"), "uint64");
}
