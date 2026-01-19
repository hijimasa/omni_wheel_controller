// Copyright 2020 PAL Robotics SL.
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

#include <gmock/gmock.h>
#include <memory>

#include "controller_manager/controller_manager.hpp"
#include "rclcpp/utilities.hpp"
//#include "ros2_control_test_assets/descriptions.hpp"
#include "descriptions.hpp"

// Check for ROS 2 Jazzy+ API changes
#include <rclcpp/version.h>
#if RCLCPP_VERSION_GTE(28, 0, 0)
#define OMNI_WHEEL_ROS2_JAZZY_OR_LATER 1
#else
#define OMNI_WHEEL_ROS2_JAZZY_OR_LATER 0
#endif

TEST(TestLoadOmniWheelController, load_controller)
{
  rclcpp::init(0, nullptr);

  std::shared_ptr<rclcpp::Executor> executor =
    std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

#if OMNI_WHEEL_ROS2_JAZZY_OR_LATER
  // In Jazzy+, ResourceManager constructor requires additional parameters
  auto node = std::make_shared<rclcpp::Node>("test_resource_manager_node");
  auto rm = std::make_unique<hardware_interface::ResourceManager>(
    ros2_control_test_assets::omni_wheel_robot_urdf,
    node->get_node_clock_interface(),
    node->get_node_logging_interface());
  controller_manager::ControllerManager cm(
    std::move(rm), executor, "test_controller_manager");
#else
  controller_manager::ControllerManager cm(
    std::make_unique<hardware_interface::ResourceManager>(ros2_control_test_assets::omni_wheel_robot_urdf),
    executor, "test_controller_manager");
#endif

  ASSERT_NE(
    cm.load_controller("test_omni_wheel_controller", "omni_wheel_controller/OmniWheelController"),
    nullptr);

  rclcpp::shutdown();
}
