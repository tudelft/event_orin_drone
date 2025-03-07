/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <depth_seeker.hpp>
#include <px4_ros2/components/node_with_mode.hpp>

#include "rclcpp/rclcpp.hpp"

using DepthSeekerNode = px4_ros2::NodeWithMode<DepthSeekerMode>;

static const std::string kNodeName = "depth_seeker";
static const bool kEnableDebugOutput = true;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthSeekerNode>(kNodeName, kEnableDebugOutput));
  rclcpp::shutdown();
  return 0;
}
