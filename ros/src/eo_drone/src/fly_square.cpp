/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <fly_square.hpp>
#include <px4_ros2/components/node_with_mode.hpp>

#include "rclcpp/rclcpp.hpp"

using FlySquareNode = px4_ros2::NodeWithMode<FlySquareMode>;

static const std::string kNodeName = "fly_square";
static const bool kEnableDebugOutput = true;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FlySquareNode>(kNodeName, kEnableDebugOutput));
  rclcpp::shutdown();
  return 0;
}
