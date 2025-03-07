/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/utils/frame_conversion.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <vector>

using namespace px4_ros2::literals;  // NOLINT

class DepthSeekerMode : public px4_ros2::ModeBase
{
public:
  explicit DepthSeekerMode(rclcpp::Node & node)
  : ModeBase(node, Settings{getName(node), false, static_cast<px4_ros2::ModeBase::ModeID>(getNavigationState(node))}),
    _inv_depth_bins(numDepthBins, 0.0f),
    _yaw_rate(0.0f),
    _forward_speed(0.0f),
    _status(false)
  {
    _inv_depth_bins_sub = node.create_subscription<std_msgs::msg::Float32MultiArray>(
      "~/avg_inv_depth", rclcpp::QoS(1).best_effort(),
      [this](std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        _inv_depth_bins = msg->data;
        computeYawRate();
      });

    node.declare_parameter<double>("max_yaw_rate", 0.5);
    _max_yaw_rate = node.get_parameter("max_yaw_rate").as_double();

    _yaw_rate_pub = node.create_publisher<std_msgs::msg::Float32>("~/yaw_rate", 10);
    _forward_speed_pub = node.create_publisher<std_msgs::msg::Float32>("~/forward_speed", 10);
    _status_pub = node.create_publisher<std_msgs::msg::Bool>("~/status", 10);

    _trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
    _vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
  }

  void onActivate() override
  {
    _state = State::Manoeuvre;
    _start_position_set = false;
    _status = true;
  }

  void onDeactivate() override { _status = false; }

  void updateSetpoint(float dt_s) override
  {
    if (!_start_position_set) {
      _start_position_m = _vehicle_local_position->positionNed();
      _start_position_set = true;
    }

    switch (_state) {
      case State::Manoeuvre: {
        // calc vertical speed based on offset from starting height
        // simple proportional control
        float vertical_speed =
          0.5f * (_start_position_m(2) - _vehicle_local_position->positionNed()(2));

        // update setpoint
        // trajectory: velocity and yaw rate in ned
        const Eigen::Vector3f target_velocity_m_s = px4_ros2::yawBodyToWorld(
          _vehicle_local_position->heading(), Eigen::Vector3f{forwardSpeed, 0.f, vertical_speed});
        _trajectory_setpoint->update(target_velocity_m_s, {}, {}, _yaw_rate);
      } break;
    }
  }

private:
  std::string getName(rclcpp::Node & node)
  {
    node.declare_parameter<std::string>("mode_name", "Depth Seeker");
    return node.get_parameter("mode_name").as_string();
  }

  static int getNavigationState(rclcpp::Node & node)
  {
    node.declare_parameter<std::string>("navigation_state", "ext1");
    std::string nav_state_str = node.get_parameter("navigation_state").as_string();

    // just allowing a couple of external modes for now
    if (nav_state_str == "ext1") {
      return px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_EXTERNAL1;
    } else if (nav_state_str == "ext2") {
      return px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_EXTERNAL2;
    } else if (nav_state_str == "ext3") {
      return px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_EXTERNAL3;
    } else {
      RCLCPP_WARN(
        node.get_logger(), "Unknown navigation state: %s, defaulting to EXTERNAL1",
        nav_state_str.c_str());
      return px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_EXTERNAL1;
    }
  }

  void computeYawRate()
  {
    // avg of center and side bins
    float center_idx = (numDepthBins - 1) / 2;
    // calculate yaw rate command
    float yaw_rate = 0;
    // only speed if steering
    float forward_speed = 0.0f;

    // avoid part
    float yaw_rate_avoid = 0;
    for (int i = 0; i < numDepthBins; i++) {
      float inv_depth_contrib = (center_idx - i) * std::exp(-constantObst / _inv_depth_bins[i]);
      float offset_contrib = std::exp(-std::pow(i - center_idx, 2) / (2 * sigma * sigma));
      yaw_rate_avoid += lambdaAvoid * inv_depth_contrib * offset_contrib;
    }

    // goal part
    // if multiple equal, chooses first
    int min_inv_depth_index = std::distance(
      _inv_depth_bins.begin(), std::min_element(_inv_depth_bins.begin(), _inv_depth_bins.end()));
    float yaw_rate_goal = lambdaGoal * (min_inv_depth_index - center_idx);

    // total
    yaw_rate = yaw_rate_avoid + yaw_rate_goal;
    forward_speed = forwardSpeed;

    // publish raw yaw rate
    std_msgs::msg::Float32 yaw_rate_msg;
    yaw_rate_msg.data = yaw_rate;
    _yaw_rate_pub->publish(yaw_rate_msg);

    // publish forward speed
    std_msgs::msg::Float32 forward_speed_msg;
    forward_speed_msg.data = forward_speed;
    _forward_speed_pub->publish(forward_speed_msg);

    // publish status
    std_msgs::msg::Bool status_msg;
    status_msg.data = _status;
    _status_pub->publish(status_msg);

    // limit yaw rate for control
    yaw_rate = std::max(-_max_yaw_rate, std::min(_max_yaw_rate, yaw_rate));

    // set computed yaw rate and speed to member variables
    _yaw_rate = yaw_rate;
    _forward_speed = forward_speed;
  }

  // TODO: make parameters
  static constexpr float forwardSpeed = 0.5f;  // forward speed during manoeuvre mode
  static constexpr int numDepthBins = 8;       // number of bins to bin the depth image to
  static constexpr float lambdaAvoid = 1.0f;   // original: 10; higher = smaller depths repulse more
  static constexpr float lambdaGoal = 0.2f;    // origina: 2; higher = larger depths attract more
  static constexpr float constantObst = 0.5f;  // scaling from depth of a bin to yaw rate
  static constexpr float sigma = 12.f;  // scaling from offset from center of a bin to yaw rate

  enum class State {
    Manoeuvre = 0,
  } _state;

  // NED earth-fixed frame
  Eigen::Vector3f _start_position_m;
  bool _start_position_set{false};

  std::vector<float> _inv_depth_bins;
  float _yaw_rate;
  float _max_yaw_rate;
  float _forward_speed;
  bool _status;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr _inv_depth_bins_sub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _yaw_rate_pub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _forward_speed_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _status_pub;

  std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;
  std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
};
