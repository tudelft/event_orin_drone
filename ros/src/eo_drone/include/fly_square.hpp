/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/goto.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/utils/frame_conversion.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <rclcpp/rclcpp.hpp>

static const std::string kName = "Fly Square";  // goes to mode id 25, so ext3

using namespace px4_ros2::literals;  // NOLINT

class FlySquareMode : public px4_ros2::ModeBase
{
public:
  explicit FlySquareMode(rclcpp::Node & node)
  : ModeBase(node, kName)
  // : ModeBase(node, Settings{kName, false, px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_EXTERNAL1})  // why doesn't this work?
  // : ModeBase(node, Settings{kName, false, px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_MISSION})  // why DOES this work?
  // : ModeBase(node, Settings{kName, false, px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_EXTERNAL3})  // why DOES this work if it's also being registered as 3?
  {
    _goto_setpoint = std::make_shared<px4_ros2::GotoSetpointType>(*this);
    _vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
  }

  void onActivate() override
  {
    _state = State::SettlingAtStart;
    _current_position_set = false;
    _current_heading_set = false;
  }

  void onDeactivate() override {}

  void updateSetpoint(float dt_s) override
  {
    if (!_current_position_set) {
      _current_position_m = _vehicle_local_position->positionNed();
      _current_position_set = true;
    }
    if (!_current_heading_set) {
      _current_heading_rad = _vehicle_local_position->heading();
      _current_heading_set = true;
    }

    switch (_state) {
      case State::SettlingAtStart: {
        // just settling at the starting vehicle position
        _goto_setpoint->update(_current_position_m, _current_heading_rad);
        if (positionReached(_current_position_m) && headingReached(_current_heading_rad)) {
          _state = State::GoingForward;
        }
      } break;

      case State::GoingForward: {
        // go forward facing in the direction of travel
        const Eigen::Vector3f target_position_m =
          _current_position_m +
          px4_ros2::yawBodyToWorld(_current_heading_rad, Eigen::Vector3f{kSquareSide, 0.f, 0.f});

        // if we deviate we want to rotate to the heading of the target
        // const Eigen::Vector2f vehicle_to_target_xy = target_position_m.head(2)
        // -
        //   _vehicle_local_position->positionNed().head(2);
        // const float heading_target_rad = atan2f(vehicle_to_target_xy(1),
        // vehicle_to_target_xy(0));

        // if (vehicle_to_target_xy.norm() < 0.1f) {
        //   // stop caring about heading along the flight direction (the
        //   arctangent becomes undefined)
        //   // instead, to make the square a square, care about aligning with
        //   starting heading _goto_setpoint->update(target_position_m,
        //   _current_heading_rad);
        // } else {
        //   _goto_setpoint->update(target_position_m, heading_target_rad);
        // }
        _goto_setpoint->update(target_position_m, _current_heading_rad);

        // we care about heading because to make the square a square
        // it's good if we end facing the same way we started
        if (positionReached(target_position_m) && headingReached(_current_heading_rad)) {
          _current_position_m = target_position_m;
          _state = State::Rotate90Right;
        }
      } break;

      case State::Rotate90Right: {
        // rotate 90 degrees to the right
        const float target_heading_rad = px4_ros2::wrapPi(_current_heading_rad + 90.0_deg);
        _goto_setpoint->update(_current_position_m, target_heading_rad);

        // also make sure we stay in position
        if (positionReached(_current_position_m) && headingReached(target_heading_rad)) {
          _current_heading_rad = target_heading_rad;
          _state = State::GoingForward;
        }
      } break;
    }
  }

private:
  static constexpr float kSquareSide = 2.f;  // [m]

  enum class State {
    SettlingAtStart = 0,
    GoingForward,
    Rotate90Right,
  } _state;

  // NED earth-fixed frame
  Eigen::Vector3f _current_position_m;
  bool _current_position_set{false};
  float _current_heading_rad{0.f};
  bool _current_heading_set{false};

  std::shared_ptr<px4_ros2::GotoSetpointType> _goto_setpoint;
  std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;

  bool positionReached(const Eigen::Vector3f & target_position_m) const
  {
    static constexpr float kPositionErrorThreshold = 0.5f;  // [m]
    static constexpr float kVelocityErrorThreshold = 0.3f;  // [m/s]
    const Eigen::Vector3f position_error_m =
      target_position_m - _vehicle_local_position->positionNed();
    return (position_error_m.norm() < kPositionErrorThreshold) &&
           (_vehicle_local_position->velocityNed().norm() < kVelocityErrorThreshold);
  }

  bool headingReached(float target_heading_rad) const
  {
    static constexpr float kHeadingErrorThreshold = 7.0_deg;
    const float heading_error_wrapped =
      px4_ros2::wrapPi(target_heading_rad - _vehicle_local_position->heading());
    return fabsf(heading_error_wrapped) < kHeadingErrorThreshold;
  }
};
