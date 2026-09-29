// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "ros2_fault_injection/assertions/twist_stopped_assertion.hpp"

#include <cmath>

namespace ros2_fault_injection::assertions
{
TwistStoppedAssertion::TwistStoppedAssertion(
  const AssertionConfig & config, const rclcpp::Time & start_time)
: config_(config), start_time_(start_time)
{
  result_.id = config.id;
  result_.type = config.type;
  result_.state = AssertionState::Pending;
  result_.message = "Waiting for fault " + config.fault_id + " to stop Twist commands on " +
    config.topic;
}

const std::string & TwistStoppedAssertion::topic() const
{
  return config_.topic;
}

void TwistStoppedAssertion::observe_fault_event(
  const msg::FaultEvent & event, const rclcpp::Time & stamp)
{
  if (result_.state != AssertionState::Pending || fault_active_) {
    return;
  }

  if (event.fault_id != config_.fault_id || event.state != "active") {
    return;
  }

  const double seconds_since_start = (stamp - start_time_).seconds();
  if (seconds_since_start < 0.0) {
    fail("Fault " + config_.fault_id + " activated before the assertion started");
    return;
  }

  if (seconds_since_start > config_.trigger_within.value()) {
    fail("Fault " + config_.fault_id + " activated after trigger_within expired");
    return;
  }

  fault_active_ = true;
  fault_activation_stamp_ = stamp;
}

void TwistStoppedAssertion::observe_message(
  const geometry_msgs::msg::Twist & message, const rclcpp::Time & stamp)
{
  if (result_.state != AssertionState::Pending) {
    return;
  }

  if (!fault_active_) {
    return;
  }

  if (!is_finite(message)) {
    fail("Received a non-finite Twist command on " + config_.topic + " after fault " +
      config_.fault_id + " activated");
    return;
  }

  if (!first_zero_stamp_ &&
    (stamp - fault_activation_stamp_.value()).seconds() > config_.within.value())
  {
    fail("No zero Twist command arrived on " + config_.topic + " within " +
      std::to_string(config_.within.value()) + " seconds after fault " + config_.fault_id);
    return;
  }

  if (!is_zero(message)) {
    if (first_zero_stamp_) {
      fail("Received a non-zero Twist command on " + config_.topic +
        " after the stop response began");
    }
    return;
  }

  if (!first_zero_stamp_) {
    first_zero_stamp_ = stamp;
  } else if ((stamp - last_zero_stamp_.value()).seconds() > config_.max_gap.value()) {
    fail("Zero Twist commands on " + config_.topic + " exceeded max_gap");
    return;
  }
  last_zero_stamp_ = stamp;
}

void TwistStoppedAssertion::update(const rclcpp::Time & now)
{
  if (result_.state != AssertionState::Pending) {
    return;
  }

  if (!fault_active_) {
    if ((now - start_time_).seconds() > config_.trigger_within.value()) {
      fail("Timed out waiting for fault " + config_.fault_id + " to activate");
    }
    return;
  }

  if (!first_zero_stamp_) {
    if ((now - fault_activation_stamp_.value()).seconds() > config_.within.value()) {
      fail("Fault " + config_.fault_id + " activated but no zero Twist command arrived on " +
        config_.topic + " before the deadline");
    }
    return;
  }

  const double gap_seconds = (now - last_zero_stamp_.value()).seconds();
  if (gap_seconds > config_.max_gap.value()) {
    fail("Zero Twist commands on " + config_.topic + " stopped arriving within max_gap");
    return;
  }

  if ((now - first_zero_stamp_.value()).seconds() >= config_.duration.value()) {
    result_.state = AssertionState::Passed;
    result_.message = "Observed zero Twist commands on " + config_.topic + " for " +
      std::to_string(config_.duration.value()) + " seconds after fault " + config_.fault_id;
  }
}

AssertionResult TwistStoppedAssertion::result() const
{
  return result_;
}

bool TwistStoppedAssertion::is_zero(const geometry_msgs::msg::Twist & message) const
{
  const double linear_tolerance = config_.linear_tolerance.value();
  const double angular_tolerance = config_.angular_tolerance.value();
  return std::abs(message.linear.x) <= linear_tolerance &&
         std::abs(message.linear.y) <= linear_tolerance &&
         std::abs(message.linear.z) <= linear_tolerance &&
         std::abs(message.angular.x) <= angular_tolerance &&
         std::abs(message.angular.y) <= angular_tolerance &&
         std::abs(message.angular.z) <= angular_tolerance;
}

bool TwistStoppedAssertion::is_finite(const geometry_msgs::msg::Twist & message) const
{
  return std::isfinite(message.linear.x) && std::isfinite(message.linear.y) &&
         std::isfinite(message.linear.z) && std::isfinite(message.angular.x) &&
         std::isfinite(message.angular.y) && std::isfinite(message.angular.z);
}

void TwistStoppedAssertion::fail(const std::string & message)
{
  result_.state = AssertionState::Failed;
  result_.message = message;
}
}  // namespace ros2_fault_injection::assertions
