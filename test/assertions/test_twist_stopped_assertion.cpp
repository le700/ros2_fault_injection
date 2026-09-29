// Copyright 2026 Reece Holland
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <limits>

#include <gtest/gtest.h>

#include "geometry_msgs/msg/twist.hpp"
#include "ros2_fault_injection/assertions/assertion_config.hpp"
#include "ros2_fault_injection/assertions/assertion_result.hpp"
#include "ros2_fault_injection/assertions/twist_stopped_assertion.hpp"
#include "ros2_fault_injection/msg/fault_event.hpp"

namespace ros2_fault_injection::assertions
{
namespace
{
AssertionConfig make_twist_stopped_assertion()
{
  AssertionConfig config;
  config.id = "watchdog_stops_after_cmd_dropout";
  config.type = "twist_stopped";
  config.topic = "/cmd_vel";
  config.fault_id = "drop_cmd_vel";
  config.trigger_within = 2.0;
  config.within = 0.5;
  config.duration = 0.5;
  config.tolerance = 0.01;
  config.max_gap = 0.2;
  return config;
}

geometry_msgs::msg::Twist moving_command()
{
  geometry_msgs::msg::Twist message;
  message.linear.x = 0.5;
  return message;
}

geometry_msgs::msg::Twist stopped_command()
{
  return geometry_msgs::msg::Twist();
}

msg::FaultEvent activate_fault()
{
  msg::FaultEvent event;
  event.fault_id = "drop_cmd_vel";
  event.state = "active";
  return event;
}
}  // namespace

TEST(TwistStoppedAssertion, StartsPending)
{
  const auto config = make_twist_stopped_assertion();
  TwistStoppedAssertion assertion(config);

  EXPECT_EQ(assertion.result().state, AssertionState::Pending);
}

TEST(TwistStoppedAssertion, PassesAfterZeroCommandIsHeldForDuration)
{
  const auto config = make_twist_stopped_assertion();
  TwistStoppedAssertion assertion(config);
  assertion.observe_message(moving_command(), rclcpp::Time(0, 0), 0.0);
  assertion.observe_fault_event(activate_fault(), 0.1);

  assertion.observe_message(stopped_command(), rclcpp::Time(0, 100000000), 0.1);
  assertion.observe_message(stopped_command(), rclcpp::Time(0, 300000000), 0.3);
  assertion.observe_message(stopped_command(), rclcpp::Time(0, 600000000), 0.6);
  assertion.update(0.7, rclcpp::Time(0, 700000000));

  EXPECT_EQ(assertion.result().state, AssertionState::Passed);
}

TEST(TwistStoppedAssertion, FailsIfFaultActivatesBeforeAnyMotionWasObserved)
{
  const auto config = make_twist_stopped_assertion();
  TwistStoppedAssertion assertion(config);
  assertion.observe_fault_event(activate_fault(), 0.1);

  EXPECT_EQ(assertion.result().state, AssertionState::Failed);
}

TEST(TwistStoppedAssertion, FailsWhenNoZeroCommandArrivesBeforeDeadline)
{
  const auto config = make_twist_stopped_assertion();
  TwistStoppedAssertion assertion(config);
  assertion.observe_message(moving_command(), rclcpp::Time(0, 0), 0.0);
  assertion.observe_fault_event(activate_fault(), 0.1);
  assertion.update(0.61, rclcpp::Time(0, 610000000));

  EXPECT_EQ(assertion.result().state, AssertionState::Failed);
}

TEST(TwistStoppedAssertion, FailsIfMotionResumesDuringHold)
{
  const auto config = make_twist_stopped_assertion();
  TwistStoppedAssertion assertion(config);
  assertion.observe_message(moving_command(), rclcpp::Time(0, 0), 0.0);
  assertion.observe_fault_event(activate_fault(), 0.1);
  assertion.observe_message(stopped_command(), rclcpp::Time(0, 200000000), 0.2);
  assertion.observe_message(moving_command(), rclcpp::Time(0, 300000000), 0.3);

  EXPECT_EQ(assertion.result().state, AssertionState::Failed);
}

TEST(TwistStoppedAssertion, FailsWhenZeroCommandsStopArrivingDuringHold)
{
  auto config = make_twist_stopped_assertion();
  config.duration = 1.0;
  TwistStoppedAssertion assertion(config);
  assertion.observe_message(moving_command(), rclcpp::Time(0, 0), 0.0);
  assertion.observe_fault_event(activate_fault(), 0.1);
  assertion.observe_message(stopped_command(), rclcpp::Time(0, 200000000), 0.2);
  assertion.update(0.41, rclcpp::Time(0, 410000000));

  EXPECT_EQ(assertion.result().state, AssertionState::Failed);
}

TEST(TwistStoppedAssertion, FailsOnNonFiniteVelocityAfterFaultActivation)
{
  const auto config = make_twist_stopped_assertion();
  TwistStoppedAssertion assertion(config);
  assertion.observe_message(moving_command(), rclcpp::Time(0, 0), 0.0);
  assertion.observe_fault_event(activate_fault(), 0.1);
  auto invalid = stopped_command();
  invalid.linear.x = std::numeric_limits<double>::quiet_NaN();
  assertion.observe_message(invalid, rclcpp::Time(0, 200000000), 0.2);

  EXPECT_EQ(assertion.result().state, AssertionState::Failed);
}

TEST(TwistStoppedAssertion, FailsWhenTheFaultDoesNotActivateBeforeDeadline)
{
  const auto config = make_twist_stopped_assertion();
  TwistStoppedAssertion assertion(config);
  assertion.update(2.1, rclcpp::Time(2, 100000000));

  EXPECT_EQ(assertion.result().state, AssertionState::Failed);
}
}  // namespace ros2_fault_injection::assertions
