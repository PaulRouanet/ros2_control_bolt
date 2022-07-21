// Copyright (c) 2021, PickNik, Inc.
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
//
/// \authors: Jack Center, Denis Stogl

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "gmock/gmock.h"

#include "test_position_velocity_effort_gain_controller.hpp"

#include "position_velocity_effort_gain_controller/position_velocity_effort_gain_controller.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/wait_set.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

using hardware_interface::LoanedCommandInterface;

namespace
{
rclcpp::WaitResultKind wait_for(rclcpp::SubscriptionBase::SharedPtr subscription)
{
  rclcpp::WaitSet wait_set;
  wait_set.add_subscription(subscription);
  const auto timeout = std::chrono::seconds(10);
  return wait_set.wait(timeout).kind();
}
}  // namespace

void PosVelTorGainsControllerTestTest::SetUpTestCase() { rclcpp::init(0, nullptr); }

void PosVelTorGainsControllerTestTest::TearDownTestCase() { rclcpp::shutdown(); }

void PosVelTorGainsControllerTestTest::SetUp()
{
  // initialize controller
  controller_ = std::make_unique<FriendPosVelTorGainsControllerTest>();
}

void PosVelTorGainsControllerTestTest::TearDown() { controller_.reset(nullptr); }

void PosVelTorGainsControllerTestTest::SetUpController(bool set_params_and_activate)
{
  const auto result = controller_->init("position_velocity_effort_gain_controller");
  ASSERT_EQ(result, controller_interface::return_type::OK);

  std::vector<LoanedCommandInterface> command_ifs;
  command_ifs.emplace_back(joint_1_pos_cmd_);
  command_ifs.emplace_back(joint_1_vel_cmd_);
  command_ifs.emplace_back(joint_1_eff_cmd_);
  command_ifs.emplace_back(joint_1_kp_cmd_);
  command_ifs.emplace_back(joint_1_kd_cmd_);
  controller_->assign_interfaces(std::move(command_ifs), {});

  if (set_params_and_activate)
  {
    SetParametersAndActivateController();
  }
}

void PosVelTorGainsControllerTestTest::SetParametersAndActivateController()
{
  controller_->get_node()->set_parameter({"joint", "joint1"});
  controller_->get_node()->set_parameter(
    {"interface_names", std::vector<std::string>{"position", "velocity", "effort", "gain_kp", "gain_kd"}});

  auto node_state = controller_->get_node()->configure();
  ASSERT_EQ(node_state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  node_state = controller_->get_node()->activate();
  ASSERT_EQ(node_state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
}

TEST_F(PosVelTorGainsControllerTestTest, JointsParameterNotSet)
{
  SetUpController();
  controller_->get_node()->set_parameter({"interface_names", std::vector<std::string>()});

  // configure failed, 'joint' parameter not set
  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::ERROR);
}

TEST_F(PosVelTorGainsControllerTestTest, InterfaceParameterNotSet)
{
  SetUpController();
  controller_->get_node()->set_parameter({"joint", ""});

  // configure failed, 'interface_names' parameter not set
  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::ERROR);
}

TEST_F(PosVelTorGainsControllerTestTest, JointsParameterIsEmpty)
{
  SetUpController();

  controller_->get_node()->set_parameter({"joint", ""});
  controller_->get_node()->set_parameter({"interface_names", std::vector<std::string>()});

  // configure failed, 'joint' is empty
  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::ERROR);
}

TEST_F(PosVelTorGainsControllerTestTest, InterfaceParameterEmpty)
{
  SetUpController();
  controller_->get_node()->set_parameter({"joint", "joint1"});
  controller_->get_node()->set_parameter({"interface_names", std::vector<std::string>()});

  // configure failed, 'interface_name' is empty
  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::ERROR);
}

TEST_F(PosVelTorGainsControllerTestTest, ConfigureParamsSuccess)
{
  SetUpController();

  controller_->get_node()->set_parameter({"joint", "joint1"});
  controller_->get_node()->set_parameter(
    {"interface_names", std::vector<std::string>{"position", "velocity", "effort", "gain_kp", "gain_kd"}});

  // configure successful
  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
}

TEST_F(PosVelTorGainsControllerTestTest, ActivateWithWrongJointsNamesFails)
{
  SetUpController();

  controller_->get_node()->set_parameter({"joint", "joint2"});
  controller_->get_node()->set_parameter(
    {"interface_names", std::vector<std::string>{"position", "velocity", "effort", "gain_kp", "gain_kd"}});

  // activate failed, 'joint2' is not a valid joint name for the hardware
  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::ERROR);
}

TEST_F(PosVelTorGainsControllerTestTest, ActivateWithWrongInterfaceNameFails)
{
  SetUpController();

  controller_->get_node()->set_parameter({"joint", "joint1"});
  controller_->get_node()->set_parameter(
    {"interface_names", std::vector<std::string>{"position", "velocity", "effort", "gain_kp", "gain_kd"}});

  // activate failed, 'acceleration' is not a registered interface for `joint1`
  ASSERT_EQ(
    controller_->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::ERROR);
}

TEST_F(PosVelTorGainsControllerTestTest, ActivateSuccess)
{
  SetUpController(true);

  // check joint commands are the default ones
  ASSERT_EQ(joint_1_pos_cmd_.get_value(), 1.1);
  ASSERT_EQ(joint_1_vel_cmd_.get_value(), 2.1);
  ASSERT_EQ(joint_1_eff_cmd_.get_value(), 3.1);
  ASSERT_EQ(joint_1_kp_cmd_.get_value(), 4.1);
  ASSERT_EQ(joint_1_kd_cmd_.get_value(), 5.1);
}

TEST_F(PosVelTorGainsControllerTestTest, CommandSuccessTest)
{
  SetUpController(true);

  // send command
  auto command_ptr = std::make_shared<position_velocity_effort_gain_controller::CmdType>();
  command_ptr->data = {10.0, 20.0, 30.0, 40.0, 50.0};
  controller_->rt_command_ptr_.writeFromNonRT(command_ptr);

  // update successful, command received
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0.1), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  // check command in handle was set
  ASSERT_EQ(joint_1_pos_cmd_.get_value(), 10.0);
  ASSERT_EQ(joint_1_vel_cmd_.get_value(), 20.0);
  ASSERT_EQ(joint_1_eff_cmd_.get_value(), 30.0);
  ASSERT_EQ(joint_1_kp_cmd_.get_value(), 40.0);
  ASSERT_EQ(joint_1_kd_cmd_.get_value(), 50.0);
}

TEST_F(PosVelTorGainsControllerTestTest, NoCommandCheckTest)
{
  SetUpController(true);

  // update successful, no command received yet
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  // check joint commands are still the default ones
  ASSERT_EQ(joint_1_pos_cmd_.get_value(), 1.1);
  ASSERT_EQ(joint_1_vel_cmd_.get_value(), 2.1);
  ASSERT_EQ(joint_1_eff_cmd_.get_value(), 3.1);
  ASSERT_EQ(joint_1_kp_cmd_.get_value(), 4.1);
  ASSERT_EQ(joint_1_kd_cmd_.get_value(), 5.1);
}

TEST_F(PosVelTorGainsControllerTestTest, WrongCommandCheckTest)
{
  SetUpController(true);

  // send command with wrong number of joints
  auto command_ptr = std::make_shared<position_velocity_effort_gain_controller::CmdType>();
  command_ptr->data = {10.0, 20.0};
  controller_->rt_command_ptr_.writeFromNonRT(command_ptr);

  // update failed, command size does not match number of joints
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::ERROR);

  // check joint commands are still the default ones
  ASSERT_EQ(joint_1_pos_cmd_.get_value(), 1.1);
  ASSERT_EQ(joint_1_vel_cmd_.get_value(), 2.1);
  ASSERT_EQ(joint_1_eff_cmd_.get_value(), 3.1);
  ASSERT_EQ(joint_1_kp_cmd_.get_value(), 4.1);
  ASSERT_EQ(joint_1_kd_cmd_.get_value(), 5.1);
}

TEST_F(PosVelTorGainsControllerTestTest, CommandCallbackTest)
{
  SetUpController(true);

  // send a new command
  rclcpp::Node test_node("test_node");
  auto command_pub = test_node.create_publisher<std_msgs::msg::Float64MultiArray>(
    std::string(controller_->get_node()->get_name()) + "/commands", rclcpp::SystemDefaultsQoS());
  std_msgs::msg::Float64MultiArray command_msg;
  command_msg.data = {10.0, 20.0, 30.0, 40.0, 50.0};
  command_pub->publish(command_msg);

  // wait for command message to be passed
  ASSERT_EQ(wait_for(controller_->joints_command_subscriber_), rclcpp::WaitResultKind::Ready);

  // process callbacks
  rclcpp::spin_some(controller_->get_node()->get_node_base_interface());

  // update successful
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  // check command in handle was set
  ASSERT_EQ(joint_1_pos_cmd_.get_value(), 10.0);
  ASSERT_EQ(joint_1_vel_cmd_.get_value(), 20.0);
  ASSERT_EQ(joint_1_eff_cmd_.get_value(), 30.0);
  ASSERT_EQ(joint_1_kp_cmd_.get_value(), 40.0);
  ASSERT_EQ(joint_1_kd_cmd_.get_value(), 50.0);
}

TEST_F(PosVelTorGainsControllerTestTest, ActivateDeactivateCommandsResetSuccess)
{
  SetUpController(true);

  // send command
  auto command_ptr = std::make_shared<position_velocity_effort_gain_controller::CmdType>();
  command_ptr->data = {10.0, 20.0, 30.0, 40.0, 50.0};
  controller_->rt_command_ptr_.writeFromNonRT(command_ptr);

  // update successful, command received
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0.1), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  // check command in handle was set
  ASSERT_EQ(joint_1_pos_cmd_.get_value(), 10.0);
  ASSERT_EQ(joint_1_vel_cmd_.get_value(), 20.0);
  ASSERT_EQ(joint_1_eff_cmd_.get_value(), 30.0);
  ASSERT_EQ(joint_1_kp_cmd_.get_value(), 40.0);
  ASSERT_EQ(joint_1_kd_cmd_.get_value(), 50.0);

  auto node_state = controller_->get_node()->deactivate();
  ASSERT_EQ(node_state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  // command ptr should be reset (nullptr) after deactivation - same check as in `update`
  ASSERT_FALSE(
    controller_->rt_command_ptr_.readFromNonRT() &&
    *(controller_->rt_command_ptr_.readFromNonRT()));
  ASSERT_FALSE(
    controller_->rt_command_ptr_.readFromRT() && *(controller_->rt_command_ptr_.readFromRT()));

  // Controller is inactive but let's put some data into buffer (simulate callback when inactive)
  auto command_msg = std::make_shared<std_msgs::msg::Float64MultiArray>();
  command_msg->data = {5.5, 6.6, 7.7, 8.8, 9.9};
  controller_->rt_command_ptr_.writeFromNonRT(command_msg);

  // command ptr should be available and message should be there - same check as in `update`
  ASSERT_TRUE(
    controller_->rt_command_ptr_.readFromNonRT() &&
    *(controller_->rt_command_ptr_.readFromNonRT()));
  ASSERT_TRUE(
    controller_->rt_command_ptr_.readFromRT() && *(controller_->rt_command_ptr_.readFromRT()));

  // Now activate again
  node_state = controller_->get_node()->activate();
  ASSERT_EQ(node_state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  // command ptr should be reset (nullptr) after activation - same check as in `update`
  ASSERT_FALSE(
    controller_->rt_command_ptr_.readFromNonRT() &&
    *(controller_->rt_command_ptr_.readFromNonRT()));
  ASSERT_FALSE(
    controller_->rt_command_ptr_.readFromRT() && *(controller_->rt_command_ptr_.readFromRT()));

  // update successful
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  // values should not change
  ASSERT_EQ(joint_1_pos_cmd_.get_value(), 10.0);
  ASSERT_EQ(joint_1_vel_cmd_.get_value(), 20.0);
  ASSERT_EQ(joint_1_eff_cmd_.get_value(), 30.0);
  ASSERT_EQ(joint_1_kp_cmd_.get_value(), 40.0);
  ASSERT_EQ(joint_1_kd_cmd_.get_value(), 50.0);

  // set commands again
  controller_->rt_command_ptr_.writeFromNonRT(command_msg);

  // update successful
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  // check command in handle was set
  ASSERT_EQ(joint_1_pos_cmd_.get_value(), 5.5);
  ASSERT_EQ(joint_1_vel_cmd_.get_value(), 6.6);
  ASSERT_EQ(joint_1_eff_cmd_.get_value(), 7.7);
  ASSERT_EQ(joint_1_kp_cmd_.get_value(), 8.8);
  ASSERT_EQ(joint_1_kd_cmd_.get_value(), 9.9);
}