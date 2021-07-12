// Copyright 2021 ros2_control Development Team
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

#include "system_bolt.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

/*Connection to ODRI for read sensors and write commands*/
#include "odri_control_interface/utils.hpp"
#include "odri_control_interface/imu.hpp"



using namespace odri_control_interface;
using namespace Eigen;


#include <iostream>
#include <stdexcept>

SystemBoltHardware Bolt;

/*typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<bool, 6, 1> Vector6b;

typedef Eigen::Matrix<long, 3, 1> Vector3l;
typedef Eigen::Matrix<long, 4, 1> Vector4l;
typedef Eigen::Matrix<long, 6, 1> Vector6l;
typedef Eigen::Matrix<int, 6, 1> Vector6i;*/


// Define Bolt robot 
//auto main_board_ptr_ = std::make_shared<MasterBoardInterface>(argv[1]);
//auto main_board_ptr_ = std::make_shared<MasterBoardInterface>("enp3s0");



/*Vector6i motor_numbers(6);
motor_numbers(0) = 0;
motor_numbers(1) = 3;
motor_numbers(2) = 2;
motor_numbers(3) = 1;
motor_numbers(4) = 5;
motor_numbers(5) = 4;*/

/*Vector6b motor_reversed;
motor_reversed << true, false, true, true, false, false;

Vector6d joint_lower_limits;
joint_lower_limits << -0.5, -1.7, -3.4, -0.5, -1.7, -3.4;     //Modif d'après lecture des capteurs (demo bolt)
Vector6d joint_upper_limits;
joint_upper_limits << 0.5, 1.7, +3.4, +0.5, +1.7, +3.4;       //Modif d'après lecture des capteurs (demo bolt)

// Define the joint module.
auto joints = std::make_shared<JointModules>(main_board_ptr_,
                                             motor_numbers,
                                             0.025,
                                             9.,
                                             12.,	  //MAX CURRENT = 12
                                             motor_reversed,
                                             joint_lower_limits,
                                             joint_upper_limits,
                                             80.,
                                             0.5);

// Define the IMU.
Vector3l rotate_vector;
Vector4l orientation_vector;
rotate_vector << 1, 2, 3;
orientation_vector << 1, 2, 3, 4;
auto imu = std::make_shared<IMU>(
    main_board_ptr_, rotate_vector, orientation_vector);

// Define the robot.
auto robot = std::make_shared<Robot>(main_board_ptr_, joints, imu);

*/



namespace ros2_control_bolt
{

/* Code issue from demo_bolt_actuator_control.cpp (ODRI)*/

Eigen::Vector6d desired_joint_position = Eigen::Vector6d::Zero();
Eigen::Vector6d desired_torque = Eigen::Vector6d::Zero();

/* Main driver interface.
robot_ = odri_control_interface::RobotFromYamlFile(
network_id_, ODRI_CONTROL_INTERFACE_YAML_PATH);*/

return_type SystemBoltHardware::configure(
  const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != return_type::OK) {
    return return_type::ERROR;
  }

  hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  hw_slowdown_ = stod(info_.hardware_parameters["example_param_hw_slowdown"]);

  //Joint number
  hw_joint_FLHAA_ = stod(info_.hardware_parameters["FLHAA"]);
  hw_joint_FLHFE_ = stod(info_.hardware_parameters["FLHFE"]);
  hw_joint_FLK_ = stod(info_.hardware_parameters["FLK"]);
  hw_joint_FRHAA_ = stod(info_.hardware_parameters["FRHAA"]);
  hw_joint_FRHFE_ = stod(info_.hardware_parameters["FRHFE"]);
  hw_joint_FRK_ = stod(info_.hardware_parameters["FRK"]);

  // For each joint.
  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    hw_states_[joint.name] =
      {std::numeric_limits<double>::quiet_NaN(),
       std::numeric_limits<double>::quiet_NaN(),
       std::numeric_limits<double>::quiet_NaN(),
       std::numeric_limits<double>::quiet_NaN(),
       std::numeric_limits<double>::quiet_NaN()};
    hw_commands_[joint.name] =
      {std::numeric_limits<double>::quiet_NaN(),
       std::numeric_limits<double>::quiet_NaN(),
       std::numeric_limits<double>::quiet_NaN(),
       std::numeric_limits<double>::quiet_NaN(),
       std::numeric_limits<double>::quiet_NaN()};
    control_mode_[joint.name] = control_mode_t::NO_VALID_MODE;


    // SystemBolt has exactly 5 doubles for the state and
    // 5 doubles for the command interface on each joint
    if (joint.command_interfaces.size() !=
      bolt_list_of_cmd_inter.size())
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("SystemBoltHardware"),
        "Joint '%s' has %d command interfaces found.",             // 5 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return return_type::ERROR;
    }

    // For each command interface of the joint
    for (const auto & a_joint_cmd_inter : joint.command_interfaces)
    {
      // Check if the command interface is inside the list
      if (bolt_list_of_cmd_inter.find(a_joint_cmd_inter.name) ==
        bolt_list_of_cmd_inter.end())
      {
        // If not then generate an error message
        RCLCPP_FATAL(
          rclcpp::get_logger("SystemBoltHardware"),
          "Joint '%s' have %s command interfaces found. One of the following values is expected",
          joint.name.c_str(),
          a_joint_cmd_inter.name.c_str());
        for (const auto & a_cmd_inter : bolt_list_of_cmd_inter)
        {
          RCLCPP_FATAL(
            rclcpp::get_logger("SystemBoltHardware"),
            "'%s' expected.", a_cmd_inter.c_str());
        }
        return return_type::ERROR;
      }
    }

    // Check if the state interface list is of the right size
    if (joint.state_interfaces.size() !=
      bolt_list_of_state_inter.size())
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("SystemBoltHardware"),
        "Joint '%s' has %d state interface.",                      // 5 expected.",
        joint.name.c_str(), joint.state_interfaces.size());
      return return_type::ERROR;
    }

    // For each state interface of the joint
    for (const auto & a_joint_state_inter : joint.state_interfaces)
    {
      std::string joint_state_inter_name = a_joint_state_inter.name;

      // Check if the state interface is inside the list
      if (bolt_list_of_state_inter.find(joint_state_inter_name) ==
        bolt_list_of_state_inter.end())
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("SystemBoltHardware"),
          "Joint '%s' have %s state interface. One of the following was expected: ",
          joint.name.c_str(),
          a_joint_state_inter.name.c_str());

        for (const auto & a_state_inter : bolt_list_of_state_inter)
        {
          RCLCPP_FATAL(
            rclcpp::get_logger("SystemBoltHardware"),
            "'%s' expected.", a_state_inter.c_str());
        }
        return return_type::ERROR;
      }
    }
  }

  status_ = hardware_interface::status::CONFIGURED;
  RCLCPP_INFO(
    rclcpp::get_logger("SystemBoltHardware"),
    "Finished configure() %p",this);

  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    RCLCPP_INFO(
      rclcpp::get_logger("SystemBoltHardware"),
      "Joint '%s' configuration.",
      joint.name.c_str());
  }
  return return_type::OK;
}


return_type
SystemBoltHardware::prepare_command_mode_switch
(
 const std::vector<std::string> & start_interfaces,
 const std::vector<std::string> & stop_interfaces
 )
{

  RCLCPP_INFO(
    rclcpp::get_logger("SystemBoltHardware"),
    "Going through SystemBoltHardware::prepare_command_mode_switch %d",
    start_interfaces.size());

  // Initialize new modes.
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
    new_modes_[joint.name] = control_mode_t::NO_VALID_MODE;

  /// Check that the key interfaces are coherent
  for (auto & key : start_interfaces) {
    RCLCPP_INFO(rclcpp::get_logger("SystemBoltHardware"),
		"prepare_command_mode_switch %s",key.c_str());

    /// For each joint
    for (const hardware_interface::ComponentInfo & joint : info_.joints) {
      if (key == joint.name + "/" + hardware_interface::HW_IF_POSITION)
	{
	  new_modes_[joint.name]=control_mode_t::POSITION;
	  RCLCPP_INFO(rclcpp::get_logger("SystemBoltHardware"),
		      "%s switch to position",key.c_str());

	}
      if (key == joint.name + "/" + hardware_interface::HW_IF_VELOCITY)
	{
	  new_modes_[joint.name]=control_mode_t::VELOCITY;
	}
      if (key == joint.name + "/" + hardware_interface::HW_IF_EFFORT)
	{
	  new_modes_[joint.name]=control_mode_t::EFFORT;
	}
      if (key == joint.name + "/" + ros2_control_bolt::HW_IF_GAIN_KP)
	{
	  new_modes_[joint.name]=control_mode_t::POS_VEL_EFF_GAINS;
	}
      if (key == joint.name + "/" + ros2_control_bolt::HW_IF_GAIN_KD)
	{
	  new_modes_[joint.name]=control_mode_t::POS_VEL_EFF_GAINS;
	}
    }
  }
  // Stop motion on all relevant joints that are stopping
  for (std::string key : stop_interfaces)
  {
    for (const hardware_interface::ComponentInfo & joint : info_.joints) {
      if (key.find(joint.name) != std::string::npos)
      {
        hw_commands_[joint.name].velocity = 0.0;
        hw_commands_[joint.name].effort = 0.0;
        control_mode_[joint.name] = control_mode_t::NO_VALID_MODE;  // Revert to undefined
      }
    }
  }
  // Set the new command modes
  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    if ((control_mode_[joint.name] == control_mode_t::NO_VALID_MODE) &&
	(new_modes_[joint.name]==control_mode_t::NO_VALID_MODE))
    {
      // Something else is using the joint! Abort!
      RCLCPP_ERROR(
        rclcpp::get_logger("SystemBoltHardware"),
	"Joint '%s' has no valid control mode %d %d",
	joint.name.c_str(),
	control_mode_[joint.name],
	new_modes_[joint.name]

      );
      return return_type::ERROR;
    }
    control_mode_[joint.name] = new_modes_[joint.name];
  }

  return return_type::OK;
}

std::vector<hardware_interface::StateInterface>
SystemBoltHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint.name, hardware_interface::HW_IF_POSITION,
        &hw_states_[joint.name].position));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint.name, hardware_interface::HW_IF_VELOCITY,
        &hw_states_[joint.name].velocity));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint.name, hardware_interface::HW_IF_EFFORT,
        &hw_states_[joint.name].effort));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint.name, HW_IF_GAIN_KP,
        &hw_states_[joint.name].effort));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint.name, HW_IF_GAIN_KD,
        &hw_states_[joint.name].effort));
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    RCLCPP_INFO(
      rclcpp::get_logger("SystemBoltHardware"),
      "Joint '%s' export_state_interface.",
      joint.name.c_str());
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
SystemBoltHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint.name, hardware_interface::HW_IF_POSITION,
        &hw_commands_[joint.name].position));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint.name, hardware_interface::HW_IF_VELOCITY,
        &hw_commands_[joint.name].velocity));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint.name, hardware_interface::HW_IF_EFFORT,
        &hw_commands_[joint.name].effort));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint.name, HW_IF_GAIN_KP,
        &hw_commands_[joint.name].Kp));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint.name, HW_IF_GAIN_KD,
        &hw_commands_[joint.name].Kd));
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    RCLCPP_INFO(
      rclcpp::get_logger("SystemBoltHardware"),
      "Joint '%s' export_command_interface.",
      joint.name.c_str());
  }

  return command_interfaces;
}









////// START


/*robot->Start();*/


return_type SystemBoltHardware::start()
{
  RCLCPP_INFO(
    rclcpp::get_logger("SystemBoltHardware"),
    "Starting ...please wait...");

/*Starting countdown*/
  for (auto i = 0; i <= hw_start_sec_; ++i) {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("SystemBoltHardware"),
      "%.1f seconds left...", hw_start_sec_ - i);
  }

  // set some default values
  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    if (std::isnan(hw_states_[joint.name].position)) {
      hw_states_[joint.name] = {0.0, 0.0, 0.0, 0.0, 0.0};
      hw_commands_[joint.name] = {0.0, 0.0, 0.0, 0.0, 0.0};
    }
  }

  status_ = hardware_interface::status::STARTED;

  RCLCPP_INFO(
    rclcpp::get_logger("SystemBoltHardware"),
    "System Sucessfully started!");

  return return_type::OK;
}










////// STOP


return_type SystemBoltHardware::stop()
{
  RCLCPP_INFO(
    rclcpp::get_logger("SystemBoltHardware"),
    "Stopping ...please wait...");

  for (int i = 0; i <= hw_stop_sec_; i++) {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("SystemBoltHardware"),
      "%.1f seconds left...", hw_stop_sec_ - i);
  }

  status_ = hardware_interface::status::STOPPED;

  RCLCPP_INFO(
    rclcpp::get_logger("SystemBoltHardware"),
    "System sucessfully stopped!");

  return return_type::OK;
}







////// READ


hardware_interface::return_type SystemBoltHardware::read()
{

   // RCLCPP_INFO(
  //rclcpp::get_logger("SystemBoltHardware"),
  //"Reading...");

  // Data acquisition
  /*robot_->ParseSensorData();

  auto sensor_positions = joints->GetPositions();
  auto sensor_velocities = joints->GetVelocities();
  auto measured_torques = joints->GetMeasuredTorques();
  auto imu_gyro = imu->GetGyroscope();
  auto imu_accelero = imu->GetAccelerometer();
  auto imu_line_acc = imu->GetLinearAcceleration();
  auto imu_euler = imu->GetAttitudeEuler();
  auto imu_quater = imu->GetAttitudeQuaternion();


  hw_states_["FLHAA"].position = sensor_positions[hw_joint_FLHAA_]
  hw_states_["FLHFE"].position = sensor_positions[hw_joint_FLHFE_]
  hw_states_["FLK"].position = sensor_positions[hw_joint_FLK_]
  hw_states_["FRHAA"].position = sensor_positions[hw_joint_FRHAA_]
  hw_states_["FRHFE"].position = sensor_positions[hw_joint_FRHFE_]
  hw_states_["FRK"].position = sensor_positions[hw_joint_FRK_]

  hw_states_["FLHAA"].velocity = sensor_velocities[hw_joint_FLHAA_]
  hw_states_["FLHFE"].velocity = sensor_velocities[hw_joint_FLHFE_]
  hw_states_["FLK"].velocity = sensor_velocities[hw_joint_FLK_]
  hw_states_["FRHAA"].velocity = sensor_velocities[hw_joint_FRHAA_]
  hw_states_["FRHFE"].velocity = sensor_velocities[hw_joint_FRHFE_]
  hw_states_["FRK"].velocity = sensor_velocities[hw_joint_FRK_]

  hw_states_["FLHAA"].effort = measured_torques[hw_joint_FLHAA_]
  hw_states_["FLHFE"].effort = measured_torques[hw_joint_FLHFE_]
  hw_states_["FLK"].effort = measured_torques[hw_joint_FLK_]
  hw_states_["FRHAA"].effort = measured_torques[hw_joint_FRHAA_]
  hw_states_["FRHFE"].effort = measured_torques[hw_joint_FRHFE_]
  hw_states_["FRK"].effort = measured_torques[hw_joint_FRK_]*/
  

/*
  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    //Sensors
    //Position

    std::cout << "Joints: ";
    joint->PrintVector(joint->GetPositions());
    std::cout << std::endl;
  }
  // RCLCPP_INFO(
  //rclcpp::get_logger("SystemBoltHardware"),
  //"Joints sucessfully read!");


*/

  /*

    //Velocity
    hw_states_[joint.name].velocity = robot.get_joint_velocities(i,0);

    //Torque
    hw_states_[joint.name].effort = robot.get_joint_torques(i,0);



    // Cf ros2_control/controller_interface/include/semantic_components/imu_sensor.hpp 
    //IMU quaternion
    //x
    interface_names_.emplace_back(name_ + "/" + "orientation.x") = robot.get_base_attitude_quaternion(0,0);
    //y
    interface_names_.emplace_back(name_ + "/" + "orientation.y") = robot.get_base_attitude_quaternion(1,0);
    //z
    interface_names_.emplace_back(name_ + "/" + "orientation.z") = robot.get_base_attitude_quaternion(2,0);
    //w
    interface_names_.emplace_back(name_ + "/" + "orientation.w") = robot.get_base_attitude_quaternion(3,0);


    //IMU attitude
    robot.get_base_attitude();

    //IMU accelerometer
    robot.get_base_accelerometer();

    //IMU gyroscope
    robot.get_base_gyroscope();

    //IMU linear acceleration
    for (size_t i = 0; i < linear_acceleration_.size(); ++i) {
      linear_acceleration_[i] = state_interfaces_[interface_offset + i].get().get_value();
    }
    interface_names_.emplace_back(name_ + "/" + "linear_acceleration.x") = robot.get_base_linear_acceleration(0,0);
    interface_names_.emplace_back(name_ + "/" + "linear_acceleration.y") = robot.get_base_linear_acceleration(1,0);
    interface_names_.emplace_back(name_ + "/" + "linear_acceleration.z") = robot.get_base_linear_acceleration(2,0);
  }*/


/*

  // RCLCPP_INFO(
  //   rclcpp::get_logger("SystemBoltHardware"),
  //   "Reading...");
  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    switch(control_mode_[joint.name]) {
    case control_mode_t::POS_VEL_EFF_GAINS:
      // Simulate Bolt's PD+ computation
      hw_states_[joint.name].effort = hw_commands_[joint.name].effort +
	hw_commands_[joint.name].Kp*
	(hw_states_[joint.name].position -
	 hw_commands_[joint.name].position) +
	hw_commands_[joint.name].Kd*
	(hw_states_[joint.name].velocity -
	 hw_commands_[joint.name].velocity);
      break;
    case control_mode_t::POSITION:
      hw_states_[joint.name].position =
	hw_commands_[joint.name].position;
      break;
    case control_mode_t::VELOCITY:
      hw_states_[joint.name].position =
	hw_states_[joint.name].position +
	hw_slowdown_ * hw_states_[joint.name].velocity;
      hw_states_[joint.name].velocity =
	hw_commands_[joint.name].velocity;
      break;
    case control_mode_t::EFFORT:
      hw_states_[joint.name].effort =
	hw_commands_[joint.name].effort;
      break;
    case control_mode_t::NO_VALID_MODE:
      hw_states_[joint.name].effort = 0.0;
      break;
    }
    //
    // RCLCPP_INFO(
    //   rclcpp::get_logger("SystemBoltHardware"),
    //   "Got state (%.5f,%.5f,%.5f) for joint %d!",
    //   hw_states_[i].position, hw_states_[i].velocity,
    //   hw_states_[i].effort, i);
  }
  // RCLCPP_INFO(
  //   rclcpp::get_logger("SystemBoltHardware"),
  //   "Joints sucessfully read!");
*/
  return return_type::OK;
}










////// WRITE


hardware_interface::return_type
SystemBoltHardware::write()
{
  // This part of the code sends command to the real robot.

  // RCLCPP_INFO(
  //   rclcpp::get_logger("SystemBoltHardware"),
  //   "Writing...");
  //  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    // Simulate sending commands to the hardware
    // RCLCPP_INFO(
    //   rclcpp::get_logger("SystemBoltHardware"),
    //   "Got command (%.5f,%.5f,%.5f,%.5f,%.5f) for joint %d!",
    //   hw_commands_[i].position,
    //   hw_commands_[i].velocity,
    //   hw_commands_[i].effort,
    //   hw_commands_[i].Kp,
    //   hw_commands_[i].Kd,
    //   i);
  // }
  // RCLCPP_INFO(
  //   rclcpp::get_logger("SystemBoltHardware"),
  //   "Joints sucessfully written!");

  return return_type::OK;
}











}  // namespace ros2_control_bolt

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_control_bolt::SystemBoltHardware,
  hardware_interface::SystemInterface
)
