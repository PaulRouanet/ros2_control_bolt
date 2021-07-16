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
using namespace semantic_components;



#include <iostream>
#include <stdexcept>



namespace ros2_control_bolt
{

/* Code issue from demo_bolt_actuator_control.cpp (ODRI)*/

Eigen::Vector6d desired_joint_position = Eigen::Vector6d::Zero();
Eigen::Vector6d desired_torque = Eigen::Vector6d::Zero();


return_type SystemBoltHardware::init_robot(const hardware_interface::HardwareInfo & info, std::shared_ptr<odri_control_interface::Robot> & robot)
{
  /*Définir le robot (ODRI) à partir des variables de l'URDF*/

  // Define board (ODRI)
  auto main_board_ptr_ = std::make_shared<MasterBoardInterface>(argv[1]);

  // Define joints (ODRI)
  motor_numbers << 0, 3, 2, 1, 5, 4;
  motor_reversed_polarities << true, false, true, true, false, false;
  
  for (const hardware_interface::ComponentInfo & joint : info.joints) {
    // Joint lower and upper limits
    joint_lower_limits[joint_name_to_motor_nb[joint.name]] = stod(joint.command_interfaces[0].min);   //Modif d'après lecture des capteurs (demo bolt)
    joint_upper_limits[joint_name_to_motor_nb[joint.name]] = stod(joint.command_interfaces[0].max);   //Modif d'après lecture des capteurs (demo bolt)
  }

  auto joints = std::make_shared<JointModules>(main_board_ptr_,
                                               motor_numbers,
                                               motor_constants,
                                               gear_ratios,
                                               max_currents,
                                               motor_reversed_polarities,
                                               joint_lower_limits,
                                               joint_upper_limits,
                                               max_joint_velocities,
                                               safety_damping);

  // Define the IMU (ODRI).
  rotate_vector << 1, 2, 3;
  orientation_vector << 1, 2, 3, 4;
  auto imu = std::make_shared<IMU>(
      main_board_ptr_, rotate_vector, orientation_vector);
  
  // Define the robot (ODRI).
  robot = std::make_shared<Robot>(main_board_ptr_, joints, imu);

  //Definition of Kp and Kd :
  std::cout << "Enter Kp value (3 in casual use) : ";
  std::cin >> kp;
  std::cout << "Kp = " << kp;
  std::cout << "\n" << std::endl;

  std::cout << "Enter Kd value (0.05 in casual use) : ";
  std::cin >> kd;
  std::cout << "Kd = " << kd;
  std::cout << "\n" << std::endl;

/*
  for (const hardware_interface::ComponentInfo & joint : info.joints) {
    //Données des joints
    //Sorties (commandes)
    joint.command_interfaces;
    //Entrées (états)
    joint.state_interfaces;
    
  }

  for (const hardware_interface::ComponentInfo & sensor : info.sensors) {
    //Données des capteurs
    //Entrées (états)
    sensor.state_interfaces;
  }

    */

return return_type::OK;
}







return_type SystemBoltHardware::configure(
  const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != return_type::OK) {
    return return_type::ERROR;
  }


  hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  hw_slowdown_ = stod(info_.hardware_parameters["example_param_hw_slowdown"]);

  //For each sensor.
  for (const hardware_interface::ComponentInfo & sensor : info_.sensors) {
    imu_states_[sensor.name] =
      {std::numeric_limits<Eigen::Vector3d>::quiet_NaN(),
       std::numeric_limits<Eigen::Vector3d>::quiet_NaN(),
       std::numeric_limits<Eigen::Vector3d>::quiet_NaN(),
       std::numeric_limits<Eigen::Vector3d>::quiet_NaN(),
       std::numeric_limits<Eigen::Vector4d>::quiet_NaN()};
  }
  // For each joint.
  for (const hardware_interface::ComponentInfo & joint : info_.joints) {

    // Map joint with motor number
    // (uses at instead of [] because of joint constantness)
    joint_name_to_motor_nb[joint.name] = stoi(joint.parameters.at("motor_number"));
    
    // Initialize state of the joint by default to NaN
    // it allows to see which joints are not properly initialized
    // from the real hardware
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









// START

return_type SystemBoltHardware::start()
{
  robot->Start();

  // set some default values
  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    if (std::isnan(hw_states_[joint.name].position)) {
      hw_states_[joint.name] = {0.0, 0.0, 0.0, 0.0, 0.0};
      hw_commands_[joint.name] = {0.0, 0.0, 0.0, 0.0, 0.0};
    }
  }

  status_ = hardware_interface::status::STARTED;

/*  RCLCPP_INFO(
    rclcpp::get_logger("SystemBoltHardware"),
    "Starting ...please wait...");

//Starting countdown
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
    "System Sucessfully started!");*/



  return return_type::OK;
}










// STOP

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

  // Data acquisition (with ODRI)
  robot->ParseSensorData();

  auto sensor_positions = robot->joints->GetPositions();
  auto sensor_velocities = robot->joints->GetVelocities();
  auto measured_torques = robot->joints->GetMeasuredTorques();
   
  auto imu_gyroscope = robot->imu->GetGyroscope();
  auto imu_accelero = robot->imu->GetAccelerometer();
  auto imu_linear_acc = robot->imu->GetLinearAcceleration();
  auto imu_euler = robot->imu->GetAttitudeEuler();
  auto imu_quater = robot->imu->GetAttitudeQuaternion();


  // Assignment of sensor data to ros2_control variables (URDF)
  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    hw_states_[joint.name].position = sensor_positions[joint_name_to_motor_nb[joint.name]];
    hw_states_[joint.name].velocity = sensor_velocities[joint_name_to_motor_nb[joint.name]];
    hw_states_[joint.name].effort = measured_torques[joint_name_to_motor_nb[joint.name]];
    hw_states_[joint.name].Kp = kp;
    hw_states_[joint.name].Kd = kd;
  }
  
  // Assignment of IMU data (URDF)
  imu_states_["IMU"].gyro[0] = imu_gyroscope[0];
  imu_states_["IMU"].gyro[1] = imu_gyroscope[1];
  imu_states_["IMU"].gyro[2] = imu_gyroscope[2];

  imu_states_["IMU"].accelero[0] = imu_accelero[0];
  imu_states_["IMU"].accelero[1] = imu_accelero[1];
  imu_states_["IMU"].accelero[2] = imu_accelero[2];

  imu_states_["IMU"].line_acc[0] = imu_linear_acc[0];
  imu_states_["IMU"].line_acc[1] = imu_linear_acc[1];
  imu_states_["IMU"].line_acc[2] = imu_linear_acc[2];

  imu_states_["IMU"].euler[0] = imu_euler[0];
  imu_states_["IMU"].euler[1] = imu_euler[1];
  imu_states_["IMU"].euler[2] = imu_euler[2];

  imu_states_["IMU"].quater[0] = imu_quater[0];
  imu_states_["IMU"].quater[1] = imu_quater[1];
  imu_states_["IMU"].quater[2] = imu_quater[2];
  imu_states_["IMU"].quater[3] = imu_quater[3];
  

  // RCLCPP_INFO(
  //rclcpp::get_logger("SystemBoltHardware"),
  //"Joints sucessfully read!");

  return return_type::OK;
}










////// WRITE


hardware_interface::return_type
SystemBoltHardware::write()
{
  
  Eigen::Vector6d positions;
  Eigen::Vector6d velocities;
  Eigen::Vector6d torques;
  Eigen::Vector6d gain_KP;
  Eigen::Vector6d gain_KD;


  double t = 0.0;
  double dt = 0.001;
  std::chrono::time_point<std::chrono::system_clock> last = std::chrono::system_clock::now();

  while (!robot->IsTimeout()) {
    if (((std::chrono::duration<double>)(std::chrono::system_clock::now()-last)).count() > 0.001) {
      
      last = std::chrono::system_clock::now();  // last+dt would be better

      if (robot->IsReady()) {
        
        // Control law        
        /*positions = ...;
        torques =  ...;*/
        t += dt;
        
        for (const hardware_interface::ComponentInfo & joint : info_.joints) {
          positions[joint_name_to_motor_nb[joint.name]] = hw_commands_[joint.name].position;
          velocities[joint_name_to_motor_nb[joint.name]] = hw_commands_[joint.name].velocity;
          torques[joint_name_to_motor_nb[joint.name]] = hw_commands_[joint.name].effort;
          gain_KP[joint_name_to_motor_nb[joint.name]] = hw_commands_[joint.name].Kp;
          gain_KD[joint_name_to_motor_nb[joint.name]] = hw_commands_[joint.name].Kd;
        }

        robot->joints->SetDesiredPositions(positions);
        robot->joints->SetDesiredVelocities(velocities);
        robot->joints->SetTorques(torques);
        robot->joints->SetPositionGains(gain_KP);
        robot->joints->SetVelocityGains(gain_KD);
          
      }

      else {
        //Chose à faire si le robot n'est pas Ready
      }
    }
    robot->SendCommand();
  }


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
