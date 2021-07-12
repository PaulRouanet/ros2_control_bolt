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

#ifndef ROS2_CONTROL_BOLT__SYSTEM_BOLT_HPP_
#define ROS2_CONTROL_BOLT__SYSTEM_BOLT_HPP_

/*Connection to ODRI for read sensors and write commands*/
#include "odri_control_interface/calibration.hpp"
#include "odri_control_interface/robot.hpp"
//#include <Eigen/Eigen>
#include "semantic_components/imu_sensor.hpp"



#include <memory>
#include <set>
#include <string>
#include <vector>
#include <map>
#include "rclcpp/macros.hpp"

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "visibility_control.h"


using hardware_interface::return_type;

#define rt_printf printf

/**
 * @brief Usefull tool for the demos and programs in order to print data in
 * real time.
 *
 * @param v_name  is a string defining the data to print.
 * @param v the vector to print.
 */
void print_vector(std::string v_name, const Eigen::Ref<const Eigen::VectorXd> v)
{
    v_name += ": [";
    rt_printf("%s", v_name.c_str());
    for (int i = 0; i < v.size(); ++i)
    {
        rt_printf("%0.3f, ", v(i));
    }
    rt_printf("]\n");
}


namespace Eigen
{
/** @brief Eigen shortcut for vector of size 6. */
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<bool, 6, 1> Vector6b;
typedef Matrix<long, 6, 1> Vector6l;
typedef Matrix<int, 6, 1> Vector6i;
/** @brief Eigen shortcut for vector of size 3. */
typedef Matrix<long, 3, 1> Vector3l;
/** @brief Eigen shortcut for vector of size 4. */
typedef Matrix<long, 4, 1> Vector4l;

}  // namespace Eigen


namespace ros2_control_bolt
{



/**
 * @brief This boolean is here to kill cleanly the application upon ctrl+c
 */
//std::atomic_bool CTRL_C_DETECTED(false);

struct PosVelEffortGains
{
  double position;
  double velocity;
  double effort;
  double Kp;
  double Kd;
};

constexpr const auto HW_IF_GAIN_KP = "gain_kp";
constexpr const auto HW_IF_GAIN_KD = "gain_kd";

std::set<std::string> bolt_list_of_cmd_inter {
  "position",
  "velocity",
  "effort",
  "gain_kp",
  "gain_kd"
};

std::set<std::string> bolt_list_of_state_inter {
  "position",
  "velocity",
  "effort",
  "gain_kp",
  "gain_kd"
};

enum control_mode_t {
  POSITION,
  VELOCITY,
  EFFORT,
  POS_VEL_EFF_GAINS,
  NO_VALID_MODE
};

class SystemBoltHardware : public
  hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  
  // Constructeur
  SystemBoltHardware();
  
  // Destructeur
  ~SystemBoltHardware();

  RCLCPP_SHARED_PTR_DEFINITIONS(SystemBoltHardware)

  ROS2_CONTROL_BOLT_PUBLIC
  return_type configure(const hardware_interface::HardwareInfo & info) override;

  ROS2_CONTROL_BOLT_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ROS2_CONTROL_BOLT_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ROS2_CONTROL_BOLT_PUBLIC
  return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  ROS2_CONTROL_BOLT_PUBLIC
  return_type start() override;

  ROS2_CONTROL_BOLT_PUBLIC
  return_type stop() override;

  ROS2_CONTROL_BOLT_PUBLIC
  return_type read() override;

  ROS2_CONTROL_BOLT_PUBLIC
  return_type write() override;

  ROS2_CONTROL_BOLT_PUBLIC
  return_type display();



private:
  // Parameters for the RRBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;
  double hw_slowdown_;

  //Joint number from urdf
  double hw_joint_FLHAA_;
  double hw_joint_FLHFE_;
  double hw_joint_FLK_;
  double hw_joint_FRHAA_;
  double hw_joint_FRHFE_;
  double hw_joint_FRK_;

  // Store the command for the simulated robot
  std::map<std::string,PosVelEffortGains> hw_commands_;
  std::map<std::string,PosVelEffortGains> hw_states_;
  std::map<std::string,control_mode_t> control_mode_;

  std::map<std::string,control_mode_t> new_modes_;

  //Definition of multiple variables about Bolt
  // Joint
  Eigen::Vector6i motor_numbers;
  Eigen::Vector6b motor_reversed;
  Eigen::Vector6d joint_lower_limits;
  Eigen::Vector6d joint_upper_limits;
  // IMU
  Eigen::Vector3l rotate_vector;
  Eigen::Vector4l orientation_vector;
  

};

SystemBoltHardware::SystemBoltHardware()
{
  motor_numbers << 0, 3, 2, 1, 5, 4;
  motor_reversed << true, false, true, true, false, false;
  joint_lower_limits << -0.5, -1.7, -3.4, -0.5, -1.7, -3.4;     //Modif d'après lecture des capteurs (demo bolt)
  joint_upper_limits << 0.5, 1.7, +3.4, +0.5, +1.7, +3.4;       //Modif d'après lecture des capteurs (demo bolt)
  rotate_vector << 1, 2, 3;
  orientation_vector << 1, 2, 3, 4;

}

}  // namespace ros2_control_bolt

#endif  // ROS2_CONTROL_BOLT__SYSTEM_BOLT_HPP_
