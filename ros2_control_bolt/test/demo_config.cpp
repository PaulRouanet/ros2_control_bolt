/* Utile pour la lecture de capteur ---> read() de system_bolt.cpp*/
/* Inspiré de demo_create_bolt_robot.cpp*/



#include <system_bolt.hpp>

#include <odri_control_interface/utils.hpp>
#include <odri_control_interface/imu.hpp>


using namespace odri_control_interface;

#include <iostream>
#include <stdexcept>


int main(int argc, char **argv)
{
    if (argc != 2)
    {
        throw std::runtime_error(
            "Please provide the interface name "
            "(i.e. using 'ifconfig' on linux");
    }

    nice(-20);  // Give the process a high priority.

    //configure()

    // Start the robot.
    robot->Start();


    int c = 0;
    std::chrono::time_point<std::chrono::system_clock> last =
        std::chrono::system_clock::now();
    while (!robot->IsTimeout())
    {
        if (((std::chrono::duration<double>)(std::chrono::system_clock::now()-last)).count() > 0.001)
        {
            last = std::chrono::system_clock::now();  // last+dt would be better
            robot->ParseSensorData();

            c++;
            if (c % 1000 == 0)
            {
                std::cout << "Count :                        " << c << "\n";
                std::cout << "\n";

                std::cout << "Joints : \n";
                std::cout << "Position:                      ";
                joints->PrintVector(joints->GetPositions());
                std::cout << "\n";
                std::cout << "Velocities:                    ";
                joints->PrintVector(joints->GetVelocities());
                std::cout << "\n";
                std::cout << "Measured Torques:              ";
                joints->PrintVector(joints->GetMeasuredTorques());
                std::cout << "\n";
                std::cout << "\n";

                std::cout << "IMU : \n";
                std::cout << "Gyroscope                      ";
                joints->PrintVector(imu->GetGyroscope());
                std::cout << "\n";
                std::cout << "Accelerometer                  ";
                joints->PrintVector(imu->GetAccelerometer());
                std::cout << "\n";
                std::cout << "Linear Acceleration            ";
                joints->PrintVector(imu->GetLinearAcceleration());
                std::cout << "\n";
                std::cout << "Attitude Euler                 ";
                joints->PrintVector(imu->GetAttitudeEuler());
                std::cout << "\n";
                std::cout << "Attitude Quaternion            ";
                joints->PrintVector(imu->GetAttitudeQuaternion());
                std::cout << "\n";
                std::cout << "\n";
                std::cout << "\n";
                std::cout << "\n";
                std::cout << std::endl;
                
            }
        }
        else
        {
            std::this_thread::yield();
        }
    }
    std::cout << "Normal program shutdown." << std::endl;
    return 0;
}

