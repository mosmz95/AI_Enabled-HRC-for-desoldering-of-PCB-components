#include <ur_rtde/rtde_io_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <iostream>
#include <thread>

using namespace ur_rtde;

int main()
{
  RTDEIOInterface rtde_io("192.168.0.100");
  RTDEReceiveInterface rtde_receive("192.168.0.100");

  /** How-to set and get standard and tool digital outputs. Notice that we need the
    * RTDEIOInterface for setting an output and RTDEReceiveInterface for getting the state
    * of an output.
    */
  std::vector<double> joint_angle = rtde_receive.getTargetQ();
  std::cout << "j1: " << joint_angle[0] << std::endl;
  std::cout << "j2  " << joint_angle[1] << std::endl;
  std::cout << "j3 "  << joint_angle[2] <<  std::endl;
  std::cout << "j4  " << joint_angle[3] << std::endl;
  std::cout << "j5  " << joint_angle[4] << std::endl;
  std::cout << "j6  " << joint_angle[5] << std::endl;
  if (rtde_receive.getDigitalOutState(7)){
    std::cout << "j1: " << joint_angle[0] << std::endl;
    std::cout << "j2  " << joint_angle[1] << std::endl;
    std::cout << "j3 "  << joint_angle[2] <<  std::endl;
    std::cout << "j4  " << joint_angle[3] << std::endl;
    std::cout << "j5  " << joint_angle[4] << std::endl;
    std::cout << "j6  " << joint_angle[5] << std::endl;
  }else
    std::cout << "Standard digital out (7) is LOW" << std::endl;

  if (rtde_receive.getDigitalOutState(16))
    std::cout << "Tool digital out (16) is HIGH" << std::endl;
  else
    std::cout << "Tool digital out (16) is LOW" << std::endl;

  rtde_io.setStandardDigitalOut(7, true);
  rtde_io.setToolDigitalOut(0, true);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  if (rtde_receive.getDigitalOutState(7))
    std::cout << "Standard digital out (7) is HIGH" << std::endl;
  else
    std::cout << "Standard digital out (7) is LOW" << std::endl;

  if (rtde_receive.getDigitalOutState(16))
    std::cout << "Tool digital out (16) is HIGH" << std::endl;
  else
    std::cout << "Tool digital out (16) is LOW" << std::endl;

  // How to set a analog output with a specified current ratio
  rtde_io.setAnalogOutputCurrent(1, 0.25);

  return 0;
}