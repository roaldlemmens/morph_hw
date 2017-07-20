#ifndef MORPH_HW_H
#define MORPH_HW_H
#include "morph_hw.h"
#include "wheel_driver.h"
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

namespace morph
{

class MORPH_HW : public hardware_interface::RobotHW
{
public:
  MORPH_HW(std::string right_wheel_port, std::string left_wheel_port, double right_wheel_correction_factor, double left_wheel_correction_factor, double tacho_pulses_per_revolution, int motor_poles, disp_pos_mode rotor_position_source, ros::NodeHandle nh);
  void read(const ros::Time& time, const ros::Duration& period);
  void write(const ros::Time& time, const ros::Duration& period);

private:
  wheel_driver _right_wheel_driver;
  wheel_driver _left_wheel_driver;
  double _right_wheel_ikv;
  double _left_wheel_ikv;
  double _tacho_pulses_per_revolution;
  double _tacho_conversion_factor;
  double _rad_per_sec_to_erpm_conversion_factor;
  int _motor_poles;
  int _maxSpeed;
  hardware_interface::JointStateInterface _jnt_state_interface;
  hardware_interface::VelocityJointInterface _jnt_vel_interface;
  double _cmd[2];
  double _pos[2];
  double _vel[2];
  double _eff[2];
};

}

#endif // MORPH_HW_H
