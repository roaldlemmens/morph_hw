#include "morph_hw.h"
#include "wheel_driver.h"
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <control_toolbox/pid.h>
#include <math.h>

namespace morph {

MORPH_HW::MORPH_HW(std::string right_wheel_port, std::string left_wheel_port, double right_wheel_ikv, double left_wheel_ikv, double tacho_pulses_per_revolution, int motor_poles, disp_pos_mode rotor_position_source, ros::NodeHandle nh) :
  _right_wheel_driver(right_wheel_port, nh, "right_wheel", rotor_position_source),
  _left_wheel_driver(left_wheel_port, nh, "left_wheel", rotor_position_source)
 {
    _pos[0]=0;
    _pos[1]=0;
    _vel[0]=0;
    _vel[1]=0;
    _cmd[0]=0;
    _cmd[1]=0;
    _eff[0]=0;
    _eff[1]=0;
    _right_wheel_ikv = right_wheel_ikv;
    _left_wheel_ikv = left_wheel_ikv;
    _tacho_pulses_per_revolution = tacho_pulses_per_revolution;
    _motor_poles = motor_poles;
    // Convert rad/s to RPM and multiply by number of poles to get ERPM
    _rad_per_sec_to_erpm_conversion_factor = (60/(2*M_PI))*_motor_poles;
    _tacho_conversion_factor = (2*M_PI)/_tacho_pulses_per_revolution;

    // connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle_a("left_wheel_joint", &_pos[0], &_vel[0], &_eff[0]);
    _jnt_state_interface.registerHandle(state_handle_a);

    hardware_interface::JointStateHandle state_handle_b("right_wheel_joint", &_pos[1], &_vel[1], &_eff[1]);
    _jnt_state_interface.registerHandle(state_handle_b);

    registerInterface(&_jnt_state_interface);

    // connect and register the joint velocity interfaces
    hardware_interface::JointHandle pos_handle_a(_jnt_state_interface.getHandle("left_wheel_joint"), &_cmd[0]);
    _jnt_vel_interface.registerHandle(pos_handle_a);

    hardware_interface::JointHandle pos_handle_b(_jnt_state_interface.getHandle("right_wheel_joint"), &_cmd[1]);
    _jnt_vel_interface.registerHandle(pos_handle_b);

    registerInterface(&_jnt_vel_interface);

    if (!nh.hasParam("/right_wheel/pid_parameters/p"))
      nh.setParam("/right_wheel/pid_parameters/p", 1.0);

    if (!_right_wheel_pid_controller.init(ros::NodeHandle(nh, "/right_wheel/pid_parameters")))
    {
        ROS_ERROR("Could not initialize right wheel PID controller");
    }
    if (!nh.hasParam("/left_wheel/pid_parameters/p"))
      nh.setParam("/left_wheel/pid_parameters/p", 1.0);

    if (!_left_wheel_pid_controller.init(ros::NodeHandle(nh, "/left_wheel/pid_parameters")))
    {
        ROS_ERROR("Could not initialize left wheel PID controller");
    }
 }

  void MORPH_HW::read(const ros::Time& time, const ros::Duration& period)
  {
    _pos[0] = _left_wheel_driver.getDisplacement()*_tacho_conversion_factor;
    _pos[1] = -_right_wheel_driver.getDisplacement()*_tacho_conversion_factor;
    _left_wheel_erpm =  _left_wheel_driver.getSpeed();
    _vel[0] = _left_wheel_erpm /_rad_per_sec_to_erpm_conversion_factor;
    _right_wheel_erpm = _right_wheel_driver.getSpeed();
    _vel[1] = _right_wheel_erpm/_rad_per_sec_to_erpm_conversion_factor;

    double encoderDisplacementLeft = _left_wheel_driver.getEncoderDisplacement();
    double encoderDisplacementRight = _right_wheel_driver.getEncoderDisplacement();

    ROS_DEBUG("Left wheel: position - %f - encoder %f - velocity %f - commands %f - effort %f", _pos[0], encoderDisplacementLeft, _vel[0], _cmd[0], _eff[0]);
    ROS_DEBUG("Right wheel: position - %f - encoder %f - velocity %f - commands %f - effort %f", _pos[1], encoderDisplacementRight, _vel[1], _cmd[1], _eff[1]);
  }

  void MORPH_HW::write(const ros::Time& time, const ros::Duration& period)
  {
    ros::Time currentTime = ros::Time::now();
    ros::Duration duration = currentTime - _previous_update_time;
    double left_voltage_in = _left_wheel_driver.getVoltageIn();
    double left_request_dutyCycle = 0.0;

    if (_cmd[0] != 0.0)
    {
      double requestedERPM = _rad_per_sec_to_erpm_conversion_factor * _cmd[0];
      double error = requestedERPM - _left_wheel_erpm;
      double command = _left_wheel_pid_controller.computeCommand(error, duration);

      left_request_dutyCycle = command / (left_voltage_in * _left_wheel_ikv * _motor_poles * 2);
      ROS_DEBUG("Requested ERPM left: %f - actual %f - command %f - dutycycle %f ", requestedERPM, _left_wheel_erpm, command, left_request_dutyCycle);
      _left_wheel_driver.setDutyCycle(left_request_dutyCycle);
    }
    else
    {
      _left_wheel_driver.releaseMotor();
    }

    double right_voltage_in = _right_wheel_driver.getVoltageIn();
    double right_request_dutyCycle = 0.0;
    if (_cmd[1] != 0.0)
    {
      double requestedERPM = -_rad_per_sec_to_erpm_conversion_factor * _cmd[1];
      double error = requestedERPM - _right_wheel_erpm;
      double command = _right_wheel_pid_controller.computeCommand(error, duration);

      right_request_dutyCycle = command / (right_voltage_in * _right_wheel_ikv * _motor_poles * 2);
      ROS_DEBUG("Requested ERPM right: %f - actual %f - command %f - dutycycle %f", requestedERPM, _right_wheel_erpm, command, right_request_dutyCycle);
      _right_wheel_driver.setDutyCycle(right_request_dutyCycle);
    }
    else
    {
      _right_wheel_driver.releaseMotor();
    }

    _previous_update_time = currentTime;
  }
}
