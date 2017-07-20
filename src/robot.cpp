#include "morph_hw.h"
#include <iostream>
#include "controller_manager/controller_manager.h"
#include <hardware_interface/robot_hw.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "morph_robot");
  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  std::string right_wheel_port;
  std::string left_wheel_port;
  double right_wheel_ikv;
  double left_wheel_ikv;
  double tacho_pulses_per_revolution;
  int motor_poles;

  ros::Rate r(100); // 100Hz
  if (!private_nh.getParam("right_wheel_port", right_wheel_port)) {
    ROS_FATAL("VESC communication right_wheel_port parameter required.");
    ros::shutdown();
    return -1;
  }

  if (!private_nh.getParam("left_wheel_port", left_wheel_port)) {
    ROS_FATAL("VESC communication left_wheel_port parameter required.");
    ros::shutdown();
    return -1;
  }

  if (!private_nh.getParam("right_wheel_ikv", right_wheel_ikv)) {
    ROS_FATAL("Parameter right_wheel_ikv required.");
    ros::shutdown();
    return -1;
  }

  if (!private_nh.getParam("left_wheel_ikv", left_wheel_ikv)) {
    ROS_FATAL("Parameter left_wheel_ikv required.");
    ros::shutdown();
    return -1;
  }

  if (!private_nh.getParam("tacho_pulses_per_revolution", tacho_pulses_per_revolution)) {
    ROS_FATAL("Parameter tacho_pulses_per_revolution required.");
    ros::shutdown();
    return -1;
  }

  if (!private_nh.getParam("motor_poles", motor_poles)) {
    ROS_FATAL("Parameter motor_poles required.");
    ros::shutdown();
    return -1;
  }

  // decide whether or not we want to publish rotor positions
  // this is enabled with by sending a "set detect" packet with a mode
  // "encoder" is probably the most useful
  std::string rotor_position_source;
  disp_pos_mode mode = DISP_POS_MODE_NONE;
  if (private_nh.getParam("rotor_position_source", rotor_position_source)) {
    if (rotor_position_source.compare("inductance") == 0) {
      mode = DISP_POS_MODE_INDUCTANCE;
    }  else if (rotor_position_source.compare("observer") == 0) {
      mode = DISP_POS_MODE_OBSERVER;
    } else if (rotor_position_source.compare("encoder") == 0) {
      mode = DISP_POS_MODE_ENCODER;
    } else if (rotor_position_source.compare("pid_pos") == 0) {
      mode = DISP_POS_MODE_PID_POS;
    } else if (rotor_position_source.compare("pid_pos_error") == 0) {
      mode = DISP_POS_MODE_PID_POS_ERROR;
    } else if (rotor_position_source.compare("encoder_observer_error") == 0) {
      mode = DISP_POS_MODE_ENCODER_OBSERVER_ERROR;
    } else if (rotor_position_source.compare("none") != 0) {
      ROS_WARN("Invalid display mode parameter, defaulting to none");
    }
  }

  morph::MORPH_HW robot(right_wheel_port, left_wheel_port, right_wheel_ikv, left_wheel_ikv, tacho_pulses_per_revolution, motor_poles, mode, nh);
  controller_manager::ControllerManager cm(&robot);
  ROS_INFO_STREAM_NAMED("hardware_interface","Starting loop");
  while (ros::ok())
  {
     robot.read(ros::Time::now(),r.cycleTime());
     cm.update(ros::Time::now(), r.cycleTime());
     robot.write(ros::Time::now(), r.cycleTime());
     r.sleep();
  }
  ROS_INFO_STREAM_NAMED("hardware_interface","Shutting down");

  return 0;
}
