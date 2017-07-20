#include <math.h>
#include "wheel_driver.h"
#include "vesc_driver/vesc_driver.h"
#include <vesc_msgs/VescStateStamped.h>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

wheel_driver::wheel_driver(std::string port, ros::NodeHandle nh, std::string name, disp_pos_mode rotor_position_mode) :
      vesc_(std::string(),
            boost::bind(&wheel_driver::vescPacketCallback, this, _1),
            boost::bind(&wheel_driver::vescErrorCallback, this, _1)),
      driver_mode_(MODE_INITIALIZING), fw_version_major_(-1), fw_version_minor_(-1)
{
    name_ = name;
    encoderDisplacementPreviousValue = NAN;
    // attempt to connect to the serial port
    try {
      vesc_.connect(port);
    }
    catch (SerialException e) {
      ROS_FATAL("Failed to connect to the VESC on port %s, %s.", port.c_str(), e.what());
      ros::shutdown();
      return;
    }

    // create vesc state (telemetry) publisher
    state_pub_ = nh.advertise<vesc_msgs::VescStateStamped>(name + "/sensors/core", 10);

    // create a 50Hz timer, used for state machine & polling VESC telemetry
    timer_ = nh.createTimer(ros::Duration(1.0/50.0), &wheel_driver::timerCallback, this);

    if (rotor_position_mode != DISP_POS_MODE_NONE) {
      // create rotor position publisher
      rotor_position_pub_ = nh.advertise<std_msgs::Float32>(name + "/sensors/rotor_position", 10);
      ROS_INFO("Enabling rotor position publisher for %s", name_.c_str());
    }
    vesc_.setDetect(rotor_position_mode);
}

void wheel_driver::setDutyCycle(double dutyCycle)
{
  if (dutyCycle >= -1.0 && dutyCycle <= 1.0) {
      ROS_DEBUG("Set duty-cycle %s : %f", name_.c_str(), dutyCycle);
      vesc_.setDutyCycle(dutyCycle);
  }
  else {
      ROS_ERROR("Dutycycle out-of-range %s : %f", name_.c_str(), dutyCycle);
  }
}

void wheel_driver::releaseMotor()
{
  ROS_DEBUG("Releasing motor %s", name_.c_str());
  vesc_.setCurrent(0.0);
}

void wheel_driver::timerCallback(const ros::TimerEvent& event)
{
  // VESC interface should not unexpectedly disconnect, but test for it anyway
  if (!vesc_.isConnected()) {
    ROS_FATAL("Unexpectedly disconnected from serial port.");
    // timer_.stop();
    ros::shutdown();
    return;
  }

  /*
   * Driver state machine, modes:
   *  INITIALIZING - request and wait for vesc version
   *  OPERATING - receiving commands from subscriber topics
   */
  if (driver_mode_ == MODE_INITIALIZING) {
    // request version number, return packet will update the internal version numbers
    vesc_.requestFWVersion();
    if (fw_version_major_ >= 0 && fw_version_minor_ >= 0) {
      ROS_INFO("Connected to VESC with firmware version %d.%d",
               fw_version_major_, fw_version_minor_);
      driver_mode_ = MODE_OPERATING;
    }
  }
  else if (driver_mode_ == MODE_OPERATING) {
    // poll for vesc state (telemetry)
    vesc_.requestState();
  }
  else {
    // unknown mode, how did that happen?
    assert(false && "unknown driver mode");
  }
}

void wheel_driver::vescPacketCallback(const boost::shared_ptr<VescPacket const>& packet)
{
  if (packet->name() == "Values") {
    boost::shared_ptr<VescPacketValues const> values =
    boost::dynamic_pointer_cast<VescPacketValues const>(packet);
    {
      boost::mutex::scoped_lock lock(mutex_);
      displacement = values->tachometer();
      speed = values->rpm();
      voltageIn = values->v_in();
    }

    vesc_msgs::VescStateStamped::Ptr state_msg(new vesc_msgs::VescStateStamped);
    state_msg->header.stamp = ros::Time::now();
    state_msg->state.voltage_input = values->v_in();
    state_msg->state.temperature_pcb = values->temp_pcb();
    state_msg->state.current_motor = values->current_motor();
    state_msg->state.current_input = values->current_in();
    state_msg->state.speed = values->rpm();
    state_msg->state.duty_cycle = values->duty_now();
    state_msg->state.charge_drawn = values->amp_hours();
    state_msg->state.charge_regen = values->amp_hours_charged();
    state_msg->state.energy_drawn = values->watt_hours();
    state_msg->state.energy_regen = values->watt_hours_charged();
    state_msg->state.displacement = values->tachometer();
    state_msg->state.distance_traveled = values->tachometer_abs();
    state_msg->state.fault_code = values->fault_code();
    state_pub_.publish(state_msg);
  }
  else if (packet->name() == "FWVersion") {
    boost::shared_ptr<VescPacketFWVersion const> fw_version =
      boost::dynamic_pointer_cast<VescPacketFWVersion const>(packet);
    // todo: might need lock here
    fw_version_major_ = fw_version->fwMajor();
    fw_version_minor_ = fw_version->fwMinor();
  }
  else if (packet->name() == "RotorPosition" && rotor_position_pub_) {
    //pubish the value of the rotor position
    boost::shared_ptr<VescPacketRotorPosition const> position =
      boost::dynamic_pointer_cast<VescPacketRotorPosition const>(packet);

    std_msgs::Float32::Ptr rotor_position_msg(new std_msgs::Float32);
    // we get degrees from the VESC, but radians are the standard in ROS
    double currentPosition = position->position() * M_PI / 180.0;
    // Subtract initial encoder position for first reading or add position to every other reading
    if (isnan(encoderDisplacementPreviousValue))
      encoderDisplacementPreviousValue = currentPosition;

    // Only calculate encoderDisplacement when the speed is above 5 ERPM, because
    // stationary value can fluctuate a few degrees and can cause issues at 360 / 0 (2pi / 0 ) point
    const int MIN_SPEED_ERPM = 5; // TODO: Parameter for threshold
    if (speed > MIN_SPEED_ERPM)
    {
      // Forward motion delta calculation
      double encoderDelta = currentPosition - encoderDisplacementPreviousValue;

      // If a lap has been made, correct the delta
      if (currentPosition < 1 && encoderDisplacementPreviousValue > 5)
        encoderDelta += 2 * M_PI;
      encoderDisplacement = encoderDisplacement + encoderDelta;

      ROS_WARN("Forward %s position: %f delta: %f displacement: %f", name_.c_str(), currentPosition, encoderDelta, encoderDisplacement);
      encoderDisplacementPreviousValue = currentPosition;
      rotor_position_msg->data = currentPosition;
      rotor_position_pub_.publish(rotor_position_msg);
    }
    else if(speed < -MIN_SPEED_ERPM)
    {
      // Backwards motion delta calculation
      double encoderDelta = encoderDisplacementPreviousValue - currentPosition;

      // If a lap has been made, correct the delta
      if (currentPosition > 5 && encoderDisplacementPreviousValue < 1)
        encoderDelta += 2 * M_PI;
      encoderDisplacement = encoderDisplacement + encoderDelta;

      ROS_WARN("Backward %s position: %f delta: %f displacement: %f", name_.c_str(), currentPosition, encoderDelta, encoderDisplacement);
      encoderDisplacementPreviousValue = currentPosition;
      rotor_position_msg->data = currentPosition;
      rotor_position_pub_.publish(rotor_position_msg);
    }
  }
}

double  wheel_driver::getSpeed()
{
  {
    boost::mutex::scoped_lock lock(mutex_);
    return speed;
  }
}

double  wheel_driver::getVoltageIn()
{
  {
    boost::mutex::scoped_lock lock(mutex_);
    return voltageIn;
  }
}

double wheel_driver::getDisplacement()
{
  {
    boost::mutex::scoped_lock lock(mutex_);
    return displacement;
  }
}

double wheel_driver::getEncoderDisplacement()
{
  {
    boost::mutex::scoped_lock lock(mutex_);
    return encoderDisplacement;
  }
}

void wheel_driver::vescErrorCallback(const std::string& error)
{
  ROS_ERROR("%s", error.c_str());
}
