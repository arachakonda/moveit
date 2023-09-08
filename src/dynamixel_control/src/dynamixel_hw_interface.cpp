
#include <dynamixel_control/dynamixel_hw_interface.h>


namespace dynamixel_ns
{
dynamixelHWInterface::dynamixelHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model)
  : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
{
  // Initialize ROS interfaces
  telemetry_sub = nh.subscribe("/dynamixelTelemetry", 1, &dynamixelHWInterface::telemetryCallback, this);

  cmd_pub=nh.advertise<dynamixel_control::dynamixelCmd>("/dynamixelCmd", 1);

  ROS_INFO("dynamixelHWInterface Constructed.");
}
void dynamixelHWInterface::telemetryCallback(const dynamixel_control::dynamixelTelemetry::ConstPtr& msg){

//   float32 angle
// float32 velocity
// float32 current
// time startSyncTime
// uint32 isrTicks
// uint8 bufferHealth

  // // States
  // std::vector<double> joint_position_;
  // std::vector<double> joint_velocity_;
  // std::vector<double> joint_effort_;



  for(int i=0; i<num_joints_; i++){
    joint_position_[i]=msg->angle;
    joint_velocity_[i]=msg->velocity;
  }



}
void dynamixelHWInterface::init()
{
  // Call parent class version of this function
  GenericHWInterface::init();

  ROS_INFO("dynamixelHWInterface Ready.");
}

void dynamixelHWInterface::read(ros::Duration& elapsed_time)
{
  // No need to read since our write() command populates our state for us

  ros::spinOnce();

}

void dynamixelHWInterface::write(ros::Duration& elapsed_time)
{
  // Safety
  //enforceLimits(elapsed_time);

  // NOTE: the following is a "simuation" example so that this boilerplate can be run without being
  // connected to hardware
  // When converting to your robot, remove the built-in PID loop and instead let the higher leverl
  // ros_control controllers take
  // care of PID loops for you. This P-controller is only intended to mimic the delay in real
  // hardware, somewhat like a simualator

  //   float32 angle
  // float32 velocity
  // float32 current
  // uint32 msg_count

    //   // Commands
  // std::vector<double> joint_position_command_;
  // std::vector<double> joint_velocity_command_;
  // std::vector<double> joint_effort_command_;
  static dynamixel_control::dynamixelCmd dyn_cmd;

  for(int i=0; i<num_joints_; i++){
    dyn_cmd.angle=joint_position_command_[i];
    dyn_cmd.velocity=joint_velocity_command_[i];
  }
  cmd_pub.publish(cmd_msg);



}

void dynamixelHWInterface::enforceLimits(ros::Duration& period)
{
  // Enforces position and velocity
  // pos_jnt_sat_interface_.enforceLimits(period);
}



}  // namespace dynamixel_ns
