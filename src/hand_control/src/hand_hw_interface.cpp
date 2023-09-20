
#include <hand_control/hand_hw_interface.h>


namespace hand_ns
{
handHWInterface::handHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model)
  : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
{
    // Initialize ROS interfaces
  telemetry_sub = nh.subscribe("/hand_ops/dynamixelTelemetry", 1, &handHWInterface::telemetryCallback, this);

  cmd_pub=nh.advertise<hand_control::dynamixelCmd>("/hand_ops/dynamixelCmd", 1);

  ROS_INFO("handHWInterface Constructed.");

}

void handHWInterface::telemetryCallback(const hand_control::dynamixelTelemetry::ConstPtr& msg){

//   float32 pulses
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
    joint_position_[i]=msg->angle[i];
    joint_velocity_[i]=msg->velocity[i];
  }

}

void handHWInterface::init()
{
  // Call parent class version of this function
  GenericHWInterface::init();

  ROS_INFO("handHWInterface Ready.");
}

void handHWInterface::read(ros::Duration& elapsed_time)
{
  // No need to read since our write() command populates our state for us
  // ros::spinOnce();
}

void handHWInterface::write(ros::Duration& elapsed_time)
{
  // Safety
  //enforceLimits(elapsed_time);

  // NOTE: the following is a "simuation" example so that this boilerplate can be run without being
  // connected to hardware
  // When converting to your robot, remove the built-in PID loop and instead let the higher leverl
  // ros_control controllers take
  // care of PID loops for you. This P-controller is only intended to mimic the delay in real
  // hardware, somewhat like a simualator
  static hand_control::dynamixelCmd dyn_cmd;
  // these are nothing but the trajectories of position and velocity given by the controller
  
  for(int i=0; i<num_joints_; i++){
    dyn_cmd.angle[i]=joint_position_command_[i];
    dyn_cmd.velocity[i]=joint_velocity_command_[i];
    // dyn_cmd.current[i]=joint_effort_command_[i];
  }
  cmd_pub.publish(dyn_cmd);


}

void handHWInterface::enforceLimits(ros::Duration& period)
{
  // Enforces position and velocity
  // pos_jnt_sat_interface_.enforceLimits(period);
}



}  // namespace hand_ns
