#ifndef HAND_HW_INTERFACE_H
#define HAND_HW_INTERFACE_H

#include <ros_control_boilerplate/generic_hw_interface.h>
#include <hand_control/dynamixelCmd.h>
#include <hand_control/dynamixelTelemetry.h>

namespace hand_ns
{
/** \brief Hardware interface for a robot */
class handHWInterface : public ros_control_boilerplate::GenericHWInterface
{
public:
  /**
   * \brief Constructor
   * \param nh - Node handle for topics.
   */
  handHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model = NULL);

  /** \brief Initialize the robot hardware interface */
  virtual void init();

  /** \brief Read the state from the robot hardware. */
  virtual void read(ros::Duration& elapsed_time);

  /** \brief Write the command to the robot hardware. */
  virtual void write(ros::Duration& elapsed_time);

  /** \breif Enforce limits for all values before writing */
  virtual void enforceLimits(ros::Duration& period);

protected:
    //define the subscriber and publisher
  ros::Subscriber telemetry_sub;
  void telemetryCallback(const hand_control::dynamixelTelemetry::ConstPtr& msg);
  std::vector<double> joint_position_prev_;


  ros::Publisher cmd_pub;



};  // class

}  // namespace ros_control_boilerplate

#endif
