#include <string>

#include <geometry_msgs/TransformStamped.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "camera_link_rev.h"

cameraLinkRev::cameraLinkRev(ros::NodeHandle &nh)
: nh(nh)  // Copy a reference to a ROS node handle
, tfListener(tfBuffer)
{
  bool debug;

  nh.param("debug", debug, false);

  // Set a log verbosity level
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
      (debug) ? ros::console::levels::Debug : ros::console::levels::Info))
    ros::console::notifyLoggerLevelsChanged();

  if (debug)
    ROS_DEBUG("Debug mode enabled");

  std::string camera_prefix;
  nh.param("camera_prefix", camera_prefix, std::string("/camera"));
  
  nh.param("camera_tf_prefix", camera_tf_prefix, std::string("camera_"));

  // Create a subscriber for the driver state updates
  this->t265OdomSampleSub = this->nh.subscribe(camera_prefix + "/odom/sample", 1000,
    &cameraLinkRev::t265OdomSampleCallback, this);
}

void cameraLinkRev::run()
{
  ros::spin();
}

void cameraLinkRev::t265OdomSampleCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  ROS_DEBUG("Incoming IMU sample");

  geometry_msgs::TransformStamped transformMsg;
  tf2::Stamped<tf2::Transform> transformStamped;

  try
  {
    transformMsg = this->tfBuffer.lookupTransform(camera_tf_prefix + "pose_frame",
      camera_tf_prefix + "link", ros::Time(0));
  }
  catch (tf2::TransformException &e)
  {
    ROS_ERROR("Failed to get transform from pose to link: \"%s\"", e.what());
    ros::Duration(1.0).sleep();
    return;
  }
  tf2::convert(transformMsg, transformStamped);
  transformStamped.setData(transformStamped.inverse());
  tf2::convert(transformStamped, transformMsg);
  transformMsg.header.frame_id = camera_tf_prefix + "link_rev";
  transformMsg.child_frame_id = camera_tf_prefix + "pose_frame_rev";
  this->tfBroadcaster.sendTransform(transformMsg);

  try
  {
    transformMsg = this->tfBuffer.lookupTransform(camera_tf_prefix + "odom_frame",
      camera_tf_prefix + "pose_frame", ros::Time(0));
  }
  catch (tf2::TransformException &e)
  {
    ROS_ERROR("Failed to get transform from odom to pose: \"%s\"", e.what());
    ros::Duration(1.0).sleep();
    return;
  }
  tf2::convert(transformMsg, transformStamped);
  transformStamped.setData(transformStamped.inverse());
  tf2::convert(transformStamped, transformMsg);
  transformMsg.header.frame_id = camera_tf_prefix + "pose_frame_rev";
  transformMsg.child_frame_id = camera_tf_prefix + "odom_frame_rev";
  this->tfBroadcaster.sendTransform(transformMsg);
}

cameraLinkRev::~cameraLinkRev()
{
  try
  {
    
  }
  // Prevent exception to be thrown out of the destructor
  catch (...)
  {
  }
}
