#ifndef CAMERA_LINK_REV_NODE
#define CAMERA_LINK_REV_NODE

#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>

class cameraLinkRev
{
  private:
    std::string camera_tf_prefix;

    ros::NodeHandle &nh;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    tf2_ros::TransformBroadcaster tfBroadcaster;

    ros::Subscriber t265OdomSampleSub;  // T265 sample subscriber

    /* T265 odometry sample callback
    * msg - a new odometry sample.
    */
    void t265OdomSampleCallback(const nav_msgs::Odometry::ConstPtr& msg);

  public:
    /* Camera link reverse node constructor
    * nh - ROS node handle.
    */
    cameraLinkRev(ros::NodeHandle &nh);

    void run();

    ~cameraLinkRev();
};

#endif  // CAMERA_LINK_REV_NODE
