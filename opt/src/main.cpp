#include <ros/ros.h>
#include <ros/console.h>

#include "camera_link_rev.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera_link_rev");

  ros::NodeHandle privateNodeHandle("~");

  ROS_INFO("Node start");

  cameraLinkRev node(privateNodeHandle);
  node.run();

  ROS_INFO("Node stop");

  return 0;
}
