// Convert depth map to OBSTACLE_DISTANCE MAVLink message.
// OBSTACLE_DISTANCE message reference: https://mavlink.io/en/messages/common.html#OBSTACLE_DISTANCE
// NOTE: Must convert to MAV_FRAME_BODY_FRD first.

#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>

namespace coex_nano {

class DepthToDistance : public nodelet::Nodelet {
private:



    void onInit() final {

    }

};

}

PLUGINLIB_EXPORT_CLASS(coex_nano::DepthToDistance, nodelet::Nodelet);