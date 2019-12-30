#include <nodelet/nodelet.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/LaserScan.h>
#include <pluginlib/class_list_macros.h>
#include <memory>

namespace coex_nano {

class LaserscanTransform : public nodelet::Nodelet {

private:
    std::string target_frame_;
    ros::Subscriber laserscan_sub_;
    ros::Publisher transformed_pub_;

    std::unique_ptr<tf2_ros::Buffer> transform_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> transform_listener_;

public:
    void onInit() final {
        ros::NodeHandle& nh = getNodeHandle();
        ros::NodeHandle& nh_priv = getPrivateNodeHandle();
        transform_buffer_ = std::make_unique<tf2_ros::Buffer>(ros::Duration(1.0));
        transform_listener_ = std::make_unique<tf2_ros::TransformListener>(*transform_buffer_.get());

        transformed_pub_ = nh_priv.advertise<sensor_msgs::LaserScan>("transformed_laserscan", 1);
        target_frame_ = nh_priv.param<std::string>("target_frame", "laserscan_frd");
        bool do_stupid_flip_instead_of_proper_transform = nh_priv.param<bool>("flip_only", false);
        if (do_stupid_flip_instead_of_proper_transform) {
            laserscan_sub_ = nh.subscribe("/laserscan", 1, &LaserscanTransform::ls_flip, this);
        } else {
            laserscan_sub_ = nh.subscribe("/laserscan", 1, &LaserscanTransform::ls_callback, this);
        }
        NODELET_INFO("Laserscan transformer up");
    }

    void ls_flip(const sensor_msgs::LaserScanConstPtr& src) {
        sensor_msgs::LaserScanPtr result = boost::make_shared<sensor_msgs::LaserScan>();
        result->header.frame_id = target_frame_;
        result->header.stamp = src->header.stamp;
        result->angle_min = src->angle_min;
        result->angle_max = src->angle_max;
        result->ranges.reserve(src->ranges.size());
        result->range_min = src->range_min;
        result->range_max = src->range_max;
        result->angle_increment = src->angle_increment;

        for(auto it = src->ranges.rbegin(); it != src->ranges.rend(); ++it)
        {
            result->ranges.push_back(*it);
        }
        transformed_pub_.publish(result);
    }

    void ls_callback(const sensor_msgs::LaserScanConstPtr& src) {
        geometry_msgs::TransformStamped transform;
        try {
            transform = transform_buffer_->lookupTransform(target_frame_, src->header.frame_id, src->header.stamp, ros::Duration(0.02));
        } catch(const tf2::TransformException &e) {
            NODELET_WARN_THROTTLE(1.0, "Could not get transform from %s to %s: %s", src->header.frame_id.c_str(), target_frame_.c_str(), e.what());
            return;
        }
        sensor_msgs::LaserScanPtr result = boost::make_shared<sensor_msgs::LaserScan>();
        result->header.frame_id = target_frame_;
        result->header.stamp = src->header.stamp;
        result->angle_min = src->angle_max;
        result->angle_max = src->angle_min;
        result->ranges.reserve(src->ranges.size());
        // FIXME: range_min, range_max may have changed
        result->range_min = src->range_min;
        result->range_max = src->range_max;

        for(size_t i = 0; i < src->ranges.size(); ++i) {
            float angle = src->angle_min + i * src->angle_increment;
            geometry_msgs::Point point, point_transformed;
            point.x = src->ranges[i] * cos(angle);
            point.y = src->ranges[i] * sin(angle);
            point.z = 0;
            tf2::doTransform(point, point_transformed, transform);
            double transformed_range = std::sqrt(point_transformed.x * point_transformed.x +
                                                 point_transformed.y * point_transformed.y +
                                                 point_transformed.z * point_transformed.z);
            // Calculate min/max angle in new frame
            if (i == 0 || i == (src->ranges.size() - 1)) {
                // We use a unit point since the one in our message might be a NaN
                geometry_msgs::Point unit_pt, unit_pt_transformed;
                unit_pt.x = cos(angle);
                unit_pt.y = sin(angle);
                unit_pt.z = 0;
                tf2::doTransform(unit_pt, unit_pt_transformed, transform);
                float angle = std::atan2(unit_pt_transformed.y, unit_pt_transformed.x);
                if (i == 0) {
                    result->angle_min = angle;
                } else {
                    result->angle_max = angle;
                }
            }
            result->ranges.push_back(transformed_range);
        }
        // FIXME: angle_increment might be inverted?
        result->angle_increment = (result->angle_max - result->angle_min) / (result->ranges.size() - 1);
        transformed_pub_.publish(result);
    }
};

}

PLUGINLIB_EXPORT_CLASS(coex_nano::LaserscanTransform, nodelet::Nodelet);
