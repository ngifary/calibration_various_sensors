#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "pcl/registration/transformation_estimation_svd.h"
#include "pcl_conversions/pcl_conversions.hpp"
#include "pcl_ros/transforms.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/transform_datatypes.h"
#include "tf2/impl/utils.h"
#include "tinyxml2.h"
#include "laser2cam_utils.h"
#include "opencv2/opencv.hpp"
#include "calibration/msg/circle_centroids.hpp"

#include <chrono>
#include <cinttypes>
#include <limits>

#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/image.hpp"

class Calibration : public rclcpp::Node
{

public:
    typedef Eigen::Matrix<double, 12, 12> Matrix12d;
    typedef Eigen::Matrix<double, 12, 1> Vector12d;

    typedef message_filters::sync_policies::ExactTime<calibration::msg::CircleCentroids, calibration::msg::CircleCentroids> ExactSync;
    typedef message_filters::sync_policies::ApproximateTime<calibration::msg::CircleCentroids, calibration::msg::CircleCentroids> ApproxSync;

    Calibration(const rclcpp::NodeOptions &options);
    ~Calibration();

    rclcpp::QoS
    cloudQoS() const
    {
        rclcpp::QoS qos(max_queue_size_);
        return qos;
    }

private:
    void initializeParams();
    void callback(const calibration::msg::CircleCentroids::ConstSharedPtr sensor1_centroids, const calibration::msg::CircleCentroids::ConstSharedPtr sensor2_centroids);
    Eigen::Affine3f calibrateExtrinsics(std::vector<pcl::PointXYZ> &sensor1_pcl, std::vector<pcl::PointXYZ> &sensor2_pcl);

    // Pubs Declaration
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr colour_sensor2_pub_, colour_sensor1_pub_;

    // Subs Declaration
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    message_filters::Subscriber<calibration::msg::CircleCentroids> sensor1_sub_;
    message_filters::Subscriber<calibration::msg::CircleCentroids> sensor2_sub_;

    /** \brief Synchronized inputs.*/
    std::shared_ptr<message_filters::Synchronizer<ExactSync>> sync_inputs_e_;
    std::shared_ptr<message_filters::Synchronizer<ApproxSync>> sync_inputs_a_;

    /** \brief TF listener object. */
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::string sensor1_frame_id_ = "";
    std::string sensor2_frame_id_ = "";

    int max_queue_size_ = 10;

    // Node Parameters Declaration
    std::string csv_name_;
    bool approximate_time_ = true;
    bool publish_tf_;
    bool sensor1_is_cam_, sensor2_is_cam_;
};
