/*
  overlay_image: Display the image from the camera superimpose with the point clouds
*/

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "cv_bridge/cv_bridge.h"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "pcl_ros/transforms.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "rclcpp/rclcpp.hpp"
#include "laser2cam_utils.h"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "image_transport/image_transport.hpp"
#include "opencv2/opencv.hpp"

class OverlayImage : public rclcpp::Node
{
public:
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::PointCloud2> exact_policy;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::PointCloud2> approximate_policy;
    OverlayImage(const rclcpp::NodeOptions &options);
    ~OverlayImage();

    rclcpp::QoS
    imageQoS() const
    {
        rclcpp::QoS qos(max_image_queue_size_);
        return qos;
    }
    rclcpp::QoS
    cloudQoS() const
    {
        rclcpp::QoS qos(max_cloud_queue_size_);
        return qos;
    }

private:
    void initializeParams();
    void callback(const sensor_msgs::msg::Image::ConstSharedPtr image, const sensor_msgs::msg::CameraInfo::ConstSharedPtr camera, sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud);
    void drawPoints(cv::Mat &image, std::vector<cv::Point3f> &pts, cv::Mat k, cv::Mat d);

    // Pubs Definition
    /** \brief The output Image publisher. */
    image_transport::Publisher img_pub_;

    // Subs Definition
    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> cinfo_sub_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub_;

    /** \brief Synchronized inputs.*/
    std::shared_ptr<message_filters::Synchronizer<exact_policy>> sync_inputs_e_;
    std::shared_ptr<message_filters::Synchronizer<approximate_policy>> sync_inputs_a_;

    int max_image_queue_size_ = 10;
    int max_cloud_queue_size_ = 10;

    // Camera info
    cv::Mat cameraMatrix_;
    cv::Mat distCoeffs_;

    tf2::Transform transform_;

    std::vector<double> rotation_{3};

    std::vector<double> translation_{3};
};