/*
  plane: Find a plane model in the cloud using RANSAC
*/

#include <memory.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl/filters/filter.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl_conversions/pcl_conversions.hpp"
#include "pcl_msgs/msg/model_coefficients.hpp"
#include "pcl_msgs/msg/point_indices.hpp"

class SACSegmentator : public rclcpp::Node
{
public:
    SACSegmentator();
    ~SACSegmentator();

private:
    void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &input);
    rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> &parameters);
    rclcpp::Publisher<pcl_msgs::msg::PointIndices>::SharedPtr inliers_pub_;
    rclcpp::Publisher<pcl_msgs::msg::ModelCoefficients>::SharedPtr coeff_pub_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

    int sac_segmentation_type_;
    double eps_angle_;

    Eigen::Vector3f Axis_;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr ret_;
    double threshold_;
};
