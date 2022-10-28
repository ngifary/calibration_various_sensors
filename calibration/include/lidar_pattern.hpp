#define PCL_NO_PRECOMPILE

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "pcl/common/eigen.h"
#include "pcl/common/transforms.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/passthrough.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl_conversions/pcl_conversions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/empty.hpp"
#include "calibration_interfaces/msg/cluster_centroids.hpp"
#include "laser2cam_utils.h"
#include <queue>

class LidarPattern : public rclcpp::Node
{
public:
  LidarPattern();
  ~LidarPattern();

private:
  void initializeParams();
  void callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr laser_cloud);
  rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> &parameters);
  void warmup_callback(const std_msgs::msg::Empty::ConstSharedPtr msg);
  struct classcomp
  {
    bool operator()(const pcl::PointXYZ &a, const pcl::PointXYZ &b) const
    {
      return a.y > b.y;
    }
  };

  // Pubs Definition
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cumulative_pub_, range_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pattern_pub_, rotated_pattern_pub_;
  rclcpp::Publisher<pcl_msgs::msg::ModelCoefficients>::SharedPtr coeff_pub_;
  rclcpp::Publisher<calibration_interfaces::msg::ClusterCentroids>::SharedPtr centers_pub_;

  // Subs Definition
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr warmup_sub;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cumulative_cloud;
  int clouds_proc_ = 0, clouds_used_ = 0;
  bool WARMUP_DONE = false;
  std::ofstream savefile;

  // Node Parameters
  // Cloud prefilter parameters
  double passthrough_radius_min_, passthrough_radius_max_;

  // Hardware parameters
  double plane_threshold_; // depend on linear resolution
  double gap_threshold_;   // depend on azimuth resolution (horizontal angular resolution)
  double line_threshold_;  // depend on polar resolution (vertical angular resolution)

  // Target parameters
  double circle_radius_, delta_width_circles_, delta_height_circles_; // constant
  double circle_threshold_, target_radius_tolerance_;
  Eigen::Vector3f axis_;   // Axis parallel to target
  double angle_threshold_; // Tilt angle w.r.t axis_
  int min_centers_found_;

  // Aggregation parameters
  bool skip_warmup_;
  double cluster_tolerance_;
  double min_cluster_factor_;

  // Debug parameters
  bool save_to_file_;
};