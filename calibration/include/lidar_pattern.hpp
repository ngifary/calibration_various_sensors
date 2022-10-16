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
  /* data */
  // Pubs Definition
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cumulative_pub, range_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pattern_pub, rotated_pattern_pub;
  rclcpp::Publisher<pcl_msgs::msg::ModelCoefficients>::SharedPtr coeff_pub;
  rclcpp::Publisher<calibration_interfaces::msg::ClusterCentroids>::SharedPtr centers_pub;

  // Subs Definition
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr warmup_sub;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cumulative_cloud;

  // Node Dynamic parameters
  double threshold_;
  double passthrough_radius_min_, passthrough_radius_max_, circle_radius_,
      centroid_distance_min_, centroid_distance_max_;
  double delta_width_circles_, delta_height_circles_;
  Eigen::Vector3f axis_;
  double angle_threshold_;

  // Non-Dynamic
  int channels_count_;
  double cluster_tolerance_;
  int clouds_proc_ = 0, clouds_used_ = 0;
  int min_centers_found_;
  double plane_threshold_;
  double gradient_threshold_;
  double plane_distance_inliers_;
  double circle_threshold_;
  double target_radius_tolerance_;
  double min_cluster_factor_;
  bool skip_warmup_;
  bool save_to_file_;
  std::ofstream savefile;

  bool WARMUP_DONE = false;
};