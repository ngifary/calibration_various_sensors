/*
  stereo_pattern: Find the circle centers in the stereo cloud
*/

#define TARGET_NUM_CIRCLES 4

// #include "dynamic_reconfigure/server.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/exact_time.h"
#include "pcl/common/transforms.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/sample_consensus/sac_model_plane.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl_conversions/pcl_conversions.hpp"
#include "pcl_msgs/msg/model_coefficients.hpp"
#include "pcl_msgs/msg/point_indices.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/empty.hpp"
#include "calibration_interfaces/msg/cluster_centroids.hpp"
#include "laser2cam_utils.h"

class StereoPattern : public rclcpp::Node
{
public:
  StereoPattern();
  ~StereoPattern();

private:
  void initializeParams();
  void callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr camera_cloud,
                const pcl_msgs::msg::ModelCoefficients::ConstSharedPtr cam_plane_coeffs);
  rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> &parameters);
  void warmup_callback(const std_msgs::msg::Empty::ConstSharedPtr msg);

  int images_proc_ = 0, images_used_ = 0;

  double delta_width_circles_, delta_height_circles_;
  double circle_radius_, circle_threshold_;
  double plane_distance_inliers_;
  double target_radius_tolerance_;
  double cluster_tolerance_;
  double min_cluster_factor_;
  unsigned min_centers_found_;
  bool WARMUP_DONE = false;
  bool skip_warmup_;
  bool save_to_file_;
  std::ofstream savefile;

  // Pubs Definition
  rclcpp::Publisher<pcl_msgs::msg::PointIndices>::SharedPtr inliers_pub;
  rclcpp::Publisher<pcl_msgs::msg::ModelCoefficients>::SharedPtr coeff_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr plane_edges_pub, xy_pattern_pub, cumulative_pub;
  rclcpp::Publisher<calibration_interfaces::msg::ClusterCentroids>::SharedPtr final_pub;

  // Subs Definition
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> camera_cloud_sub_;
  message_filters::Subscriber<pcl_msgs::msg::ModelCoefficients> cam_plane_coeffs_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr warmup_sub;

  /** \brief Synchronized image and camera info.*/
  std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ExactTime<sensor_msgs::msg::PointCloud2, pcl_msgs::msg::ModelCoefficients>>> sync_;
  /** \brief The maximum queue size (default: 3). */
  int max_queue_size_ = 3;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cumulative_cloud;

  std_msgs::msg::Header header_;
};