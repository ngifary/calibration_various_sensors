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
#include "calibration/msg/circle_centroids.hpp"
#include "laser2cam_utils.h"

class StereoPattern : public rclcpp::Node
{
public:
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::msg::PointCloud2, pcl_msgs::msg::ModelCoefficients> ExactSync;

  StereoPattern();
  ~StereoPattern();

private:
  void initializeParams();
  void callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr camera_cloud,
                const pcl_msgs::msg::ModelCoefficients::ConstSharedPtr cam_plane_coeffs);
  rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> &parameters);

  double delta_width_circles_, delta_height_circles_;
  double circle_radius_, circle_threshold_;
  double plane_threshold_;
  double target_radius_tolerance_;

  // Pubs Definition
  rclcpp::Publisher<pcl_msgs::msg::PointIndices>::SharedPtr inliers_pub_;
  rclcpp::Publisher<pcl_msgs::msg::ModelCoefficients>::SharedPtr coeff_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr plane_edges_pub_, xy_pattern_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr centers_pub_;
  rclcpp::Publisher<calibration::msg::CircleCentroids>::SharedPtr final_pub_;

  // Subs Definition
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> camera_cloud_sub_;
  message_filters::Subscriber<pcl_msgs::msg::ModelCoefficients> cam_plane_coeffs_sub_;

  /** \brief Synchronized image and camera info.*/
  std::shared_ptr<message_filters::Synchronizer<ExactSync>> sync_;
  /** \brief The maximum queue size (default: 3). */
  int max_queue_size_ = 3;
  bool save_to_file_;
  std::ofstream savefile_;
  int iter_ = 0;
};