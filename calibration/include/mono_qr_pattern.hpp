/*
  mono_qr_pattern: Find the circle centers in the color image by making use of
  the ArUco markers
*/

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_geometry/pinhole_camera_model.h"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/empty.hpp"
#include "calibration_interfaces/msg/cluster_centroids.hpp"
#include "laser2cam_utils.h"

#include "opencv2/aruco.hpp"
#include "opencv2/opencv.hpp"

class MonoQRPattern : public rclcpp::Node
{
public:
  MonoQRPattern();
  ~MonoQRPattern();

private:
  void initializeParams();
  cv::Point2f projectPointDist(cv::Point3f pt_cv, const cv::Mat intrinsics, const cv::Mat distCoeffs);
  Eigen::Vector3f mean(pcl::PointCloud<pcl::PointXYZ>::Ptr cumulative_cloud);
  Eigen::Matrix3f covariance(pcl::PointCloud<pcl::PointXYZ>::Ptr cumulative_cloud, Eigen::Vector3f means);
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg, const sensor_msgs::msg::CameraInfo::ConstSharedPtr left_info);
  rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> &parameters);
  // void warmup_callback(const std_msgs::msg::Empty::ConstSharedPtr msg);

  // Pubs Definition
  // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr qr_pub_, centers_cloud_pub_, cumulative_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr centers_pub_;
  // rclcpp::Publisher<calibration_interfaces::msg::ClusterCentroids>::SharedPtr clusters_pub_;

  // Subs Definition
  message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> cinfo_sub_;
  // rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr warmup_sub_;

  // pcl::PointCloud<pcl::PointXYZ>::Ptr cumulative_cloud_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  /** \brief Synchronized image and camera info.*/
  std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>>> sync_;
  /** \brief The maximum queue size (default: 3). */
  int max_queue_size_ = 3;
  // int frames_proc_ = 0, frames_used_ = 0;
  // bool WARMUP_DONE = false;
  // std::ofstream savefile_;

  // Node Parameters
  // Target parameters
  double marker_size_, delta_width_qr_center_, delta_height_qr_center_;
  double delta_width_circles_, delta_height_circles_;

  // Algorithm parameters
  unsigned min_detected_markers_;
  std::string config_file_;

  // // Aggregation parameters
  // bool skip_warmup_;
  // double cluster_tolerance_;
  // double min_cluster_factor_;

  // // Debug parameters
  // bool save_to_file_;
};