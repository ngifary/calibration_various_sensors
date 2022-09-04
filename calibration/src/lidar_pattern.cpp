/*
  velo2cam_calibration - Automatic calibration algorithm for extrinsic
  parameters of a stereo camera and a velodyne Copyright (C) 2017-2021 Jorge
  Beltran, Carlos Guindel

  This file is part of velo2cam_calibration.

  velo2cam_calibration is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 2 of the License, or
  (at your option) any later version.

  velo2cam_calibration is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with velo2cam_calibration.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
  lidar_pattern: Find the circle centers in the lidar cloud
*/

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
#include "velo2cam_utils.h"

using namespace std;
using namespace sensor_msgs;

class LidarPattern : public rclcpp::Node
{
public:
  LidarPattern();
  ~LidarPattern();

private:
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
  int rings_count_;
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

LidarPattern::LidarPattern() : Node("lidar_pattern")
{
  RCLCPP_INFO(this->get_logger(), "[LiDAR] Starting....");

  sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "cloud1", 100, std::bind(&LidarPattern::callback, this, std::placeholders::_1));

  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

  range_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("range_filtered_cloud", 1);
  if (DEBUG)
  {
    pattern_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("pattern_circles", 1);
    rotated_pattern_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("rotated_pattern", 1);
    cumulative_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("cumulative_cloud", 1);
  }
  centers_pub = this->create_publisher<calibration_interfaces::msg::ClusterCentroids>("centers_cloud", 1);
  coeff_pub = this->create_publisher<pcl_msgs::msg::ModelCoefficients>("plane_model", 1);

  string csv_name;

  // Dynamic Parameters
  axis_[0] = this->declare_parameter("x", 0);
  axis_[1] = this->declare_parameter("y", 0);
  axis_[2] = this->declare_parameter("z", 1);
  angle_threshold_ = this->declare_parameter("angle_threshold", 0.55);
  circle_radius_ = this->declare_parameter("circle_radius", 0.12);
  passthrough_radius_min_ = this->declare_parameter("passthrough_radius_min", 1.0);
  passthrough_radius_max_ = this->declare_parameter("passthrough_radius_max", 6.0);
  centroid_distance_min_ = this->declare_parameter("centroid_distance_min", 0.15);
  centroid_distance_max_ = this->declare_parameter("centroid_distance_max", 0.8);
  // Non-Dynamic Parameters
  delta_width_circles_ = this->declare_parameter("delta_width_circles", 0.5);
  delta_height_circles_ = this->declare_parameter("delta_height_circles", 0.4);
  plane_threshold_ = this->declare_parameter("plane_threshold", 0.1);
  gradient_threshold_ = this->declare_parameter("gradient_threshold", 0.1);
  plane_distance_inliers_ = this->declare_parameter("plane_distance_inliers", 0.1);
  circle_threshold_ = this->declare_parameter("circle_threshold", 0.05);
  target_radius_tolerance_ = this->declare_parameter("target_radius_tolerance", 0.01);
  cluster_tolerance_ = this->declare_parameter("cluster_tolerance", 0.05);
  min_centers_found_ = this->declare_parameter("min_centers_found", TARGET_NUM_CIRCLES);
  min_cluster_factor_ = this->declare_parameter("min_cluster_factor", 0.5);
  rings_count_ = this->declare_parameter("rings_count", 64);
  skip_warmup_ = this->declare_parameter("skip_warmup", false);
  save_to_file_ = this->declare_parameter("save_to_file", false);
  csv_name = this->declare_parameter("csv_name", "lidar_pattern_" + currentDateTime() + ".csv");

  cumulative_cloud =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

  auto ret = this->add_on_set_parameters_callback(std::bind(&LidarPattern::param_callback, this, std::placeholders::_1));

  warmup_sub = this->create_subscription<std_msgs::msg::Empty>(
      "warmup_switch", 100, std::bind(&LidarPattern::warmup_callback, this, std::placeholders::_1));

  if (skip_warmup_)
  {
    RCLCPP_WARN(this->get_logger(), "Skipping warmup");
    WARMUP_DONE = true;
  }

  // Just for statistics
  if (save_to_file_)
  {
    ostringstream os;
    os << getenv("HOME") << "/v2c_experiments/" << csv_name;
    if (save_to_file_)
    {
      if (DEBUG)
        RCLCPP_INFO(this->get_logger(), "Opening %s", os.str().c_str());
      savefile.open(os.str().c_str());
      savefile << "det1_x, det1_y, det1_z, det2_x, det2_y, det2_z, det3_x, "
                  "det3_y, det3_z, det4_x, det4_y, det4_z, cent1_x, cent1_y, "
                  "cent1_z, cent2_x, cent2_y, cent2_z, cent3_x, cent3_y, "
                  "cent3_z, cent4_x, cent4_y, cent4_z, it"
               << endl;
    }
  }
}

LidarPattern::~LidarPattern()
{
  RCLCPP_INFO(this->get_logger(), "[LiDAR] Terminating....");
}

void LidarPattern::callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr laser_cloud)
{
  if (DEBUG)
    RCLCPP_INFO(this->get_logger(), "[LiDAR] Processing cloud...");

  // TODO: Check algorithm to find edges_cloud
  pcl::PointCloud<Velodyne::Point>::Ptr velocloud(
      new pcl::PointCloud<Velodyne::Point>),
      velo_filtered(new pcl::PointCloud<Velodyne::Point>),
      pattern_cloud(new pcl::PointCloud<Velodyne::Point>),
      edges_cloud(new pcl::PointCloud<Velodyne::Point>);

  clouds_proc_++;

  // This cloud is already xyz-filtered
  fromROSMsg(*laser_cloud, *velocloud);

  Velodyne::addRange(*velocloud);

  // Range passthrough filter
  pcl::PassThrough<Velodyne::Point> pass2;
  pass2.setInputCloud(velocloud);
  pass2.setFilterFieldName("range");
  pass2.setFilterLimits(passthrough_radius_min_, passthrough_radius_max_);
  pass2.filter(*velo_filtered);

  // Publishing "range_filtered_velo" cloud (segmented plane)
  sensor_msgs::msg::PointCloud2 range_ros;
  pcl::toROSMsg(*velo_filtered, range_ros);
  range_ros.header = laser_cloud->header;
  range_pub->publish(range_ros);

  // Plane segmentation
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  pcl::SACSegmentation<Velodyne::Point> plane_segmentation;
  plane_segmentation.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
  plane_segmentation.setDistanceThreshold(plane_threshold_);
  plane_segmentation.setMethodType(pcl::SAC_RANSAC);
  plane_segmentation.setAxis(Eigen::Vector3f(axis_[0], axis_[1], axis_[2]));
  plane_segmentation.setEpsAngle(angle_threshold_);
  plane_segmentation.setOptimizeCoefficients(true);
  plane_segmentation.setMaxIterations(1000);
  plane_segmentation.setInputCloud(velo_filtered);
  plane_segmentation.segment(*inliers, *coefficients);

  if (inliers->indices.size() == 0)
  {
    // Exit 1: plane not found
    RCLCPP_WARN(this->get_logger(),
                "[LiDAR] Could not estimate a planar model for the given dataset.");
    return;
  }

  // Copy coefficients to proper object for further filtering
  Eigen::VectorXf coefficients_v(4);
  coefficients_v(0) = coefficients->values[0];
  coefficients_v(1) = coefficients->values[1];
  coefficients_v(2) = coefficients->values[2];
  coefficients_v(3) = coefficients->values[3];

  // Get edges points by range
  vector<vector<Velodyne::Point *>> rings =
      Velodyne::getRings(*velocloud, rings_count_);
  for (vector<vector<Velodyne::Point *>>::iterator ring = rings.begin();
       ring < rings.end(); ++ring)
  {
    Velodyne::Point *prev, *succ;
    if (ring->empty())
      continue;

    (*ring->begin())->intensity = 0;
    (*(ring->end() - 1))->intensity = 0;
    for (vector<Velodyne::Point *>::iterator pt = ring->begin() + 1;
         pt < ring->end() - 1; pt++)
    {
      Velodyne::Point *prev = *(pt - 1);
      Velodyne::Point *succ = *(pt + 1);
      (*pt)->intensity =
          max(max(prev->range - (*pt)->range, succ->range - (*pt)->range), 0.f);
    }
  }

  float THRESHOLD =
      gradient_threshold_; // 10 cm between the pattern and the background
  for (pcl::PointCloud<Velodyne::Point>::iterator pt =
           velocloud->points.begin();
       pt < velocloud->points.end(); ++pt)
  {
    if (pt->intensity > THRESHOLD)
    {
      edges_cloud->push_back(*pt);
    }
  }

  if (edges_cloud->points.size() == 0)
  {
    // Exit 2: pattern edges not found
    RCLCPP_WARN(this->get_logger(), "[LiDAR] Could not detect pattern edges.");
    return;
  }

  // Get points belonging to plane in pattern pointcloud
  pcl::SampleConsensusModelPlane<Velodyne::Point>::Ptr dit(
      new pcl::SampleConsensusModelPlane<Velodyne::Point>(edges_cloud));
  std::vector<int> inliers2;
  dit->selectWithinDistance(coefficients_v, plane_distance_inliers_, inliers2);
  pcl::copyPointCloud<Velodyne::Point>(*edges_cloud, inliers2, *pattern_cloud);

  // Velodyne specific info no longer needed for calibration
  // so standard PointXYZ is used from now on
  pcl::PointCloud<pcl::PointXYZ>::Ptr circles_cloud(
      new pcl::PointCloud<pcl::PointXYZ>),
      xy_cloud(new pcl::PointCloud<pcl::PointXYZ>),
      aux_cloud(new pcl::PointCloud<pcl::PointXYZ>),
      auxrotated_cloud(new pcl::PointCloud<pcl::PointXYZ>),
      cloud_f(new pcl::PointCloud<pcl::PointXYZ>),
      centroid_candidates(new pcl::PointCloud<pcl::PointXYZ>);

  vector<vector<Velodyne::Point *>> rings2 =
      Velodyne::getRings(*pattern_cloud, rings_count_);

  // Conversion from Velodyne::Point to pcl::PointXYZ
  for (vector<vector<Velodyne::Point *>>::iterator ring = rings2.begin();
       ring < rings2.end(); ++ring)
  {
    for (vector<Velodyne::Point *>::iterator pt = ring->begin();
         pt < ring->end(); ++pt)
    {
      pcl::PointXYZ point;
      point.x = (*pt)->x;
      point.y = (*pt)->y;
      point.z = (*pt)->z;
      circles_cloud->push_back(point);
    }
  }

  // Publishing "pattern_circles" cloud (points belonging to the detected plane)
  if (DEBUG)
  {
    sensor_msgs::msg::PointCloud2 velocloud_ros2;
    pcl::toROSMsg(*circles_cloud, velocloud_ros2);
    velocloud_ros2.header = laser_cloud->header;
    pattern_pub->publish(velocloud_ros2);
  }

  // Rotate cloud to face pattern plane
  Eigen::Vector3f xy_plane_normal_vector, floor_plane_normal_vector;
  xy_plane_normal_vector[0] = 0.0;
  xy_plane_normal_vector[1] = 0.0;
  xy_plane_normal_vector[2] = -1.0;

  floor_plane_normal_vector[0] = coefficients->values[0];
  floor_plane_normal_vector[1] = coefficients->values[1];
  floor_plane_normal_vector[2] = coefficients->values[2];

  Eigen::Affine3f rotation =
      getRotationMatrix(floor_plane_normal_vector, xy_plane_normal_vector);
  pcl::transformPointCloud(*circles_cloud, *xy_cloud, rotation);

  // Publishing "rotated_pattern" cloud (plane transformed to be aligned with
  // XY)
  if (DEBUG)
  {
    sensor_msgs::msg::PointCloud2 ros_rotated_pattern;
    pcl::toROSMsg(*xy_cloud, ros_rotated_pattern);
    ros_rotated_pattern.header = laser_cloud->header;
    rotated_pattern_pub->publish(ros_rotated_pattern);
  }

  // Force pattern points to belong to computed plane
  pcl::PointXYZ aux_point;
  aux_point.x = 0;
  aux_point.y = 0;
  aux_point.z = (-coefficients_v(3) / coefficients_v(2));
  aux_cloud->push_back(aux_point);
  pcl::transformPointCloud(*aux_cloud, *auxrotated_cloud, rotation);
  double zcoord_xyplane = auxrotated_cloud->at(0).z;

  for (pcl::PointCloud<pcl::PointXYZ>::iterator pt = xy_cloud->points.begin();
       pt < xy_cloud->points.end(); ++pt)
  {
    pt->z = zcoord_xyplane;
  }

  // RANSAC circle detection
  pcl::ModelCoefficients::Ptr coefficients3(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers3(new pcl::PointIndices);

  pcl::SACSegmentation<pcl::PointXYZ> circle_segmentation;
  circle_segmentation.setModelType(pcl::SACMODEL_CIRCLE2D);
  circle_segmentation.setDistanceThreshold(circle_threshold_);
  circle_segmentation.setMethodType(pcl::SAC_RANSAC);
  circle_segmentation.setOptimizeCoefficients(true);
  circle_segmentation.setMaxIterations(1000);
  circle_segmentation.setRadiusLimits(
      circle_radius_ - target_radius_tolerance_,
      circle_radius_ + target_radius_tolerance_);

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  if (DEBUG)
    RCLCPP_INFO(this->get_logger(), "[LiDAR] Searching for points in cloud of size %lu",
                xy_cloud->points.size());
  while (xy_cloud->points.size() > 3)
  {
    circle_segmentation.setInputCloud(xy_cloud);
    circle_segmentation.segment(*inliers3, *coefficients3);
    if (inliers3->indices.size() == 0)
    {
      if (DEBUG)
        RCLCPP_INFO(this->get_logger(),
                    "[LiDAR] Optimized circle segmentation failed, trying unoptimized "
                    "version");
      circle_segmentation.setOptimizeCoefficients(false);

      circle_segmentation.setInputCloud(xy_cloud);
      circle_segmentation.segment(*inliers3, *coefficients3);

      // Reset for next iteration
      circle_segmentation.setOptimizeCoefficients(true);
      if (inliers3->indices.size() == 0)
      {
        break;
      }
    }

    // Add center point to cloud
    pcl::PointXYZ center;
    center.x = *coefficients3->values.begin();
    center.y = *(coefficients3->values.begin() + 1);
    center.z = zcoord_xyplane;

    centroid_candidates->push_back(center);

    // Remove inliers from pattern cloud to find next circle
    extract.setInputCloud(xy_cloud);
    extract.setIndices(inliers3);
    extract.setNegative(true);
    extract.filter(*cloud_f);
    xy_cloud.swap(cloud_f);

    if (DEBUG)
      RCLCPP_INFO(this->get_logger(), "[LiDAR] Remaining points in cloud %lu",
                  xy_cloud->points.size());
  }

  if (centroid_candidates->size() < TARGET_NUM_CIRCLES)
  {
    // Exit 3: all centers not found
    RCLCPP_WARN(this->get_logger(), "[LiDAR] Not enough centers: %ld of %d", centroid_candidates->size(), TARGET_NUM_CIRCLES);
    return;
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "[LiDAR] Enough centers: %ld", centroid_candidates->size());
  }

  /**
    Geometric consistency check
    At this point, circles' center candidates have been computed
  (found_centers). Now we need to select the set of 4 candidates that best fit
  the calibration target geometry. To that end, the following steps are
  followed: 1) Create a cloud with 4 points representing the exact geometry of
  the calibration target 2) For each possible set of 4 points: compute
  similarity score 3) Rotate back the candidates with the highest score to their
  original position in the cloud, and add them to cumulative cloud
  **/
  std::vector<std::vector<int>> groups;
  comb(centroid_candidates->size(), TARGET_NUM_CIRCLES, groups);
  double groups_scores[groups.size()]; // -1: invalid; 0-1 normalized score
  for (unsigned i = 0; i < groups.size(); ++i)
  {
    std::vector<pcl::PointXYZ> candidates;
    // Build candidates set
    for (unsigned j = 0; j < groups[i].size(); ++j)
    {
      pcl::PointXYZ center;
      center.x = centroid_candidates->at(groups[i][j]).x;
      center.y = centroid_candidates->at(groups[i][j]).y;
      center.z = centroid_candidates->at(groups[i][j]).z;
      candidates.push_back(center);
    }

    // Compute candidates score
    Square square_candidate(candidates, delta_width_circles_,
                            delta_height_circles_);
    groups_scores[i] = square_candidate.is_valid()
                           ? 1.0
                           : -1; // -1 when it's not valid, 1 otherwise
  }

  int best_candidate_idx = -1;
  double best_candidate_score = -1;
  for (unsigned i = 0; i < groups.size(); ++i)
  {
    if (best_candidate_score == 1 && groups_scores[i] == 1)
    {
      // Exit 4: Several candidates fit target's geometry
      RCLCPP_ERROR(this->get_logger(),
                   "[LiDAR] More than one set of candidates fit target's geometry. "
                   "Please, make sure your parameters are well set. Exiting callback");
      return;
    }
    if (groups_scores[i] > best_candidate_score)
    {
      best_candidate_score = groups_scores[i];
      best_candidate_idx = i;
    }
  }

  if (best_candidate_idx == -1)
  {
    // Exit 5: No candidates fit target's geometry
    RCLCPP_WARN(this->get_logger(),
                "[LiDAR] Unable to find a candidate set that matches target's "
                "geometry");
    return;
  }

  // Build selected centers set
  pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_back_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  for (unsigned j = 0; j < groups[best_candidate_idx].size(); ++j)
  {
    pcl::PointXYZ center_rotated_back = pcl::transformPoint(
        centroid_candidates->at(groups[best_candidate_idx][j]),
        rotation.inverse());
    center_rotated_back.x = (-coefficients->values[1] * center_rotated_back.y -
                             coefficients->values[2] * center_rotated_back.z -
                             coefficients->values[3]) /
                            coefficients->values[0];

    rotated_back_cloud->push_back(center_rotated_back);
    cumulative_cloud->push_back(center_rotated_back);
  }

  if (save_to_file_)
  {
    std::vector<pcl::PointXYZ> sorted_centers;
    sortPatternCenters(rotated_back_cloud, sorted_centers);
    for (std::vector<pcl::PointXYZ>::iterator it = sorted_centers.begin();
         it < sorted_centers.end(); ++it)
    {
      savefile << it->x << ", " << it->y << ", " << it->z << ", ";
    }
  }

  // Publishing "cumulative_cloud" cloud (centers found from the beginning)
  if (DEBUG)
  {
    sensor_msgs::msg::PointCloud2 ros_pointcloud;
    pcl::toROSMsg(*cumulative_cloud, ros_pointcloud);
    ros_pointcloud.header = laser_cloud->header;
    cumulative_pub->publish(ros_pointcloud);
  }

  xy_cloud.reset(); // Free memory
  cloud_f.reset();  // Free memory

  ++clouds_used_;

  // Publishing "plane_model"
  pcl_msgs::msg::ModelCoefficients m_coeff;
  pcl_conversions::moveFromPCL(*coefficients, m_coeff);
  m_coeff.header = laser_cloud->header;
  coeff_pub->publish(m_coeff);

  if (DEBUG)
    RCLCPP_INFO(this->get_logger(), "[LiDAR] %d/%d frames: %ld pts in cloud", clouds_used_,
                clouds_proc_, cumulative_cloud->points.size());

  // Create cloud for publishing centers
  pcl::PointCloud<pcl::PointXYZ>::Ptr centers_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);

  // Compute circles centers
  if (!WARMUP_DONE)
  { // Compute clusters from detections in the latest frame
    getCenterClusters(cumulative_cloud, centers_cloud, cluster_tolerance_, 1,
                      1);
  }
  else
  { // Use cumulative information from previous frames
    getCenterClusters(cumulative_cloud, centers_cloud, cluster_tolerance_,
                      min_cluster_factor_ * clouds_used_, clouds_used_);
    if (centers_cloud->points.size() > TARGET_NUM_CIRCLES)
    {
      getCenterClusters(cumulative_cloud, centers_cloud, cluster_tolerance_,
                        3.0 * clouds_used_ / 4.0, clouds_used_);
    }
  }

  // Exit 6: clustering failed
  if (centers_cloud->points.size() == TARGET_NUM_CIRCLES)
  {
    sensor_msgs::msg::PointCloud2 ros2_pointcloud;
    pcl::toROSMsg(*centers_cloud, ros2_pointcloud);
    ros2_pointcloud.header = laser_cloud->header;

    calibration_interfaces::msg::ClusterCentroids to_send;
    to_send.header = laser_cloud->header;
    to_send.cluster_iterations = clouds_used_;
    to_send.total_iterations = clouds_proc_;
    to_send.cloud = ros2_pointcloud;

    centers_pub->publish(to_send);
    if (DEBUG)
      RCLCPP_INFO(this->get_logger(), "[LiDAR] Pattern centers published");

    if (save_to_file_)
    {
      std::vector<pcl::PointXYZ> sorted_centers;
      sortPatternCenters(centers_cloud, sorted_centers);
      for (std::vector<pcl::PointXYZ>::iterator it = sorted_centers.begin();
           it < sorted_centers.end(); ++it)
      {
        savefile << it->x << ", " << it->y << ", " << it->z << ", ";
      }
      savefile << cumulative_cloud->width;
    }
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Not enough circle found");
  }

  if (save_to_file_)
  {
    savefile << endl;
  }

  // Clear cumulative cloud during warm-up phase
  if (!WARMUP_DONE)
  {
    cumulative_cloud->clear();
    clouds_proc_ = 0;
    clouds_used_ = 0;
  }
}

rcl_interfaces::msg::SetParametersResult LidarPattern::param_callback(const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  for (const auto &param : parameters)
  {
    if (param.get_name() == "passthrough_radius_min")
    {
      passthrough_radius_min_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "[LiDAR] New passthrough_radius_min_ threshold: %f",
                  passthrough_radius_min_);
    }
    if (param.get_name() == "passthrough_radius_max")
    {
      passthrough_radius_max_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "[LiDAR] New passthrough_radius_max_ threshold: %f",
                  passthrough_radius_max_);
    }
    if (param.get_name() == "circle_radius")
    {
      circle_radius_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "[LiDAR] New pattern circle radius: %f", circle_radius_);
    }
    if (param.get_name() == "x")
    {
      axis_[0] = param.as_double();
      RCLCPP_INFO(this->get_logger(), "[LiDAR] New normal axis for plane segmentation: %f, %f, %f",
                  axis_[0], axis_[1], axis_[2]);
    }
    if (param.get_name() == "y")
    {
      axis_[1] = param.as_double();
      RCLCPP_INFO(this->get_logger(), "[LiDAR] New normal axis for plane segmentation: %f, %f, %f",
                  axis_[0], axis_[1], axis_[2]);
    }
    if (param.get_name() == "z")
    {
      axis_[1] = param.as_double();
      RCLCPP_INFO(this->get_logger(), "[LiDAR] New normal axis for plane segmentation: %f, %f, %f",
                  axis_[0], axis_[1], axis_[2]);
    }
    if (param.get_name() == "angle_threshold")
    {
      angle_threshold_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "[LiDAR] New angle threshold: %f", angle_threshold_);
    }
    if (param.get_name() == "centroid_distance_min")
    {
      centroid_distance_min_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "[LiDAR] New minimum distance between centroids: %f",
                  centroid_distance_min_);
    }
    if (param.get_name() == "centroid_distance_max")
    {
      centroid_distance_max_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "[LiDAR] New maximum distance between centroids: %f",
                  centroid_distance_max_);
    }
  }
  return result;
}

void LidarPattern::warmup_callback(const std_msgs::msg::Empty::ConstSharedPtr msg)
{
  WARMUP_DONE = !WARMUP_DONE;
  if (WARMUP_DONE)
  {
    RCLCPP_INFO(this->get_logger(), "[LiDAR] Warm up done, pattern detection started");
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "[LiDAR] Detection stopped. Warm up mode activated");
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<LidarPattern>();

  rclcpp::spin(nh);
  return 0;
}