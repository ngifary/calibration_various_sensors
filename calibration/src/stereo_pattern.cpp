/*
  stereo_pattern: Find the circle centers in the stereo cloud
*/

#include "stereo_pattern.hpp"

StereoPattern::StereoPattern() : Node("stereo_pattern")
{
  RCLCPP_INFO(this->get_logger(), "[Stereo] Starting....");

  rclcpp::QoS qos(10);
  auto rmw_qos_profile = qos.get_rmw_qos_profile();

  camera_cloud_sub_.subscribe(this, "cloud2", rmw_qos_profile);
  cam_plane_coeffs_sub_.subscribe(this, "cam_plane_coeffs", rmw_qos_profile);

  if (DEBUG)
  {
    inliers_pub_ = this->create_publisher<pcl_msgs::msg::PointIndices>("inliers_pub_", 1);
    coeff_pub_ = this->create_publisher<pcl_msgs::msg::ModelCoefficients>("coeff_pub_", 1);
    plane_edges_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("plane_edges_pub_", 1);
    xy_pattern_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("xy_pattern_pub_", 1);
    centers_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("centers_cloud", 1);
  }
  final_pub_ = this->create_publisher<calibration::msg::CircleCentroids>("centers_msg", 1);

  sync_ = std::make_shared<message_filters::Synchronizer<ExactSync>>(max_queue_size_);
  sync_->connectInput(camera_cloud_sub_, cam_plane_coeffs_sub_);
  sync_->registerCallback(std::bind(&StereoPattern::callback, this, std::placeholders::_1, std::placeholders::_2));

  std::string csv_name;

  initializeParams();

  csv_name = this->declare_parameter("csv_name", "stereo_pattern_" + currentDateTime() + ".csv");

  // ROS param callback
  auto ret = this->add_on_set_parameters_callback(std::bind(&StereoPattern::param_callback, this, std::placeholders::_1));
}

StereoPattern::~StereoPattern()
{
  RCLCPP_INFO(this->get_logger(), "[Stereo] Terminating....");
}

void StereoPattern::initializeParams()
{
  rcl_interfaces::msg::ParameterDescriptor desc;

  desc.name = "delta_width_circles";
  desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  desc.description = "distance from left circles centre to right circles centre (m)";
  delta_width_circles_ = declare_parameter(desc.name, 0.5);

  desc.name = "delta_height_circles_";
  desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  desc.description = "distance from top circles centre to bottom circles centre (m)";
  delta_height_circles_ = declare_parameter(desc.name, 0.4);

  desc.name = "plane_distance_inliers";
  desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  desc.description = "";
  plane_distance_inliers_ = declare_parameter(desc.name, 0.1);

  desc.name = "circle_threshold";
  desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  desc.description = "";
  circle_threshold_ = declare_parameter(desc.name, 0.05);

  desc.name = "circle_radius";
  desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  desc.description = "Radius of pattern's circles";
  circle_radius_ = declare_parameter(desc.name, 0.12);

  desc.name = "target_radius_tolerance";
  desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  desc.description = "";
  target_radius_tolerance_ = declare_parameter(desc.name, 0.1);
}

void StereoPattern::callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr camera_cloud,
                             const pcl_msgs::msg::ModelCoefficients::ConstSharedPtr cam_plane_coeffs)
{
  if (DEBUG)
    RCLCPP_INFO(this->get_logger(), "[Stereo] Processing image...");

  pcl::PointCloud<pcl::PointXYZ>::Ptr cam_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());

  pcl::fromROSMsg(*camera_cloud, *cam_cloud);

  // 1.FILTER THE EDGES-CLOUD ACCORDING TO THE DETECTED PLANE
  // Segment inliers
  Eigen::VectorXf coefficients_v(4);
  coefficients_v(0) = cam_plane_coeffs->values[0];
  coefficients_v(1) = cam_plane_coeffs->values[1];
  coefficients_v(2) = cam_plane_coeffs->values[2];
  coefficients_v(3) = cam_plane_coeffs->values[3];

  pcl::PointCloud<pcl::PointXYZ>::Ptr cam_plane_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr dit(
      new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cam_cloud));
  std::vector<int> plane_inliers;
  dit->selectWithinDistance(coefficients_v, plane_distance_inliers_,
                            plane_inliers);
  pcl::copyPointCloud<pcl::PointXYZ>(*cam_cloud, plane_inliers,
                                     *cam_plane_cloud);

  // Publish plane as "plane_edges_cloud"
  if (DEBUG)
  {
    sensor_msgs::msg::PointCloud2 plane_edges_ros;
    pcl::toROSMsg(*cam_plane_cloud, plane_edges_ros);
    plane_edges_ros.header = camera_cloud->header;
    plane_edges_pub_->publish(plane_edges_ros);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  // 2.ROTATE CLOUD TO FACE PATTERN PLANE
  pcl::PointCloud<pcl::PointXYZ>::Ptr xy_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  Eigen::Vector3f xy_plane_normal_vector, floor_plane_normal_vector;
  xy_plane_normal_vector[0] = 0.0;
  xy_plane_normal_vector[1] = 0.0;
  xy_plane_normal_vector[2] = -1.0;

  floor_plane_normal_vector[0] = cam_plane_coeffs->values[0];
  floor_plane_normal_vector[1] = cam_plane_coeffs->values[1];
  floor_plane_normal_vector[2] = cam_plane_coeffs->values[2];

  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cam_plane_cloud, *cam_plane_cloud, indices);

  Eigen::Affine3f rotation =
      getRotationMatrix(floor_plane_normal_vector, xy_plane_normal_vector);
  pcl::transformPointCloud(*cam_plane_cloud, *xy_cloud, rotation);

  pcl::PointCloud<pcl::PointXYZ>::Ptr aux_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointXYZ aux_point;
  aux_point.x = 0;
  aux_point.y = 0;
  aux_point.z = (-coefficients_v(3) / coefficients_v(2));
  aux_cloud->push_back(aux_point);

  pcl::PointCloud<pcl::PointXYZ>::Ptr auxrotated_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*aux_cloud, *auxrotated_cloud, rotation);

  double zcoord_xyplane = auxrotated_cloud->at(0).z;

  // Force pattern points to belong to computed plane
  for (pcl::PointCloud<pcl::PointXYZ>::iterator pt = xy_cloud->points.begin();
       pt < xy_cloud->points.end(); ++pt)
  {
    pt->z = zcoord_xyplane;
  }

  // Publishing "xy_pattern" (pattern transformed to be aligned with XY)
  if (DEBUG)
  {
    sensor_msgs::msg::PointCloud2 xy_pattern_ros;
    pcl::toROSMsg(*xy_cloud, xy_pattern_ros);
    xy_pattern_ros.header = camera_cloud->header;
    xy_pattern_pub_->publish(xy_pattern_ros);
  }

  // 3.FIND CIRCLES
  pcl::SACSegmentation<pcl::PointXYZ> seg;

  seg.setModelType(pcl::SACMODEL_CIRCLE2D);
  seg.setDistanceThreshold(circle_threshold_);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setOptimizeCoefficients(true);
  seg.setMaxIterations(1000);
  seg.setRadiusLimits(circle_radius_ - target_radius_tolerance_,
                      circle_radius_ + target_radius_tolerance_);

  pcl::PointCloud<pcl::PointXYZ>::Ptr circle_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);

  std::vector<std::vector<float>> found_centers;

  do
  {
    seg.setInputCloud(xy_cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0)
    {
      if (found_centers.size() < 1)
      {
        RCLCPP_WARN(this->get_logger(),
                    "[Stereo] Could not estimate a circle model for the given "
                    "dataset.");
      }
      break;
    }
    else
    {
      if (DEBUG)
        RCLCPP_INFO(this->get_logger(), "[Stereo] Found circle: %lu inliers", inliers->indices.size());
    }

    // Extract the inliers
    extract.setInputCloud(xy_cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*circle_cloud);

    // Add center point to cloud (only if it makes sense)
    pcl::PointXYZ center;
    center.x = *coefficients->values.begin();
    center.y = *(coefficients->values.begin() + 1);
    center.z = zcoord_xyplane;

    std::vector<float> found_center;
    found_center.push_back(center.x);
    found_center.push_back(center.y);
    found_center.push_back(center.z);
    found_centers.push_back(found_center);

    // Remove inliers from pattern cloud to find next circle
    extract.setNegative(true);
    extract.filter(*temp_cloud);
    xy_cloud.swap(temp_cloud);

  } while (xy_cloud->points.size() > 3);

  if (found_centers.size() < TARGET_NUM_CIRCLES)
  { // Usually TARGET_NUM_CIRCLES = TARGET_NUM_CIRCLES
    // Exit 1: centers not found
    RCLCPP_WARN(this->get_logger(), "Not enough centers: %ld", found_centers.size());
    return;
  }

  /**
    4. GEOMETRIC CONSISTENCY CHECK
    At this point, circles' center candidates have been computed
  (found_centers). Now we need to select the set of 4 candidates that best fit
  the calibration target geometry. To that end, the following steps are
  followed: 1) Create a cloud with 4 points representing the exact geometry of
  the calibration target 2) For each possible set of 4 points: compute
  similarity score 3) Rotate back the candidates with the highest score to their
  original position in the cloud, and add them to cumulative cloud
  **/
  std::vector<std::vector<int>> groups;
  comb(found_centers.size(), TARGET_NUM_CIRCLES, groups);
  double groups_scores[groups.size()]; // -1: invalid; 0-1 normalized score
  for (unsigned i = 0; i < groups.size(); ++i)
  {
    std::vector<pcl::PointXYZ> candidates;
    // Build candidates set
    for (unsigned j = 0; j < groups[i].size(); ++j)
    {
      pcl::PointXYZ center;
      center.x = found_centers[groups[i][j]][0];
      center.y = found_centers[groups[i][j]][1];
      center.z = found_centers[groups[i][j]][2];
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
      // Exit 2: Several candidates fit target's geometry
      RCLCPP_ERROR(this->get_logger(),
                   "[Stereo] More than one set of candidates fit target's geometry. "
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
    // Exit 3: No candidates fit target's geometry
    RCLCPP_WARN(this->get_logger(),
                "[Stereo] Unable to find a candidate set that matches target's "
                "geometry");
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_back_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  for (unsigned j = 0; j < groups[best_candidate_idx].size(); ++j)
  {
    pcl::PointXYZ point;
    point.x = found_centers[groups[best_candidate_idx][j]][0];
    point.y = found_centers[groups[best_candidate_idx][j]][1];
    point.z = found_centers[groups[best_candidate_idx][j]][2];

    pcl::PointXYZ center_rotated_back =
        pcl::transformPoint(point, rotation.inverse());
    center_rotated_back.z =
        (-cam_plane_coeffs->values[0] * center_rotated_back.x -
         cam_plane_coeffs->values[1] * center_rotated_back.y -
         cam_plane_coeffs->values[3]) /
        cam_plane_coeffs->values[2];

    rotated_back_cloud->push_back(center_rotated_back);
  }

  pcl_msgs::msg::PointIndices p_ind;

  pcl_conversions::moveFromPCL(*inliers, p_ind);
  p_ind.header = camera_cloud->header;

  pcl_msgs::msg::ModelCoefficients m_coeff;

  pcl_conversions::moveFromPCL(*coefficients, m_coeff);
  m_coeff.header = camera_cloud->header;

  if (DEBUG)
  {
    inliers_pub_->publish(p_ind);
    coeff_pub_->publish(m_coeff);
  }

  if (rotated_back_cloud->points.size() == TARGET_NUM_CIRCLES)
  {
    sensor_msgs::msg::PointCloud2 ros2_pointcloud;
    pcl::toROSMsg(*rotated_back_cloud, ros2_pointcloud);
    ros2_pointcloud.header = camera_cloud->header;
    if (DEBUG)
    {
      centers_pub_->publish(ros2_pointcloud);
      RCLCPP_INFO(this->get_logger(), "[%s] Pattern centers published.", get_name());
    }

    calibration::msg::CircleCentroids to_send;
    to_send.header = camera_cloud->header;
    to_send.sensor_type = to_send.STEREOCAMERA;
    to_send.cloud = *camera_cloud;
    to_send.centers = ros2_pointcloud;
    final_pub_->publish(to_send);
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Not enough circle found");
  }
}

rcl_interfaces::msg::SetParametersResult StereoPattern::param_callback(const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  for (const auto &param : parameters)
  {
    if (param.get_name() == "circle_radius")
    {
      circle_radius_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "[%s] New pattern circle radius: %f", get_name(), circle_radius_);
    }
    if (param.get_name() == "circle_threshold")
    {
      circle_threshold_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "[Stereo] New circle threshold: %f", circle_threshold_);
    }
  }
  return result;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<StereoPattern>();
  rclcpp::spin(nh);
  return 0;
}
