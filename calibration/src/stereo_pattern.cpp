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
    // cumulative_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cumulative_pub_", 1);
  }
  centers_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("centers_cloud", 1);
  // final_pub_ = this->create_publisher<calibration_interfaces::msg::ClusterCentroids>("centers_cloud", 1);

  // cumulative_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

  sync_ = std::make_shared<message_filters::Synchronizer<ExactSync>>(max_queue_size_);

  sync_->connectInput(camera_cloud_sub_, cam_plane_coeffs_sub_);

  sync_->registerCallback(std::bind(&StereoPattern::callback, this, std::placeholders::_1, std::placeholders::_2));

  std::string csv_name;

  initializeParams();

  csv_name = this->declare_parameter("csv_name", "stereo_pattern_" + currentDateTime() + ".csv");

  // ROS param callback
  auto ret = this->add_on_set_parameters_callback(std::bind(&StereoPattern::param_callback, this, std::placeholders::_1));

  // warmup_sub_ = this->create_subscription<std_msgs::msg::Empty>(
  //     "warmup_switch", 100, std::bind(&StereoPattern::warmup_callback, this, std::placeholders::_1));

  // if (skip_warmup_)
  // {
  //   RCLCPP_WARN(this->get_logger(), "Skipping warmup");
  //   WARMUP_DONE = true;
  // }

  // // Just for statistics
  // if (save_to_file_)
  // {
  //   std::ostringstream os;
  //   os << getenv("HOME") << "/v2c_experiments/" << csv_name;
  //   if (save_to_file_)
  //   {
  //     if (DEBUG)
  //       RCLCPP_INFO(this->get_logger(), "Opening %s", os.str().c_str());
  //     savefile_.open(os.str().c_str());
  //     savefile_ << "det1_x, det1_y, det1_z, det2_x, det2_y, det2_z, det3_x, "
  //                  "det3_y, det3_z, det4_x, det4_y, det4_z, cent1_x, cent1_y, "
  //                  "cent1_z, cent2_x, cent2_y, cent2_z, cent3_x, cent3_y, "
  //                  "cent3_z, cent4_x, cent4_y, cent4_z, it"
  //               << std::endl;
  //   }
  // }
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

  // desc.name = "min_centers_found";
  // desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  // desc.description = "minimum circle to be detected (-)";
  // TARGET_NUM_CIRCLES = declare_parameter(desc.name, TARGET_NUM_CIRCLES);

  // desc.name = "cluster_tolerance";
  // desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  // desc.description = "maximal distance to still be included in a cluster (m)";
  // cluster_tolerance_ = declare_parameter(desc.name, 0.05);

  // desc.name = "min_cluster_factor";
  // desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  // desc.description = "minimum cluster size to frame ratio (-)";
  // min_cluster_factor_ = declare_parameter(desc.name, 0.5);

  // desc.name = "skip_warmup";
  // desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  // desc.description = "skip warmup";
  // skip_warmup_ = declare_parameter(desc.name, false);

  // desc.name = "save_to_file";
  // desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  // desc.description = "save result to a file";
  // save_to_file_ = declare_parameter(desc.name, false);
}

void StereoPattern::callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr camera_cloud,
                             const pcl_msgs::msg::ModelCoefficients::ConstSharedPtr cam_plane_coeffs)
{
  if (DEBUG)
    RCLCPP_INFO(this->get_logger(), "[Stereo] Processing image...");

  // images_proc_++;

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
    // cumulative_cloud_->push_back(center_rotated_back);
  }

  // if (save_to_file_)
  // {
  //   std::vector<pcl::PointXYZ> sorted_centers;
  //   sortPatternCenters(rotated_back_cloud, sorted_centers);
  //   for (std::vector<pcl::PointXYZ>::iterator it = sorted_centers.begin();
  //        it < sorted_centers.end(); ++it)
  //   {
  //     savefile_ << it->x << ", " << it->y << ", " << it->z << ", ";
  //   }
  // }

  // // Publishing "cumulative_cloud_" (centers found from the beginning)
  // if (DEBUG)
  // {
  //   sensor_msgs::msg::PointCloud2 cumulative_ros;
  //   pcl::toROSMsg(*cumulative_cloud_, cumulative_ros);
  //   cumulative_ros.header = camera_cloud->header;
  //   cumulative_pub_->publish(cumulative_ros);
  // }

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

  // images_used_++;
  // if (DEBUG)
  // {
  //   RCLCPP_INFO(this->get_logger(), "[Stereo] %d/%d frames: %ld pts in cloud", images_used_,
  //               images_proc_, cumulative_cloud_->points.size());
  // }
  // pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud(
  //     new pcl::PointCloud<pcl::PointXYZ>);

  // // Compute circles centers
  // if (!WARMUP_DONE)
  // { // Compute clusters from detections in the latest frame
  //   getCenterClusters(cumulative_cloud_, final_cloud, cluster_tolerance_, 1, 1);
  // }
  // else
  // { // Use cumulative information from previous frames
  //   getCenterClusters(cumulative_cloud_, final_cloud, cluster_tolerance_,
  //                     min_cluster_factor_ * images_used_, images_used_);
  //   if (final_cloud->points.size() > TARGET_NUM_CIRCLES)
  //   {
  //     getCenterClusters(cumulative_cloud_, final_cloud, cluster_tolerance_,
  //                       3.0 * images_used_ / 4.0, images_used_);
  //   }
  // }

  // // Exit 4: clustering failed
  // if (final_cloud->points.size() == TARGET_NUM_CIRCLES)
  // {
  //   if (save_to_file_)
  //   {
  //     std::vector<pcl::PointXYZ> sorted_centers;
  //     sortPatternCenters(final_cloud, sorted_centers);
  //     for (std::vector<pcl::PointXYZ>::iterator it = sorted_centers.begin();
  //          it < sorted_centers.end(); ++it)
  //     {
  //       savefile_ << it->x << ", " << it->y << ", " << it->z << ", ";
  //     }
  //     savefile_ << cumulative_cloud_->width;
  //   }

  //   sensor_msgs::msg::PointCloud2 final_ros;
  //   pcl::toROSMsg(*final_cloud, final_ros);
  //   final_ros.header = camera_cloud->header;

  //   calibration_interfaces::msg::ClusterCentroids to_send;
  //   to_send.header = camera_cloud->header;
  //   to_send.total_iterations = images_proc_;
  //   to_send.cluster_iterations = images_used_;
  //   to_send.cloud = final_ros;

  //   final_pub_->publish(to_send);
  // }

  // if (save_to_file_)
  // {
  //   savefile_ << std::endl;
  // }

  // // Clear cumulative cloud during warm-up phase
  // if (!WARMUP_DONE)
  // {
  //   cumulative_cloud_->clear();
  //   images_proc_ = 0;
  //   images_used_ = 0;
  // }
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

// void StereoPattern::warmup_callback(const std_msgs::msg::Empty::ConstSharedPtr msg)
// {
//   WARMUP_DONE = !WARMUP_DONE;
//   if (WARMUP_DONE)
//   {
//     RCLCPP_INFO(this->get_logger(), "[Stereo] Warm up done, pattern detection started");
//   }
//   else
//   {
//     RCLCPP_INFO(this->get_logger(), "[Stereo] Detection stopped. Warm up mode activated");
//   }
// }

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<StereoPattern>();
  rclcpp::spin(nh);
  return 0;
}
