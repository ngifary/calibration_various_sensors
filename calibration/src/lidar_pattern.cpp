/*
  lidar_pattern: Find the circle centers in the lidar cloud
*/

#include "lidar_pattern.hpp"

LidarPattern::LidarPattern() : Node("lidar_pattern")
{
  RCLCPP_INFO(this->get_logger(), "[%s] Starting....", get_name());

  sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "cloud1", 10, std::bind(&LidarPattern::callback, this, std::placeholders::_1));

  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

  range_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("range_filtered_cloud", 1);
  if (DEBUG)
  {
    pattern_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pattern_circles", 1);
    rotated_pattern_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("rotated_pattern", 1);
    // cumulative_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cumulative_cloud_", 1);
  }
  // centers_pub_ = this->create_publisher<calibration_interfaces::msg::ClusterCentroids>("centers_cloud", 1);
  centers_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("centers_cloud", 1);
  coeff_pub_ = this->create_publisher<pcl_msgs::msg::ModelCoefficients>("plane_model", 1);

  std::string csv_name;

  initializeParams();

  csv_name = this->declare_parameter("csv_name", "lidar_pattern_" + currentDateTime() + ".csv");

  // cumulative_cloud_ =
  //     pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

  auto ret = this->add_on_set_parameters_callback(std::bind(&LidarPattern::param_callback, this, std::placeholders::_1));

  // warmup_sub_ = this->create_subscription<std_msgs::msg::Empty>(
  //     "warmup_switch", 100, std::bind(&LidarPattern::warmup_callback, this, std::placeholders::_1));

  // if (skip_warmup_)
  // {
  //   RCLCPP_WARN(this->get_logger(), "Skipping warmup");
  //   WARMUP_DONE = true;
  // }

  // // Just for statistics
  // if (save_to_file_)
  // {
  //   std::ostringstream os;
  //   os << getenv("HOME") << "/l2c_experiments/" << csv_name;
  //   if (save_to_file_)
  //   {
  //     if (DEBUG)
  //       RCLCPP_INFO(this->get_logger(), "Opening %s", os.str().c_str());
  //     savefile_.open(os.str().c_str());
  //     savefile_ << "det1_x, det1_y, det1_z, det2_x, det2_y, det2_z, det3_x, "
  //                 "det3_y, det3_z, det4_x, det4_y, det4_z, cent1_x, cent1_y, "
  //                 "cent1_z, cent2_x, cent2_y, cent2_z, cent3_x, cent3_y, "
  //                 "cent3_z, cent4_x, cent4_y, cent4_z, it"
  //              << std::endl;
  //   }
  // }
}

LidarPattern::~LidarPattern()
{
  RCLCPP_INFO(this->get_logger(), "[%s] Terminating....", get_name());
}

void LidarPattern::initializeParams()
{
  rcl_interfaces::msg::ParameterDescriptor desc;

  desc.name = "x";
  desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  desc.description = "x-coordinate";
  axis_[0] = declare_parameter(desc.name, 0);

  desc.name = "y";
  desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  desc.description = "y-coordinate";
  axis_[1] = declare_parameter(desc.name, 0);

  desc.name = "z";
  desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  desc.description = "z-coordinate";
  axis_[2] = declare_parameter(desc.name, 1);

  desc.name = "angle_threshold";
  desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  desc.description = "Angle threshold for plane segmentation";
  angle_threshold_ = declare_parameter(desc.name, 0.55);

  desc.name = "circle_radius";
  desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  desc.description = "Radius of pattern's circles";
  circle_radius_ = declare_parameter(desc.name, 0.12);

  desc.name = "passthrough_radius_min";
  desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  desc.description = "Min radius for passthrough";
  passthrough_radius_min_ = declare_parameter(desc.name, 1.0);

  desc.name = "passthrough_radius_max";
  desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  desc.description = "Max radius for passthrough";
  passthrough_radius_max_ = declare_parameter(desc.name, 6.0);

  desc.name = "delta_width_circles";
  desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  desc.description = "distance from left circles centre to right circles centre (m)";
  delta_width_circles_ = declare_parameter(desc.name, 0.5);

  desc.name = "delta_height_circles";
  desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  desc.description = "distance from top circles centre to bottom circles centre (m)";
  delta_height_circles_ = declare_parameter(desc.name, 0.4);

  // desc.name = "theta";
  // desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  // desc.description = "The vertical angular resolution (polar resolution) of the LiDAR (deg)";
  // desc.read_only = true;
  // theta_ = declare_parameter(desc.name, 0.625);

  // desc.name = "phi";
  // desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  // desc.description = "The horizontal angular resolution (azimuth resolution) of the LiDAR (deg)";
  // desc.read_only = true;
  // phi_ = declare_parameter(desc.name, 0.13);

  // desc.name = "radius";
  // desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  // desc.description = "The linear resolution of the LiDAR (m)";
  // desc.read_only = true;
  // radius_ = declare_parameter(desc.name, 0.625);

  desc.name = "line_threshold";
  desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  desc.description = "Line threshold for line segmentation (m)";
  line_threshold_ = declare_parameter(desc.name, 0.005);

  desc.name = "plane_threshold";
  desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  desc.description = "";
  plane_threshold_ = declare_parameter(desc.name, 0.1);

  desc.name = "gap_threshold";
  desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  desc.description = "";
  gap_threshold_ = declare_parameter(desc.name, 0.01);

  desc.name = "circle_threshold";
  desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  desc.description = "";
  circle_threshold_ = declare_parameter(desc.name, 0.05);

  desc.name = "target_radius_tolerance";
  desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  desc.description = "";
  target_radius_tolerance_ = declare_parameter(desc.name, 0.01);

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

void LidarPattern::callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr laser_cloud)
{
  if (DEBUG)
    RCLCPP_INFO(this->get_logger(), "[%s] Processing cloud...", get_name());

  pcl::PointCloud<LaserScanner::Point>::Ptr lasercloud(new pcl::PointCloud<LaserScanner::Point>),
      laser_filtered(new pcl::PointCloud<LaserScanner::Point>),
      pattern_cloud(new pcl::PointCloud<LaserScanner::Point>);

  // clouds_proc_++;

  // This cloud is already xyz-filtered
  fromROSMsg(*laser_cloud, *lasercloud);

  LaserScanner::addRange(*lasercloud);

  // Range passthrough filter
  pcl::PassThrough<LaserScanner::Point> pass2;
  pass2.setInputCloud(lasercloud);
  pass2.setFilterFieldName("range");
  pass2.setFilterLimits(passthrough_radius_min_, passthrough_radius_max_);
  pass2.filter(*laser_filtered);

  // Publishing "range_filtered_laser" cloud (segmented plane)
  sensor_msgs::msg::PointCloud2 range_ros;
  pcl::toROSMsg(*laser_filtered, range_ros);
  range_ros.header = laser_cloud->header;
  range_pub_->publish(range_ros);

  // Plane segmentation
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  pcl::SACSegmentation<LaserScanner::Point> plane_segmentation;
  plane_segmentation.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
  plane_segmentation.setDistanceThreshold(plane_threshold_);
  plane_segmentation.setMethodType(pcl::SAC_RANSAC);
  plane_segmentation.setAxis(axis_);
  plane_segmentation.setEpsAngle(angle_threshold_);
  plane_segmentation.setOptimizeCoefficients(true);
  plane_segmentation.setMaxIterations(1000);
  plane_segmentation.setInputCloud(laser_filtered);
  plane_segmentation.segment(*inliers, *coefficients);

  if (inliers->indices.size() == 0)
  {
    // Exit 1: plane not found
    RCLCPP_WARN(this->get_logger(),
                "[%s] Could not estimate a planar model for the given dataset.", get_name());
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::copyPointCloud<LaserScanner::Point>(*laser_filtered, *inliers, *plane_cloud);

  LaserScanner::toPlane(*plane_cloud, *coefficients);

  // Copy coefficients to proper object for further filtering
  Eigen::VectorXf coefficients_v(4);
  coefficients_v(0) = coefficients->values[0];
  coefficients_v(1) = coefficients->values[1];
  coefficients_v(2) = coefficients->values[2];
  coefficients_v(3) = coefficients->values[3];

  // Laser scanner specific info no longer needed for calibration
  // so standard PointXYZ is used from now on
  pcl::PointCloud<pcl::PointXYZ>::Ptr edges_cloud(new pcl::PointCloud<pcl::PointXYZ>),
      xy_cloud(new pcl::PointCloud<pcl::PointXYZ>),
      aux_cloud(new pcl::PointCloud<pcl::PointXYZ>),
      auxrotated_cloud(new pcl::PointCloud<pcl::PointXYZ>),
      cloud_f(new pcl::PointCloud<pcl::PointXYZ>),
      cloud_f_sorted(new pcl::PointCloud<pcl::PointXYZ>),
      centroid_candidates(new pcl::PointCloud<pcl::PointXYZ>);

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
  pcl::transformPointCloud(*plane_cloud, *xy_cloud, rotation);

  // Publishing "rotated_pattern" cloud (plane transformed to be aligned with
  // XY)
  if (DEBUG)
  {
    sensor_msgs::msg::PointCloud2 ros_rotated_pattern;
    pcl::toROSMsg(*xy_cloud, ros_rotated_pattern);
    ros_rotated_pattern.header = laser_cloud->header;
    rotated_pattern_pub_->publish(ros_rotated_pattern);
  }

  double zcoord_xyplane = xy_cloud->at(0).z;

  // line detection
  pcl::ModelCoefficients::Ptr coefficients2(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers2(new pcl::PointIndices);

  pcl::SACSegmentation<pcl::PointXYZ> line_segmentation;
  line_segmentation.setModelType(pcl::SACMODEL_LINE);
  line_segmentation.setDistanceThreshold(line_threshold_);
  line_segmentation.setMethodType(pcl::SAC_RANSAC);
  line_segmentation.setMaxIterations(1000);

  pcl::ExtractIndices<pcl::PointXYZ> extract;

  while (xy_cloud->points.size() > 6)
  {
    line_segmentation.setInputCloud(xy_cloud);
    line_segmentation.segment(*inliers2, *coefficients2);

    if (inliers2->indices.size() == 0)
    {
      break;
    }

    extract.setInputCloud(xy_cloud);
    extract.setIndices(inliers2);
    extract.setNegative(false);
    extract.filter(*cloud_f);

    // Get edges points by gap
    std::priority_queue<pcl::PointXYZ, std::vector<pcl::PointXYZ>, classcomp> priQue;
    for (unsigned int k = 0; k < cloud_f->size(); k++)
      priQue.push(cloud_f->points[k]);
    while (!priQue.empty())
    {
      const pcl::PointXYZ &point = priQue.top();
      cloud_f_sorted->push_back(point);
      priQue.pop();
    }

    float gap;
    for (unsigned int l = 2; l < cloud_f_sorted->size() - 2; l++)
    {
      gap = pcl::squaredEuclideanDistance(cloud_f_sorted->points[l + 1], cloud_f_sorted->points[l]);
      if (gap > (gap_threshold_ * gap_threshold_))
      {
        edges_cloud->push_back(cloud_f_sorted->points[l]);
        edges_cloud->push_back(cloud_f_sorted->points[l + 1]);
        l++;
      }
    }
    cloud_f_sorted->clear();

    // Remove inliers from pattern cloud to find next line
    extract.setNegative(true);
    extract.filter(*cloud_f);
    xy_cloud.swap(cloud_f);
  }

  if (edges_cloud->points.size() == 0)
  {
    // Exit 2: pattern edges not found
    RCLCPP_WARN(this->get_logger(), "[%s] Could not detect pattern edges.", get_name());
    return;
  }

  // Publishing "pattern_circles" cloud (points belonging to the detected plane)
  if (DEBUG)
  {
    sensor_msgs::msg::PointCloud2 lasercloud_ros2;
    pcl::toROSMsg(*edges_cloud, lasercloud_ros2);
    lasercloud_ros2.header = laser_cloud->header;
    pattern_pub_->publish(lasercloud_ros2);
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

  if (DEBUG)
    RCLCPP_INFO(this->get_logger(), "[%s] Searching for points in cloud of size %lu",
                get_name(), edges_cloud->points.size());
  if (edges_cloud->points.size() > 20)
  {
    RCLCPP_INFO(get_logger(), "[%s] Too much points in cloud. Please adjust gap_threshold", get_name());
    return;
  }
  while (edges_cloud->points.size() > 3)
  {
    circle_segmentation.setInputCloud(xy_cloud);
    circle_segmentation.segment(*inliers3, *coefficients3);
    if (inliers3->indices.size() == 0)
    {
      if (DEBUG)
        RCLCPP_INFO(this->get_logger(),
                    "[%s] Optimized circle segmentation failed, trying unoptimized "
                    "version",
                    get_name());
      circle_segmentation.setOptimizeCoefficients(false);

      circle_segmentation.setInputCloud(edges_cloud);
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
    extract.setInputCloud(edges_cloud);
    extract.setIndices(inliers3);
    extract.setNegative(true);
    extract.filter(*cloud_f);
    edges_cloud.swap(cloud_f);

    if (DEBUG)
      RCLCPP_INFO(this->get_logger(), "[%s] Remaining points in cloud %lu",
                  get_name(), edges_cloud->points.size());
  }

  if (centroid_candidates->size() < TARGET_NUM_CIRCLES)
  {
    // Exit 3: all centers not found
    RCLCPP_WARN(this->get_logger(), "[%s] Not enough centers: %ld of %d", get_name(), centroid_candidates->size(), TARGET_NUM_CIRCLES);
    return;
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "[%s] Enough centers: %ld", get_name(), centroid_candidates->size());
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
                   "[%s] More than one set of candidates fit target's geometry. "
                   "Please, make sure your parameters are well set. Exiting callback",
                   get_name());
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
                "[%s] Unable to find a candidate set that matches target's "
                "geometry",
                get_name());
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

  // // Publishing "cumulative_cloud_" cloud (centers found from the beginning)
  // if (DEBUG)
  // {
  //   sensor_msgs::msg::PointCloud2 ros_pointcloud;
  //   pcl::toROSMsg(*cumulative_cloud_, ros_pointcloud);
  //   ros_pointcloud.header = laser_cloud->header;
  //   cumulative_pub_->publish(ros_pointcloud);
  // }

  xy_cloud.reset(); // Free memory
  cloud_f.reset();  // Free memory

  // ++clouds_used_;

  // Publishing "plane_model"
  pcl_msgs::msg::ModelCoefficients m_coeff;
  pcl_conversions::moveFromPCL(*coefficients, m_coeff);
  m_coeff.header = laser_cloud->header;
  coeff_pub_->publish(m_coeff);

  // if (DEBUG)
  //   RCLCPP_INFO(this->get_logger(), "[%s] %d/%d frames: %ld pts in cloud", get_name(), clouds_used_,
  //               clouds_proc_, cumulative_cloud_->points.size());

  // // Create cloud for publishing centers
  // pcl::PointCloud<pcl::PointXYZ>::Ptr centers_cloud(
  //     new pcl::PointCloud<pcl::PointXYZ>);

  // // Compute circles centers
  // if (!WARMUP_DONE)
  // { // Compute clusters from detections in the latest frame
  //   getCenterClusters(cumulative_cloud_, centers_cloud, cluster_tolerance_, 1,
  //                     1);
  // }
  // else
  // { // Use cumulative information from previous frames
  //   getCenterClusters(cumulative_cloud_, centers_cloud, cluster_tolerance_,
  //                     min_cluster_factor_ * clouds_used_, clouds_used_);
  //   if (centers_cloud->points.size() > TARGET_NUM_CIRCLES)
  //   {
  //     getCenterClusters(cumulative_cloud_, centers_cloud, cluster_tolerance_,
  //                       3.0 * clouds_used_ / 4.0, clouds_used_);
  //   }
  // }

  if (rotated_back_cloud->points.size() == TARGET_NUM_CIRCLES)
  {
    sensor_msgs::msg::PointCloud2 ros2_pointcloud;
    pcl::toROSMsg(*rotated_back_cloud, ros2_pointcloud);
    ros2_pointcloud.header = laser_cloud->header;
    centers_pub_->publish(ros2_pointcloud);

    if (DEBUG)
      RCLCPP_INFO(this->get_logger(), "[%s] Pattern centers published.", get_name());
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Not enough circle found");
  }

  // // Exit 6: clustering failed
  // if (centers_cloud->points.size() == TARGET_NUM_CIRCLES)
  // {
  //   sensor_msgs::msg::PointCloud2 ros2_pointcloud;
  //   pcl::toROSMsg(*centers_cloud, ros2_pointcloud);
  //   ros2_pointcloud.header = laser_cloud->header;
  //   centers_pub_->publish(ros2_pointcloud);

  //   calibration_interfaces::msg::ClusterCentroids to_send;
  //   to_send.header = laser_cloud->header;
  //   to_send.cluster_iterations = clouds_used_;
  //   to_send.total_iterations = clouds_proc_;
  //   to_send.cloud = ros2_pointcloud;
  //   centers_pub_->publish(to_send);

  //   if (DEBUG)
  //     RCLCPP_INFO(this->get_logger(), "[%s] Pattern centers published.", get_name());

  //   if (save_to_file_)
  //   {
  //     std::vector<pcl::PointXYZ> sorted_centers;
  //     sortPatternCenters(centers_cloud, sorted_centers);
  //     for (std::vector<pcl::PointXYZ>::iterator it = sorted_centers.begin();
  //          it < sorted_centers.end(); ++it)
  //     {
  //       savefile_ << it->x << ", " << it->y << ", " << it->z << ", ";
  //     }
  //     savefile_ << cumulative_cloud_->width;
  //   }
  // }
  // else
  // {
  //   RCLCPP_ERROR(this->get_logger(), "Not enough circle found");
  // }

  // if (save_to_file_)
  // {
  //   savefile_ << std::endl;
  // }

  // // Clear cumulative cloud during warm-up phase
  // if (!WARMUP_DONE)
  // {
  //   cumulative_cloud_->clear();
  //   clouds_proc_ = 0;
  //   clouds_used_ = 0;
  // }
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
      RCLCPP_INFO(this->get_logger(), "[%s] New passthrough_radius_min_ threshold: %f",
                  get_name(), passthrough_radius_min_);
    }
    if (param.get_name() == "passthrough_radius_max")
    {
      passthrough_radius_max_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "[%s] New passthrough_radius_max_ threshold: %f",
                  get_name(), passthrough_radius_max_);
    }
    if (param.get_name() == "circle_radius")
    {
      circle_radius_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "[%s] New pattern circle radius: %f", get_name(), circle_radius_);
    }
    if (param.get_name() == "x")
    {
      axis_[0] = param.as_double();
      RCLCPP_INFO(this->get_logger(), "[%s] New normal axis for plane segmentation: %f, %f, %f",
                  get_name(), axis_[0], axis_[1], axis_[2]);
    }
    if (param.get_name() == "y")
    {
      axis_[1] = param.as_double();
      RCLCPP_INFO(this->get_logger(), "[%s] New normal axis for plane segmentation: %f, %f, %f",
                  get_name(), axis_[0], axis_[1], axis_[2]);
    }
    if (param.get_name() == "z")
    {
      axis_[1] = param.as_double();
      RCLCPP_INFO(this->get_logger(), "[%s] New normal axis for plane segmentation: %f, %f, %f",
                  get_name(), axis_[0], axis_[1], axis_[2]);
    }
    if (param.get_name() == "gap_threshold")
    {
      gap_threshold_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "[%s] New angle threshold: %f", get_name(), gap_threshold_);
    }
    if (param.get_name() == "angle_threshold")
    {
      angle_threshold_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "[%s] New angle threshold: %f", get_name(), angle_threshold_);
    }
  }
  return result;
}

// void LidarPattern::warmup_callback(const std_msgs::msg::Empty::ConstSharedPtr msg)
// {
//   WARMUP_DONE = !WARMUP_DONE;
//   if (WARMUP_DONE)
//   {
//     RCLCPP_INFO(this->get_logger(), "[%s] Warm up done, pattern detection started", get_name());
//   }
//   else
//   {
//     RCLCPP_INFO(this->get_logger(), "[%s] Detection stopped. Warm up mode activated", get_name());
//   }
// }

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<LidarPattern>();

  rclcpp::spin(nh);
  return 0;
}