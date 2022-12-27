/*
  mono_qr_pattern: Find the circle centers in the color image by making use of
  the ArUco markers
*/

#include "mono_qr_pattern.hpp"

/**
 */
static bool readDetectorParameters(std::string filename, cv::Ptr<cv::aruco::DetectorParameters> &params)
{
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  if (!fs.isOpened())
    return false;
  fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
  fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
  fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
  fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
  fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
  fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
  fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
  fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
  fs["minDistanceToBorder"] >> params->minDistanceToBorder;
  fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
  fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
  fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
  fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
  fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
  fs["markerBorderBits"] >> params->markerBorderBits;
  fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
  fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
  fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
  fs["minOtsuStdDev"] >> params->minOtsuStdDev;
  fs["errorCorrectionRate"] >> params->errorCorrectionRate;
  return true;
}

MonoQRPattern::MonoQRPattern() : Node("mono_qr_pattern")
{
  RCLCPP_INFO(get_logger(), "[%s] Starting....", get_name());

  // Initialize QR dictionary
  dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

  if (DEBUG)
  {
    centers_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("centers_cloud", 1);
  }
  final_pub_ = this->create_publisher<calibration::msg::CircleCentroids>("centers_msg", 1);

  initializeParams();

  std::string csv_name;

  csv_name = this->declare_parameter("csv_name", "mono_pattern_" + currentDateTime() + ".csv");

  std::string image_topic, cinfo_topic;
  image_topic = this->declare_parameter("image_topic", "/stereo_camera/left/image_rect_color");
  cinfo_topic = this->declare_parameter("cinfo_topic", "/stereo_camera/left/camera_info");

  rclcpp::QoS qos(10);
  auto rmw_qos_profile = qos.get_rmw_qos_profile();

  image_sub_.subscribe(this, image_topic, rmw_qos_profile);
  cinfo_sub_.subscribe(this, cinfo_topic, rmw_qos_profile);

  sync_ = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>>>(max_queue_size_);

  sync_->connectInput(image_sub_, cinfo_sub_);

  sync_->registerCallback(std::bind(&MonoQRPattern::imageCallback, this, std::placeholders::_1, std::placeholders::_2));

  // ROS param callback
  auto ret = this->add_on_set_parameters_callback(std::bind(&MonoQRPattern::param_callback, this, std::placeholders::_1));

  // Just for statistics
  if (save_to_file_)
  {
    std::ostringstream os;
    os << getenv("HOME") << "/calibration_tests/" << csv_name;
    if (save_to_file_)
    {
      if (DEBUG)
      {
        RCLCPP_INFO(get_logger(), "Opening %s", os.str().c_str());
        savefile_.open(os.str().c_str());
        savefile_ << "cent1_x; cent1_y; cent1_z; "
                     "cent2_x; cent2_y; cent2_z; "
                     "cent3_x; cent3_y; cent3_z; "
                     "cent4_x; cent4_y; cent4_z; it"
                  << std::endl;
      }
    }
  }
}

MonoQRPattern::~MonoQRPattern()
{
  RCLCPP_INFO(get_logger(), "[%s] Terminating....", get_name());
}

void MonoQRPattern::initializeParams()
{
  rcl_interfaces::msg::ParameterDescriptor desc;

  desc.name = "config_file";
  desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  desc.description = "Parameter config file for detecting ArUco";
  config_file_ = declare_parameter(desc.name, "detector_params");

  desc.name = "delta_width_circles";
  desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  desc.description = "distance from left circles centre to right circles centre (m)";
  delta_width_circles_ = declare_parameter(desc.name, 0.5);

  desc.name = "delta_height_circles";
  desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  desc.description = "distance from top circles centre to bottom circles centre (m)";
  delta_height_circles_ = declare_parameter(desc.name, 0.4);

  desc.name = "marker_size";
  desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  desc.description = "Size of the marker (m)";
  marker_size_ = declare_parameter(desc.name, 0.2);

  desc.name = "delta_width_qr_center";
  desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  desc.description = "width increment from target center to qr center (m)";
  delta_width_qr_center_ = declare_parameter(desc.name, 0.55);

  desc.name = "delta_height_qr_center";
  desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  desc.description = "height increment from target center to qr center (m)";
  delta_height_qr_center_ = declare_parameter(desc.name, 0.35);

  desc.name = "min_detected_markers";
  desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  desc.description = "minimum marker to be detected (-)";
  min_detected_markers_ = declare_parameter(desc.name, 3);

  desc.name = "save_to_file";
  desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  desc.description = "save result to a file";
  save_to_file_ = declare_parameter(desc.name, false);
}

cv::Point2f MonoQRPattern::projectPointDist(cv::Point3f pt_cv, const cv::Mat intrinsics, const cv::Mat distCoeffs)
{
  // Project a 3D point taking into account distortion
  std::vector<cv::Point3f> input{pt_cv};
  std::vector<cv::Point2f> projectedPoints;
  projectedPoints.resize(1);
  projectPoints(input, cv::Mat::zeros(3, 1, CV_64FC1), cv::Mat::zeros(3, 1, CV_64FC1),
                intrinsics, distCoeffs, projectedPoints);
  return projectedPoints[0];
}

Eigen::Vector3f MonoQRPattern::mean(pcl::PointCloud<pcl::PointXYZ>::Ptr cumulative_cloud)
{
  double x = 0, y = 0, z = 0;
  int npoints = cumulative_cloud->points.size();
  for (pcl::PointCloud<pcl::PointXYZ>::iterator pt =
           cumulative_cloud->points.begin();
       pt < cumulative_cloud->points.end(); pt++)
  {
    x += (pt->x) / npoints;
    y += (pt->y) / npoints;
    z += (pt->z) / npoints;
  }
  return Eigen::Vector3f(x, y, z);
}

Eigen::Matrix3f
MonoQRPattern::covariance(pcl::PointCloud<pcl::PointXYZ>::Ptr cumulative_cloud, Eigen::Vector3f means)
{
  double x = 0, y = 0, z = 0;
  int npoints = cumulative_cloud->points.size();
  std::vector<Eigen::Vector3f> points;

  for (pcl::PointCloud<pcl::PointXYZ>::iterator pt =
           cumulative_cloud->points.begin();
       pt < cumulative_cloud->points.end(); pt++)
  {
    Eigen::Vector3f p(pt->x, pt->y, pt->z);
    points.push_back(p);
  }

  Eigen::Matrix3f covarianceMatrix(3, 3);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
    {
      covarianceMatrix(i, j) = 0.0;
      for (int k = 0; k < npoints; k++)
      {
        covarianceMatrix(i, j) +=
            (means[i] - points[k][i]) * (means[j] - points[k][j]);
      }
      covarianceMatrix(i, j) /= npoints - 1;
    }
  return covarianceMatrix;
}

void MonoQRPattern::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg,
                                  const sensor_msgs::msg::CameraInfo::ConstSharedPtr left_info)
{
  // Convert message to a matrix
  cv_bridge::CvImageConstPtr cv_img_ptr;
  try
  {
    cv_img_ptr = cv_bridge::toCvShare(msg);
  }
  catch (cv_bridge::Exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat image = cv_img_ptr->image;
  cv::Mat imageCopy;
  image.copyTo(imageCopy);

  // Assign camera_info to a matrix
  sensor_msgs::msg::CameraInfo::SharedPtr cinfo(new sensor_msgs::msg::CameraInfo(*left_info));

  // TODO Not needed at each frame -> Move it to separate callback
  cv::Mat cameraMatrix(3, 3, CV_32F);
  // Note that camera matrix is K, not P; both are interchangeable only if D is zero
  cameraMatrix.at<float>(0, 0) = cinfo->k[0];
  cameraMatrix.at<float>(0, 1) = cinfo->k[1];
  cameraMatrix.at<float>(0, 2) = cinfo->k[2];
  cameraMatrix.at<float>(1, 0) = cinfo->k[3];
  cameraMatrix.at<float>(1, 1) = cinfo->k[4];
  cameraMatrix.at<float>(1, 2) = cinfo->k[5];
  cameraMatrix.at<float>(2, 0) = cinfo->k[6];
  cameraMatrix.at<float>(2, 1) = cinfo->k[7];
  cameraMatrix.at<float>(2, 2) = cinfo->k[8];

  cv::Mat distCoeffs(1, cinfo->d.size(), CV_32F);
  for (unsigned i = 0; i < cinfo->d.size(); i++)
    distCoeffs.at<float>(0, i) = cinfo->d[i];
  // TODO End of block to move

  // Create vector of markers corners. 4 markers * 4 corners
  // Markers order:
  // 0-------1
  // |       |
  // |   C   |
  // |       |
  // 3-------2

  // WARNING: IDs are in different order:
  // Marker 0 -> aRuCo ID: 1
  // Marker 1 -> aRuCo ID: 2
  // Marker 2 -> aRuCo ID: 4
  // Marker 3 -> aRuCo ID: 3

  std::vector<std::vector<cv::Point3f>> boardCorners;
  std::vector<cv::Point3f> boardCircleCenters;
  float width = delta_width_qr_center_;
  float height = delta_height_qr_center_;
  float circle_width = delta_width_circles_ / 2.;
  float circle_height = delta_height_circles_ / 2.;
  boardCorners.resize(4);
  for (int i = 0; i < 4; ++i)
  {
    int x_qr_center =
        (i % 3) == 0 ? -1 : 1; // x distances are substracted for QRs on the
                               // left, added otherwise
    int y_qr_center =
        (i < 2) ? 1 : -1; // y distances are added for QRs above target's
                          // center, substracted otherwise
    float x_center = x_qr_center * width;
    float y_center = y_qr_center * height;

    cv::Point3f circleCenter3d(x_qr_center * circle_width,
                               y_qr_center * circle_height, 0);
    boardCircleCenters.push_back(circleCenter3d);
    for (int j = 0; j < 4; ++j)
    {
      int x_qr = (j % 3) == 0 ? -1 : 1; // x distances are added for QRs 0 and
                                        // 3, substracted otherwise
      int y_qr = (j < 2) ? 1 : -1;      // y distances are added for QRs 0 and 1,
                                        // substracted otherwise
      cv::Point3f pt3d(x_center + x_qr * marker_size_ / 2.,
                       y_center + y_qr * marker_size_ / 2., 0);
      boardCorners[i].push_back(pt3d);
    }
  }

  std::vector<int> boardIds{1, 2, 4, 3}; // IDs order as explained above
  cv::Ptr<cv::aruco::Board> board =
      cv::aruco::Board::create(boardCorners, dictionary_, boardIds);

  cv::Ptr<cv::aruco::DetectorParameters> parameters =
      cv::aruco::DetectorParameters::create();

  // Get config file
  std::string config_path = ament_index_cpp::get_package_share_directory("calibration") + "/config/" + config_file_ + ".yaml";
  bool readOk = readDetectorParameters(config_path, parameters);
  if (!readOk)
  {
    RCLCPP_WARN(get_logger(), "Invalid detector parameters file");
    return;
  }
  parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

  // Detect markers
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  cv::aruco::detectMarkers(image, dictionary_, corners, ids, parameters);

  // Draw detections if at least one marker detected
  if (ids.size() > 0)
    cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);

  cv::Vec3d rvec(0, 0, 0), tvec(0, 0, 0); // Vectors to store initial guess

  // Compute initial guess as average of individual markers poses
  if (ids.size() >= min_detected_markers_ && ids.size() <= TARGET_NUM_CIRCLES)
  {
    RCLCPP_INFO(this->get_logger(), "%lu marker(s) of %d found!. Processing frame...", ids.size(),
                TARGET_NUM_CIRCLES);
    // Estimate 3D position of the markers
    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::Vec3f rvec_sin, rvec_cos;
    cv::aruco::estimatePoseSingleMarkers(corners, marker_size_, cameraMatrix,
                                         distCoeffs, rvecs, tvecs);

    // Draw markers' axis and centers in color image (Debug purposes)
    for (unsigned i = 0; i < ids.size(); i++)
    {
      double x = tvecs[i][0];
      double y = tvecs[i][1];
      double z = tvecs[i][2];

      cv::Point3f pt_cv(x, y, z);
      cv::Point2f uv;
      uv = projectPointDist(pt_cv, cameraMatrix, distCoeffs);

      cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvecs[i],
                          tvecs[i], 0.1);

      // Accumulate pose for initial guess
      tvec[0] += tvecs[i][0];
      tvec[1] += tvecs[i][1];
      tvec[2] += tvecs[i][2];
      rvec_sin[0] += sin(rvecs[i][0]);
      rvec_sin[1] += sin(rvecs[i][1]);
      rvec_sin[2] += sin(rvecs[i][2]);
      rvec_cos[0] += cos(rvecs[i][0]);
      rvec_cos[1] += cos(rvecs[i][1]);
      rvec_cos[2] += cos(rvecs[i][2]);
    }

    // Compute average pose. Rotation computed as atan2(sin/cos)
    tvec = tvec / int(ids.size());
    rvec_sin = rvec_sin / int(ids.size()); // Average sin
    rvec_cos = rvec_cos / int(ids.size()); // Average cos
    rvec[0] = atan2(rvec_sin[0], rvec_cos[0]);
    rvec[1] = atan2(rvec_sin[1], rvec_cos[1]);
    rvec[2] = atan2(rvec_sin[2], rvec_cos[2]);

    pcl::PointCloud<pcl::PointXYZ>::Ptr centers_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr candidates_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);

    // Estimate 3D position of the board using detected markers
    int valid = cv::aruco::estimatePoseBoard(corners, ids, board, cameraMatrix,
                                             distCoeffs, rvec, tvec, true);

    cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvec, tvec, 0.2);

    // Build transformation matrix to calibration target axis
    cv::Mat R(3, 3, cv::DataType<float>::type);
    cv::Rodrigues(rvec, R);

    cv::Mat t = cv::Mat::zeros(3, 1, CV_32F);
    t.at<float>(0) = tvec[0];
    t.at<float>(1) = tvec[1];
    t.at<float>(2) = tvec[2];

    cv::Mat board_transform = cv::Mat::eye(3, 4, CV_32F);
    R.copyTo(board_transform.rowRange(0, 3).colRange(0, 3));
    t.copyTo(board_transform.rowRange(0, 3).col(3));

    // Compute coordintates of circle centers
    for (unsigned i = 0; i < boardCircleCenters.size(); ++i)
    {
      cv::Mat mat = cv::Mat::zeros(4, 1, CV_32F);
      mat.at<float>(0, 0) = boardCircleCenters[i].x;
      mat.at<float>(1, 0) = boardCircleCenters[i].y;
      mat.at<float>(2, 0) = boardCircleCenters[i].z;
      mat.at<float>(3, 0) = 1.0;

      // Transform center to target coords
      cv::Mat mat_qr = board_transform * mat;
      cv::Point3f center3d;
      center3d.x = mat_qr.at<float>(0, 0);
      center3d.y = mat_qr.at<float>(1, 0);
      center3d.z = mat_qr.at<float>(2, 0);

      // Draw center (DEBUG)
      cv::Point2f uv;
      uv = projectPointDist(center3d, cameraMatrix, distCoeffs);
      circle(imageCopy, uv, 10, cv::Scalar(0, 255, 0), -1);

      // Add center to list
      pcl::PointXYZ qr_center;
      qr_center.x = center3d.x;
      qr_center.y = center3d.y;
      qr_center.z = center3d.z;
      candidates_cloud->push_back(qr_center);
    }

    /**
      NOTE: This is included here in the same way as the rest of the modalities
    to avoid obvious misdetections, which sometimes happened in our experiments.
    In this modality, it should be impossible to have more than a set of
    candidates, but we keep the ability of handling different combinations for
    eventual future extensions.

      Geometric consistency check
      At this point, circles' center candidates have been computed
    (found_centers). Now we need to select the set of 4 candidates that best fit
    the calibration target geometry. To that end, the following steps are
    followed: 1) Create a cloud with 4 points representing the exact geometry of
    the calibration target 2) For each possible set of 4 points: compute
    similarity score 3) Rotate back the candidates with the highest score to
    their original position in the cloud, and add them to cumulative cloud
    **/
    std::vector<std::vector<int>> groups;
    comb(candidates_cloud->size(), TARGET_NUM_CIRCLES, groups);
    double groups_scores[groups.size()]; // -1: invalid; 0-1 normalized score
    for (unsigned i = 0; i < groups.size(); ++i)
    {
      std::vector<pcl::PointXYZ> candidates;
      // Build candidates set
      for (unsigned j = 0; j < groups[i].size(); ++j)
      {
        pcl::PointXYZ center;
        center.x = candidates_cloud->at(groups[i][j]).x;
        center.y = candidates_cloud->at(groups[i][j]).y;
        center.z = candidates_cloud->at(groups[i][j]).z;
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
      // Exit: No candidates fit target's geometry
      RCLCPP_WARN(this->get_logger(),
                  "[%s] Unable to find a candidate set that matches target's "
                  "geometry",
                  get_name());
      return;
    }

    for (unsigned j = 0; j < groups[best_candidate_idx].size(); ++j)
    {
      centers_cloud->push_back(
          candidates_cloud->at(groups[best_candidate_idx][j]));
    }

    if (DEBUG)
    { // Draw centers
      for (unsigned i = 0; i < centers_cloud->size(); i++)
      {
        cv::Point3f pt_circle1(centers_cloud->at(i).x, centers_cloud->at(i).y,
                               centers_cloud->at(i).z);
        cv::Point2f uv_circle1;
        uv_circle1 = projectPointDist(pt_circle1, cameraMatrix, distCoeffs);
        circle(imageCopy, uv_circle1, 2, cv::Scalar(255, 0, 255), -1);
      }
    }

    if (centers_cloud->points.size() == TARGET_NUM_CIRCLES)
    {
      sensor_msgs::msg::PointCloud2 ros_pointcloud;
      pcl::toROSMsg(*centers_cloud, ros_pointcloud); // circles_cloud
      ros_pointcloud.header = msg->header;
      if (DEBUG)
      {
        centers_pub_->publish(ros_pointcloud);
        RCLCPP_INFO(this->get_logger(), "[%s] Pattern centers published.", get_name());
      }
      calibration::msg::CircleCentroids to_send;
      to_send.header = msg->header;
      to_send.sensor_type = to_send.MONOCAMERA;
      to_send.image = *msg;
      to_send.camera_info = *left_info;
      to_send.centers = ros_pointcloud;
      final_pub_->publish(to_send);

      if (save_to_file_)
      {
        iter_++;
        pcl::PointCloud<pcl::PointXYZ>::Ptr aux(new pcl::PointCloud<pcl::PointXYZ>);
        camera_to_lidar(centers_cloud, aux);
        *aux = sortPatternCenters(aux);
        for (pcl::PointCloud<pcl::PointXYZ>::iterator it = aux->begin(); it < aux->end(); ++it)
        {
          savefile_ << it->x << "; " << it->y << "; " << it->z << "; ";
        }
        savefile_ << iter_;
        RCLCPP_INFO(get_logger(), "Saving data of the %i-th iteration", iter_);
      }
      if (save_to_file_)
        savefile_ << std::endl;
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Not enough circle found");
    }
  }
  else
  { // Markers found != TARGET_NUM_CIRCLES
    RCLCPP_WARN(this->get_logger(), "%lu marker(s) found, %d expected. Skipping frame...", ids.size(),
                TARGET_NUM_CIRCLES);
  }

  if (DEBUG)
  {
    cv::namedWindow("out", cv::WINDOW_NORMAL);
    cv::Mat imageCopy_resize;
    cv::Size size(1024, 750);
    cv::resize(imageCopy, imageCopy_resize, size);
    cv::imshow("out", imageCopy_resize);
    cv::waitKey(1);
  }
}

rcl_interfaces::msg::SetParametersResult
MonoQRPattern::param_callback(const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  for (const auto &param : parameters)
  {
    if (param.get_name() == "marker_size")
    {
      marker_size_ = param.as_double();
      RCLCPP_INFO(get_logger(), "[%s] New marker_size_: %f", get_name(), marker_size_);
    }
    if (param.get_name() == "delta_width_qr_center")
    {
      delta_width_qr_center_ = param.as_double();
      RCLCPP_INFO(get_logger(), "[%s] New marker_size_: %f", get_name(), delta_width_qr_center_);
    }
    if (param.get_name() == "delta_height_qr_center")
    {
      delta_height_qr_center_ = param.as_double();
      RCLCPP_INFO(get_logger(), "[%s] New marker_size_: %f", get_name(), delta_height_qr_center_);
    }
  }
  return result;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<MonoQRPattern>();
  rclcpp::spin(nh);
  cv::destroyAllWindows();
  return 0;
}
