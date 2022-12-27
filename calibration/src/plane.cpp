#include "plane.hpp"

SACSegmentator::SACSegmentator() : Node("plane_segmentation"), threshold_(0.1)
{
  inliers_pub_ = this->create_publisher<pcl_msgs::msg::PointIndices>("inliers", 1);
  coeff_pub_ = this->create_publisher<pcl_msgs::msg::ModelCoefficients>("model", 1);
  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "input", 1, std::bind(&SACSegmentator::cloud_callback, this, std::placeholders::_1));

  std::vector<double> axis_param;
  Axis_ = Eigen::Vector3f::Zero();

  declare_parameter("axis", axis_param);
  axis_param = this->get_parameter("axis").as_double_array();

  rcl_interfaces::msg::ParameterDescriptor desc;

  desc.name = "threshold";
  desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  desc.description = "Distance threshold";
  desc.floating_point_range.resize(1);
  auto &floating_point_range = desc.floating_point_range.at(0);
  floating_point_range.from_value = 0.0;
  floating_point_range.to_value = 5.0;
  threshold_ = declare_parameter(desc.name, threshold_);

    if (axis_param.size() == 3)
  {
    for (int i = 0; i < 3; ++i)
    {
      Axis_[i] = axis_param[i];
    }
  }
  else
  {
    Axis_[0] = 0.0;
    Axis_[1] = 1.0;
    Axis_[2] = 0.0;
  }

  eps_angle_ = declare_parameter("eps_angle", 0.35);

  // 0: SACMODEL_NORMAL_PLANE
  // 1: SACMODEL_PARALLEL_PLANE
  sac_segmentation_type_ = declare_parameter("segmentation_type", 0);

  if (sac_segmentation_type_ == 0)
  {
    RCLCPP_INFO(get_logger(), "Searching for planes perpendicular to %f %f %f", Axis_[0],
                Axis_[1], Axis_[2]);
  }
  else
  {
    RCLCPP_INFO(get_logger(), "Searching for planes parallel to %f %f %f", Axis_[0], Axis_[1],
                Axis_[2]);
  }

  ret_ = add_on_set_parameters_callback(std::bind(&SACSegmentator::param_callback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Segmentator ready");
}

SACSegmentator::~SACSegmentator() {}

void SACSegmentator::cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &input)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*input, *cloud);

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;

  seg.setModelType(sac_segmentation_type_
                       ? pcl::SACMODEL_PARALLEL_PLANE
                       : pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setDistanceThreshold(threshold_);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setAxis(Axis_);
  seg.setOptimizeCoefficients(true);

  seg.setEpsAngle(eps_angle_);
  seg.setMaxIterations(250);

  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

  seg.setInputCloud(cloud->makeShared());
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.size() == 0)
  {
    RCLCPP_WARN(this->get_logger(), "Could not estimate a planar model for the given dataset.");
    return;
  }

  pcl_msgs::msg::PointIndices p_ind;

  pcl_conversions::moveFromPCL(*inliers, p_ind);
  p_ind.header = input->header;

  pcl_msgs::msg::ModelCoefficients m_coeff;

  pcl_conversions::moveFromPCL(*coefficients, m_coeff);
  m_coeff.header = input->header;

  inliers_pub_->publish(p_ind);
  coeff_pub_->publish(m_coeff);
}

rcl_interfaces::msg::SetParametersResult SACSegmentator::param_callback(const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  for (const auto &param : parameters)
  {
    if (param.get_name() == "threshold")
    {
      threshold_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "New distance threshold: %f", threshold_);
    }
  }
  return result;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<SACSegmentator>();
  rclcpp::spin(nh);
  return 0;
}
