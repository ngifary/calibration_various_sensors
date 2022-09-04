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
  plane: Find a plane model in the cloud using RANSAC
*/

#include <memory.h>
// #include "dynamic_reconfigure/server.h"
#include "pcl/filters/filter.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl_conversions/pcl_conversions.hpp"
// #include <pcl_msgs/ModelCoefficients.h>
#include "pcl_msgs/msg/model_coefficients.hpp"
// #include <pcl_msgs/PointIndices.h>
#include "pcl_msgs/msg/point_indices.hpp"
// #include <ros/ros.h>
#include "rclcpp/rclcpp.hpp"
// #include <sensor_msgs/PointCloud2.h>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_cloud2.h"
// #include "velo2cam_calibration/PlaneConfig.h"

class SACSegmentator : public rclcpp::Node
{
public:
  // SACSegmentator() : nh_("~"), threshold_(0.1)
  SACSegmentator() : Node("plane_segmentation"), threshold_(0.1)
  {
    inliers_pub = this->create_publisher<pcl_msgs::msg::PointIndices>("inliers", 1);
    coeff_pub = this->create_publisher<pcl_msgs::msg::ModelCoefficients>("model", 1);
    auto cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "input", 1, std::bind(&SACSegmentator::cloud_callback, this, std::placeholders::_1));

    std::vector<double> axis_param;
    Axis = Eigen::Vector3f::Zero();

    axis_param = this->get_parameter("axis").as_double_array();

    if (axis_param.size() == 3)
    {
      for (int i = 0; i < 3; ++i)
      {
        Axis[i] = axis_param[i];
      }
    }
    else
    {
      Axis[0] = 0.0;
      Axis[1] = 1.0;
      Axis[2] = 0.0;
    }

    eps_angle_ = this->declare_parameter("eps_angle", 0.35);

    // 0: SACMODEL_NORMAL_PLANE
    // 1: SACMODEL_PARALLEL_PLANE
    sac_segmentation_type_ = this->declare_parameter("segmentation_type", 0);

    if (sac_segmentation_type_ == 0)
    {
      RCLCPP_INFO(this->get_logger(), "Searching for planes perpendicular to %f %f %f", Axis[0],
               Axis[1], Axis[2]);
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Searching for planes parallel to %f %f %f", Axis[0], Axis[1],
               Axis[2]);
    }

    ret = this->add_on_set_parameters_callback(std::bind(&SACSegmentator::param_callback, this, std::placeholders::_1));
    // f = boost::bind(&SACSegmentator::param_callback, this, _1, _2);
    // server.setCallback(f);

    RCLCPP_INFO(this->get_logger(), "Segmentator ready");
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &input)
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
    seg.setAxis(Axis);
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

    inliers_pub->publish(p_ind);
    coeff_pub->publish(m_coeff);
  }

  rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  for (const auto &param : parameters)
  {
    if (param.get_name() == "threshold_")
    {
      threshold_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "New distance threshold: %f", threshold_);
    }
  }
  return result;
}
  // void param_callback(velo2cam_calibration::PlaneConfig &config,
  //                     uint32_t level)
  // {
  //   threshold_ = config.threshold;
  //   ROS_INFO("New distance threshold: %f", threshold_);
  // }
private:
  // std::shared_ptr<rclcpp::Node> nh;
  // ros::NodeHandle nh_;
  rclcpp::Publisher<pcl_msgs::msg::PointIndices>::SharedPtr inliers_pub;
  rclcpp::Publisher<pcl_msgs::msg::ModelCoefficients>::SharedPtr coeff_pub;

  // rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;
  int sac_segmentation_type_;
  double eps_angle_;

  Eigen::Vector3f Axis;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr ret;
  // dynamic_reconfigure::Server<velo2cam_calibration::PlaneConfig> server;
  // dynamic_reconfigure::Server<velo2cam_calibration::PlaneConfig>::CallbackType
  //     f;
  double threshold_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto seg = std::make_shared<SACSegmentator>();
  rclcpp::spin(seg);
}
