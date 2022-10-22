/*
  disp_masker: Mask the disparity map according to the edges image
*/

#include "cv_bridge/cv_bridge.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/exact_time.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "stereo_msgs/msg/disparity_image.hpp"

#include "opencv2/opencv.hpp"

class Masker : public rclcpp::Node
{
public:
  Masker() : Node("image_masker"),
             image_sub_(this, "image"),
             mask_sub_(this, "mask"),
             sync(ExSync(100), image_sub_, mask_sub_)
  {
    masked_pub_ = this->create_publisher<stereo_msgs::msg::DisparityImage>("output", 1);

    isfreeobs_ = this->declare_parameter("isFreeobs", false);
    edges_threshold_ = this->declare_parameter("edges_threshold", 16);

    sync.registerCallback(std::bind(&Masker::callback, this, std::placeholders::_1, std::placeholders::_2));
  }

  void callback(const stereo_msgs::msg::DisparityImage::ConstSharedPtr &disp,
                const sensor_msgs::msg::Image::ConstSharedPtr &ma)
  {
    cv::Mat mask, binary_mask, output;
    cv_bridge::CvImageConstPtr cv_im;

    try
    {
      cv_im = cv_bridge::toCvShare(disp->image, disp,
                                   sensor_msgs::image_encodings::TYPE_32FC1);
      mask = cv_bridge::toCvShare(ma, "mono8")->image;
    }
    catch (cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "CvBridge failed");
    }

    static cv::Mat disparity32(cv_im->image.rows, cv_im->image.cols, CV_32FC1);
    disparity32 = cv_im->image;

    if (isfreeobs_)
    {
      const static int OBSTACLE_LABEL = 32;
      cv::Mat obs_pattern(mask.rows, mask.cols, CV_8UC1,
                          cv::Scalar(OBSTACLE_LABEL));
      cv::bitwise_and(mask, obs_pattern, binary_mask);
      binary_mask = binary_mask * (255.0 / OBSTACLE_LABEL);
    }
    else
    {
      cv::threshold(mask, binary_mask, edges_threshold_, 255, 0);
    }

    // Copy input disparity to another DisparityImage variable
    stereo_msgs::msg::DisparityImage::SharedPtr copy_disp =
        std::make_shared<stereo_msgs::msg::DisparityImage>();
    copy_disp->valid_window.x_offset = disp->valid_window.x_offset;
    copy_disp->valid_window.y_offset = disp->valid_window.y_offset;
    copy_disp->valid_window.width = disp->valid_window.width;
    copy_disp->valid_window.height = disp->valid_window.height;
    copy_disp->header = disp->header;
    copy_disp->image.header = disp->header;
    copy_disp->image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    copy_disp->image.height = disp->image.height;
    copy_disp->image.width = disp->image.width;
    copy_disp->image.step = disp->image.step;
    copy_disp->t = disp->t;
    copy_disp->f = disp->f;

    copy_disp->min_disparity = disp->min_disparity;
    copy_disp->max_disparity = disp->max_disparity;
    copy_disp->delta_d = disp->delta_d;

    // Create cv::Mat from the copies DisparityImage input
    sensor_msgs::msg::Image &d_image = copy_disp->image;
    d_image.height = disparity32.rows;
    d_image.width = disparity32.cols;
    d_image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    d_image.step = d_image.width * sizeof(float);

    d_image.data.resize(d_image.step * d_image.height);

    cv::Mat_<float> dmat(d_image.height, d_image.width,
                         (float *)&d_image.data[0], d_image.step);

    // Check data
    assert(dmat.data == &d_image.data[0]);
    // ROS_ASSERT(dmat.data == &d_image.data[0]);

    disparity32.copyTo(dmat, binary_mask);

    // Publish obstacle disparity
    masked_pub_->publish(*copy_disp.get());
  }

private:
  message_filters::Subscriber<stereo_msgs::msg::DisparityImage> image_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> mask_sub_;
  rclcpp::Publisher<stereo_msgs::msg::DisparityImage>::SharedPtr masked_pub_;

  bool isfreeobs_;
  int edges_threshold_;

  typedef message_filters::sync_policies::ExactTime<stereo_msgs::msg::DisparityImage,
                                                    sensor_msgs::msg::Image>
      ExSync;
  message_filters::Synchronizer<ExSync> sync;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto seg = std::make_shared<Masker>();
  rclcpp::spin(seg);
  // ros::init(argc, argv, "image_masker");

  // Masker im;

  // ROS_INFO("Ready");
  // ros::spin();
  return 0; // Delet
}
