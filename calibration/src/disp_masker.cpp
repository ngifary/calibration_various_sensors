/*
  disp_masker: Mask the disparity map according to the edges image
*/

#include "disp_masker.hpp"

Masker::Masker() : Node("image_masker")
{
  isfreeobs_ = declare_parameter("isFreeobs", false);
  edges_threshold_ = declare_parameter("edges_threshold", 16);
  disp_in_image_ = declare_parameter("disp_in_image", disp_in_image_);

  mask_img_pub_ = create_publisher<sensor_msgs::msg::Image>("output_img", 1);
  masked_pub_ = create_publisher<stereo_msgs::msg::DisparityImage>("output", 1);

  mask_sub_.subscribe(this, "mask");
  if (disp_in_image_)
  {
    image_sub_.subscribe(this, "image");
    sync_i_ = std::make_shared<message_filters::Synchronizer<ExSyncI>>(max_queue_size_);
    sync_i_->connectInput(image_sub_, mask_sub_);
    sync_i_->registerCallback(std::bind(&Masker::img_callback, this, std::placeholders::_1, std::placeholders::_2));
  }
  else
  {
    disp_sub_.subscribe(this, "image");
    sync_d_ = std::make_shared<message_filters::Synchronizer<ExSyncD>>(max_queue_size_);
    sync_d_->connectInput(disp_sub_, mask_sub_);
    sync_d_->registerCallback(std::bind(&Masker::callback, this, std::placeholders::_1, std::placeholders::_2));
  }
}

Masker::~Masker() {}

void Masker::img_callback(const sensor_msgs::msg::Image::ConstSharedPtr img,
                          const sensor_msgs::msg::Image::ConstSharedPtr ma)
{
  std::shared_ptr<stereo_msgs::msg::DisparityImage> disp = std::make_shared<stereo_msgs::msg::DisparityImage>();
  disp->header = img->header;
  disp->image = *img;
  callback(disp, ma);
}

void Masker::callback(const stereo_msgs::msg::DisparityImage::ConstSharedPtr disp,
                      const sensor_msgs::msg::Image::ConstSharedPtr ma)
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

  disparity32.copyTo(dmat, binary_mask);

  // Publish obstacle disparity
  mask_img_pub_->publish(copy_disp->image);
  masked_pub_->publish(*copy_disp);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<Masker>();
  rclcpp::spin(nh);
  return 0; // Delet
}
