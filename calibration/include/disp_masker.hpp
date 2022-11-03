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
    typedef message_filters::sync_policies::ExactTime<stereo_msgs::msg::DisparityImage,
                                                      sensor_msgs::msg::Image>
        ExSyncD;

    typedef message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image,
                                                      sensor_msgs::msg::Image>
        ExSyncI;

    Masker();
    ~Masker();

    rclcpp::QoS
    imageQoS() const
    {
        rclcpp::QoS qos(max_queue_size_);
        return qos;
    }

private:
    void img_callback(const sensor_msgs::msg::Image::ConstSharedPtr disp, const sensor_msgs::msg::Image::ConstSharedPtr ma);
    void disp_callback(const stereo_msgs::msg::DisparityImage::ConstSharedPtr disp, const sensor_msgs::msg::Image::ConstSharedPtr ma);
    void callback(const stereo_msgs::msg::DisparityImage::ConstSharedPtr disp, const sensor_msgs::msg::Image::ConstSharedPtr ma);

    message_filters::Subscriber<stereo_msgs::msg::DisparityImage> disp_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> mask_sub_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_img_pub_;
    rclcpp::Publisher<stereo_msgs::msg::DisparityImage>::SharedPtr masked_pub_;

    std::shared_ptr<message_filters::Synchronizer<ExSyncD>> sync_d_;
    std::shared_ptr<message_filters::Synchronizer<ExSyncI>> sync_i_;

    int max_queue_size_ = 100;
    bool isfreeobs_;
    int edges_threshold_;
    bool disp_in_image_ = false;
};
