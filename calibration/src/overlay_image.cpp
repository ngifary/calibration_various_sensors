/*
  overlay_image: Display the image from the camera superimpose with the point clouds
*/

#include "overlay_image.hpp"

/**
 * @brief x_norm = x^gamma / (1 + x^gamma)
 *
 * @param range
 * @param gamma
 * @return double
 */
double normalizeRange(double range, double gamma = 1.0)
{
    return std::pow(range, std::abs(gamma)) / (1 + std::pow(range, std::abs(gamma)));
}

OverlayImage::OverlayImage(const rclcpp::NodeOptions &options = rclcpp::NodeOptions{}) : Node("overlay_image", options)
{
    RCLCPP_INFO(this->get_logger(), "Overlay image....");

    initializeParams();

    tf2::Quaternion quaternion;

    quaternion.setEuler(rotation_[2], rotation_[1], rotation_[0]);

    tf2::Matrix3x3 matrix(quaternion);

    tf2::Vector3 vector(translation_[0], translation_[1], translation_[2]);

    transform_ = tf2::Transform(matrix, vector);

    img_pub_ = image_transport::create_publisher(this, "overlay", imageQoS().get_rmw_qos_profile());

    image_sub_.subscribe(this, "image", imageQoS().get_rmw_qos_profile());
    cinfo_sub_.subscribe(this, "camera", imageQoS().get_rmw_qos_profile());
    cloud_sub_.subscribe(this, "cloud", cloudQoS().get_rmw_qos_profile());

    sync_inputs_a_ = std::make_shared<message_filters::Synchronizer<approximate_policy>>(std::min(max_image_queue_size_, max_cloud_queue_size_));
    sync_inputs_a_->connectInput(image_sub_, cinfo_sub_, cloud_sub_);
    sync_inputs_a_->registerCallback(std::bind(&OverlayImage::callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
}

OverlayImage::~OverlayImage()
{
    RCLCPP_INFO(get_logger(), "[%s] Terminating....", get_name());
}

void OverlayImage::initializeParams()
{
    rcl_interfaces::msg::ParameterDescriptor desc;

    // gamma, transform

    // desc.name = "x";
    // desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    // desc.description = "x-coordinate";
    // axis_[0] = declare_parameter(desc.name, 0);

    // desc.name = "y";
    // desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    // desc.description = "y-coordinate";
    // axis_[1] = declare_parameter(desc.name, 0);
}

void OverlayImage::callback(const sensor_msgs::msg::Image::ConstSharedPtr image_msg, const sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_msg, sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
{
    // Save camera info
    cv::Mat cameraMatrix(3, 3, CV_32F);

    cameraMatrix.at<float>(0, 0) = camera_msg->k[0];
    cameraMatrix.at<float>(0, 1) = camera_msg->k[1];
    cameraMatrix.at<float>(0, 2) = camera_msg->k[2];
    cameraMatrix.at<float>(1, 0) = camera_msg->k[3];
    cameraMatrix.at<float>(1, 1) = camera_msg->k[4];
    cameraMatrix.at<float>(1, 2) = camera_msg->k[5];
    cameraMatrix.at<float>(2, 0) = camera_msg->k[6];
    cameraMatrix.at<float>(2, 1) = camera_msg->k[7];
    cameraMatrix.at<float>(2, 2) = camera_msg->k[8];

    cv::Mat distCoeffs(1, camera_msg->d.size(), CV_32F);
    for (unsigned i = 0; i < camera_msg->d.size(); i++)
        distCoeffs.at<float>(0, i) = camera_msg->d[i];

    // Convert image to a matrix
    cv_bridge::CvImageConstPtr cv_img_ptr;
    try
    {
        cv_img_ptr = cv_bridge::toCvShare(image_msg);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat *image = new cv::Mat;
    *image = cv_img_ptr->image;
    cv::Mat *imageCopy = new cv::Mat;
    image->copyTo(*imageCopy);

    // Convert cloud message to pcl cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl(new pcl::PointCloud<pcl::PointXYZ>);

    fromROSMsg(*cloud_msg, *cloud_pcl);

    std::vector<cv::Point3f> *cloud_cv = new std::vector<cv::Point3f>;

    lidar_to_camera(cloud_pcl, *cloud_cv);

    drawPoints(*imageCopy, *cloud_cv, cameraMatrix, distCoeffs);

    // Publish the image.
    sensor_msgs::msg::Image::SharedPtr out_img =
        cv_bridge::CvImage(image_msg->header, image_msg->encoding, *imageCopy).toImageMsg();
    out_img->header.frame_id = image_msg->header.frame_id;
    img_pub_.publish(out_img);
}

void OverlayImage::drawPoints(cv::Mat &image, std::vector<cv::Point3f> &pts, cv::Mat k, cv::Mat d)
{
    std::vector<cv::Point2f> points;

    cv::projectPoints(pts, cv::Mat::zeros(3, 1, CV_64FC1), cv::Mat::zeros(3, 1, CV_64FC1), k, d, points);

    bool debug = true;

    for (auto i = 0; i < points.size(); i++)
    {
        int x = round(points[i].x);
        int y = round(points[i].y);
        if (0.0 < x < image.size().width && 0.0 < y < image.size().height)
        {
            double distance = std::sqrt(std::pow(pts[i].x, 2) + std::pow(pts[i].y, 2) + std::pow(pts[i].z, 2));
            double normal = normalizeRange(distance);

            cv::circle(image, points[i], 10, cv::Scalar(255, 0, 0), -1);
        }
        else
        {
            continue;
        }
    }
}

void OverlayImage::lidar_to_camera(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<cv::Point3f> &points_cv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cv(new pcl::PointCloud<pcl::PointXYZ>());
    tf2::Transform transform;
    tf2::Quaternion quaternion;
    tf2Scalar roll, pitch, yaw;
    roll = M_PI_2;
    pitch = 0.0;
    yaw = M_PI_2;
    quaternion.setRPY(roll, pitch, yaw);

    transform.setRotation(quaternion);

    pcl_ros::transformPointCloud(*cloud, *cloud_cv, transform);

    points_cv.resize(cloud_cv->size());

    for (int i = 0; i < cloud_cv->size(); ++i)
    {
        points_cv[i].x = cloud_cv->points[i].x;
        points_cv[i].y = cloud_cv->points[i].y;
        points_cv[i].z = cloud_cv->points[i].z;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<OverlayImage>();

    rclcpp::spin(nh);
    return 0;
}