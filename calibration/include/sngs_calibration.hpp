#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "pcl/registration/transformation_estimation_svd.h"
#include "pcl_conversions/pcl_conversions.hpp"
#include "pcl_ros/transforms.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/transform_datatypes.h"
#include "tinyxml2.h"
#include "laser2cam_utils.h"
#include "opencv2/opencv.hpp"
#include "calibration_interfaces/msg/cluster_centroids.hpp"

#include <chrono>
#include <cinttypes>
#include <limits>

#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/image.hpp"

class Registration : public rclcpp::Node
{

public:
    typedef Eigen::Matrix<double, 12, 12> Matrix12d;
    typedef Eigen::Matrix<double, 12, 1> Vector12d;

    typedef message_filters::sync_policies::ExactTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2> exact_policy;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2> approximate_policy;

    Registration(const rclcpp::NodeOptions &options);
    ~Registration();

    bool isFinished()
    {
        return calibration_ended_;
    }

    rclcpp::QoS
    cloudQoS() const
    {
        rclcpp::QoS qos(max_queue_size_);
        return qos;
    }

private:
    void initializeParams();
    void calibrateExtrinsics(int seek_iter);
    void callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr sensor1_centroids, const sensor_msgs::msg::PointCloud2::ConstSharedPtr sensor2_centroids);

    // Pubs Declaration
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clusters_sensor2_pub_, clusters_sensor1_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr both_centroids_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr colour_sensor2_pub_, colour_sensor1_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr iterations_pub_;

    // Subs Declaration
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sensor1_sub_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sensor2_sub_;

    // Sensor 1 Declaration
    bool sensor1Received_, sensor2Received_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr sensor1_cloud_, sensor2_cloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr isensor1_cloud_, isensor2_cloud_;

    /** \brief Synchronized inputs.*/
    std::shared_ptr<message_filters::Synchronizer<exact_policy>> sync_inputs_e_;
    std::shared_ptr<message_filters::Synchronizer<approximate_policy>> sync_inputs_a_;

    /** \brief TF listener object. */
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    int nFrames_;

    int max_queue_size_ = 10;

    std::vector<pcl::PointXYZ> sensor1_vector_{4}, sensor2_vector_{4};

    geometry_msgs::msg::TransformStamped tf_sensor1_sensor2_;
    tf2::Stamped<tf2::Transform> tf2_sensor1_s2_;

    std::string sensor1_frame_id_ = "";
    std::string sensor1_rotated_frame_id_ = "";
    std::string sensor2_frame_id_ = "";
    std::string sensor2_rotated_frame_id_ = "";

    tf2::Transform transf_;

    std::vector<std::vector<std::tuple<int, int, pcl::PointCloud<pcl::PointXYZ>,
                                       std::vector<pcl::PointXYZ>>>>
        sensor1_buffer_;
    std::vector<std::vector<std::tuple<int, int, pcl::PointCloud<pcl::PointXYZ>,
                                       std::vector<pcl::PointXYZ>>>>
        sensor2_buffer_;

    long int sensor1_count_, sensor2_count_;

    std::ofstream savefile_;

    // Internal State Definistion
    bool calibration_ended_;

    int S1_WARMUP_COUNT = 0, S2_WARMUP_COUNT = 0;
    bool S1_WARMUP_DONE = false, S2_WARMUP_DONE = false;
    unsigned TARGET_POSITIONS_COUNT = 0;
    int TARGET_ITERATIONS = 30;

    // Node Parameters Declaration
    std::string csv_name_;
    bool approximate_time_ = true;
    bool sync_iterations_;
    bool save_to_file_;
    bool publish_tf_;
    bool is_sensor1_cam_, is_sensor2_cam_;
    bool skip_warmup_, single_pose_mode_;
    bool results_every_pose_;
};
