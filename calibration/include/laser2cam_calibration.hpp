#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/twist.hpp"
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

#include "std_srvs/srv/empty.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

class Registration : public rclcpp::Node
{
public:
    typedef Eigen::Matrix<double, 12, 12> Matrix12d;
    typedef Eigen::Matrix<double, 12, 1> Vector12d;

    Registration(const rclcpp::NodeOptions &options);
    ~Registration();

    bool isFinished()
    {
        return calibration_ended;
    }

private:
    void calibrateExtrinsics(int seek_iter);
    void sensor1_callback(const calibration_interfaces::msg::ClusterCentroids::ConstSharedPtr sensor1_centroids);
    void sensor2_callback(const calibration_interfaces::msg::ClusterCentroids::ConstSharedPtr sensor2_centroids);
    /**
     * \brief Force immediate unsubscription of this subscriber from its topic
     */
    void unsubscribe()
    {
        sensor1_sub.reset();
        sensor2_sub.reset();
    }

    // Pubs Declaration
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clusters_sensor2_pub, clusters_sensor1_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr colour_sensor2_pub, colour_sensor1_pub;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr sensor_switch_pub;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr iterations_pub;

    // Subs Declaration
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Subscription<calibration_interfaces::msg::ClusterCentroids>::SharedPtr sensor1_sub;
    rclcpp::Subscription<calibration_interfaces::msg::ClusterCentroids>::SharedPtr sensor2_sub;

    // Sensor 1 Declaration
    bool sensor1Received, sensor2Received;
    pcl::PointCloud<pcl::PointXYZ>::Ptr sensor1_cloud, sensor2_cloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr isensor1_cloud, isensor2_cloud;

    /** \brief TF listener object. */
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    int nFrames;

    std::vector<pcl::PointXYZ> sensor1_vector{4}, sensor2_vector{4};

    geometry_msgs::msg::TransformStamped tf_sensor1_sensor2;
    tf2::Stamped<tf2::Transform> tf2_sensor1_s2;

    std::string sensor1_frame_id = "";
    std::string sensor1_rotated_frame_id = "";
    std::string sensor2_frame_id = "";
    std::string sensor2_rotated_frame_id = "";

    tf2::Transform transf;

    std::vector<std::vector<std::tuple<int, int, pcl::PointCloud<pcl::PointXYZ>,
                                       std::vector<pcl::PointXYZ>>>>
        sensor1_buffer;
    std::vector<std::vector<std::tuple<int, int, pcl::PointCloud<pcl::PointXYZ>,
                                       std::vector<pcl::PointXYZ>>>>
        sensor2_buffer;

    long int sensor1_count, sensor2_count;

    std::ofstream savefile;

    // Internal State Definistion
    bool calibration_ended;

    int S1_WARMUP_COUNT = 0, S2_WARMUP_COUNT = 0;
    bool S1_WARMUP_DONE = false, S2_WARMUP_DONE = false;
    unsigned TARGET_POSITIONS_COUNT = 0;
    int TARGET_ITERATIONS = 30;

    // Node Parameters Declaration
    std::string csv_name;
    bool sync_iterations;
    bool save_to_file_;
    bool publish_tf_;
    bool is_sensor1_cam, is_sensor2_cam;
    bool skip_warmup, single_pose_mode;
    bool results_every_pose;
};