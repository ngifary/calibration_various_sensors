/*
  laser2cam_calibration - Automatic calibration algorithm for extrinsic
  parameters of a stereo camera and a velodyne Copyright (C) 2017-2021 Jorge
  Beltran, Carlos Guindel

  This file is part of laser2cam_calibration.

  laser2cam_calibration is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 2 of the License, or
  (at your option) any later version.

  laser2cam_calibration is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with laser2cam_calibration.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
  laser2cam_calibration: Perform the registration step
*/

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "pcl/registration/transformation_estimation_svd.h"
#include "pcl_conversions/pcl_conversions.hpp"
#include "pcl_ros/transforms.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/int32.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/convert.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/transform_datatypes.h"
#include "tinyxml2.h"
#include "velo2cam_utils.h"
#include "opencv2/opencv.hpp"
#include "calibration_interfaces/msg/cluster_centroids.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cinttypes>
#include <limits>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace sensor_msgs;
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

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

    string sensor1_frame_id = "";
    string sensor1_rotated_frame_id = "";
    string sensor2_frame_id = "";
    string sensor2_rotated_frame_id = "";

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
    string csv_name;
    bool sync_iterations;
    bool save_to_file_;
    bool publish_tf_;
    bool is_sensor1_cam, is_sensor2_cam;
    bool skip_warmup, single_pose_mode;
    bool results_every_pose;
};

Registration::Registration(const rclcpp::NodeOptions &options = rclcpp::NodeOptions{}) : Node("registration", options),
                               tf_buffer_(this->get_clock()),
                               tf_listener_(tf_buffer_)
{
    RCLCPP_INFO(this->get_logger(), "Calibration Starting....");
    std::cout << "It's here!-1";

    // Node Parameters Definition
    sync_iterations = this->declare_parameter("sync_iterations", false);
    save_to_file_ = this->declare_parameter("save_to_file", false);
    publish_tf_ = this->declare_parameter("publish_tf", true);
    is_sensor2_cam = this->declare_parameter("is_sensor2_cam", false);
    is_sensor1_cam = this->declare_parameter("is_sensor1_cam", false);
    skip_warmup = this->declare_parameter("skip_warmup", false);
    single_pose_mode = this->declare_parameter("single_pose_mode", false);
    results_every_pose = this->declare_parameter("results_every_pose", false);
    csv_name = this->declare_parameter("csv_name", "registration_" + currentDateTime() + ".csv");

    // Sensor 1 Definition
    sensor1Received = false;
    sensor1_cloud =
        pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    isensor1_cloud =
        pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

    // Sensor 2 Definition
    sensor2Received = false;
    sensor2_cloud =
        pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    isensor2_cloud =
        pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

    // Subs Definition
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    std::cout << "It's here!";
    sensor1_sub = this->create_subscription<calibration_interfaces::msg::ClusterCentroids>(
        "cloud1", 100, std::bind(&Registration::sensor1_callback, this, std::placeholders::_1));
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    sensor2_sub = this->create_subscription<calibration_interfaces::msg::ClusterCentroids>(
        "cloud2", 100, std::bind(&Registration::sensor2_callback, this, std::placeholders::_1));

    // Pubs Definition
    if (DEBUG)
    {
        clusters_sensor2_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("clusters_sensor2", 1);
        clusters_sensor1_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("clusters_sensor1", 1);
        colour_sensor2_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("colour_sensor2", 1);
        colour_sensor1_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("colour_sensor1", 1);
    }
    sensor_switch_pub = this->create_publisher<std_msgs::msg::Empty>("warmup_switch", 1);
    iterations_pub = this->create_publisher<std_msgs::msg::Int32>("iterations", 1);

    // Internal State
    calibration_ended = false;

    // Saving results to file
    if (save_to_file_)
    {
        ostringstream os;
        os << getenv("HOME") << "/v2c_experiments/" << csv_name;
        if (save_to_file_)
        {
            if (DEBUG)
                RCLCPP_INFO(this->get_logger(), "Opening %s", os.str().c_str());
            savefile.open(os.str().c_str());
            savefile << "it, x, y, z, r, p, y, used_sen1, used_sen2, total_sen1, "
                        "total_sen2"
                     << endl;
        }
    }

    // Check warmup condition
    if (skip_warmup)
    {
        S1_WARMUP_DONE = true;
        S2_WARMUP_DONE = true;
        RCLCPP_WARN(this->get_logger(), "Skipping warmup");
    }
    else
    {
        cout << "Please, adjust the filters for each sensor before the calibration "
                "starts."
             << endl;
    }
}

Registration::~Registration()
{
    this->unsubscribe();

    if (save_to_file_)
        savefile.close();

    // Save calibration params to launch file for testing

    // Get time
    time_t rawtime;
    struct tm *timeinfo;
    char buffer[80];

    time(&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer, 80, "%Y-%m-%d-%H-%M-%S", timeinfo);
    std::string str(buffer);

    // Get tf data
    geometry_msgs::msg::TransformStamped transform;
    tf2::Stamped<tf2::Transform> tf_transform;
    tf2::fromMsg(transform, tf_transform);
    tf2::Transform inverse = tf_transform.inverse();
    double roll, pitch, yaw;
    double xt = inverse.getOrigin().getX(), yt = inverse.getOrigin().getY(),
           zt = inverse.getOrigin().getZ();
    inverse.getBasis().getRPY(roll, pitch, yaw);

    std::string path = ament_index_cpp::get_package_share_directory("calibration");

    string backuppath = path + "/launch/calibrated_tf_" + str + ".launch";
    path = path + "/launch/calibrated_tf.launch";

    cout << endl
         << "Creating .launch file with calibrated TF in: " << endl
         << path.c_str() << endl;
    // Create .launch file with calibrated TF

    tinyxml2::XMLDocument doc;
    tinyxml2::XMLDeclaration *decl = doc.NewDeclaration();
    decl->SetValue(R"(xml version="1.0" encoding="UTF-8")");
    doc.LinkEndChild(decl);
    tinyxml2::XMLElement *root = doc.NewElement("launch");
    doc.LinkEndChild(root);

    tinyxml2::XMLElement *arg = doc.NewElement("arg");
    arg->SetAttribute("name", "stdout");
    arg->SetAttribute("default", "screen");
    root->LinkEndChild(arg);

    string sensor2_final_transformation_frame = sensor2_frame_id;
    if (is_sensor2_cam)
    {
        sensor2_final_transformation_frame = sensor2_rotated_frame_id;
        std::ostringstream sensor2_rot_stream_pub;
        sensor2_rot_stream_pub << "0 0 0 -1.57079632679 0 -1.57079632679 "
                               << sensor2_rotated_frame_id << " "
                               << sensor2_frame_id << " 10";
        string sensor2_rotation = sensor2_rot_stream_pub.str();

        tinyxml2::XMLElement *sensor2_rotation_node = doc.NewElement("node");
        sensor2_rotation_node->SetAttribute("pkg", "tf");
        sensor2_rotation_node->SetAttribute("type", "static_transform_publisher");
        sensor2_rotation_node->SetAttribute("name", "sensor2_rot_tf");
        sensor2_rotation_node->SetAttribute("args", sensor2_rotation.c_str());
        root->LinkEndChild(sensor2_rotation_node);
    }

    string sensor1_final_transformation_frame = sensor1_frame_id;
    if (is_sensor1_cam)
    {
        sensor1_final_transformation_frame = sensor1_rotated_frame_id;
        std::ostringstream sensor1_rot_stream_pub;
        sensor1_rot_stream_pub << "0 0 0 -1.57079632679 0 -1.57079632679 "
                               << sensor1_rotated_frame_id << " "
                               << sensor1_frame_id << " 10";
        string sensor1_rotation = sensor1_rot_stream_pub.str();

        tinyxml2::XMLElement *sensor1_rotation_node = doc.NewElement("node");
        sensor1_rotation_node->SetAttribute("pkg", "tf");
        sensor1_rotation_node->SetAttribute("type", "static_transform_publisher");
        sensor1_rotation_node->SetAttribute("name", "sensor1_rot_tf");
        sensor1_rotation_node->SetAttribute("args", sensor1_rotation.c_str());
        root->LinkEndChild(sensor1_rotation_node);
    }

    std::ostringstream sstream;
    sstream << xt << " " << yt << " " << zt << " " << yaw << " " << pitch << " "
            << roll << " " << sensor2_final_transformation_frame << " "
            << sensor1_final_transformation_frame << " 100";
    string tf_args = sstream.str();

    tinyxml2::XMLElement *node = doc.NewElement("node");
    node->SetAttribute("pkg", "tf");
    node->SetAttribute("type", "static_transform_publisher");
    node->SetAttribute("name", "velo2cam_tf");
    node->SetAttribute("args", tf_args.c_str());
    root->LinkEndChild(node);

    // Save XML file and copy
    doc.SaveFile(path.c_str());
    doc.SaveFile(backuppath.c_str());

    if (DEBUG)
        cout << "Calibration process finished." << endl;

    RCLCPP_INFO(this->get_logger(), "Shutting down....");
}

void Registration::calibrateExtrinsics(int seek_iter = -1)
{
    std::vector<pcl::PointXYZ> local_sensor1_vector, local_sensor2_vector;
    pcl::PointCloud<pcl::PointXYZ>::Ptr local_sensor1_cloud(
        new pcl::PointCloud<pcl::PointXYZ>),
        local_sensor2_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> local_l_cloud, local_c_cloud;

    int used_sensor2, used_sensor1;

    // Get final frame names for TF broadcaster
    string sensor1_final_transformation_frame = sensor1_frame_id;
    if (is_sensor1_cam)
    {
        sensor1_final_transformation_frame = sensor1_rotated_frame_id;
    }
    string sensor2_final_transformation_frame = sensor2_frame_id;
    if (is_sensor2_cam)
    {
        sensor2_final_transformation_frame = sensor2_rotated_frame_id;
    }

    int total_sensor1, total_sensor2;

    if (seek_iter > 0)
    { // Add clouds (per sensor) from every position using
        // last 'seek_iter' detection
        if (DEBUG)
            RCLCPP_INFO(this->get_logger(), "Seeking %d iterations", seek_iter);

        for (unsigned i = 0; i < TARGET_POSITIONS_COUNT + 1; ++i)
        {
            if (DEBUG)
                RCLCPP_INFO(this->get_logger(), "Target position: %d, Last sensor2: %d, last sensor1: %d",
                            i + 1, std::get<0>(sensor2_buffer[i].back()),
                            std::get<0>(sensor1_buffer[i].back()));
            // Sensor 1
            auto it1 = std::find_if(
                sensor1_buffer[i].begin(), sensor1_buffer[i].end(),
                [&seek_iter](
                    const std::tuple<int, int, pcl::PointCloud<pcl::PointXYZ>,
                                     std::vector<pcl::PointXYZ>> &e)
                {
                    return std::get<0>(e) == seek_iter;
                });
            if (it1 == sensor1_buffer[i].end())
            {
                RCLCPP_WARN(this->get_logger(), "Could not sync sensor1");
                return;
            }

            local_sensor1_vector.insert(
                local_sensor1_vector.end(), std::get<3>(*it1).begin(),
                std::get<3>(*it1).end()); // Add sorted centers (for equations)
            *local_sensor1_cloud +=
                std::get<2>(*it1); // Add centers cloud (for registration)
            used_sensor1 = std::get<1>(*it1);
            total_sensor1 = std::get<0>(*it1);

            // Sensor 2
            auto it2 = std::find_if(
                sensor2_buffer[i].begin(), sensor2_buffer[i].end(),
                [&seek_iter](
                    const std::tuple<int, int, pcl::PointCloud<pcl::PointXYZ>,
                                     std::vector<pcl::PointXYZ>> &e)
                {
                    return std::get<0>(e) == seek_iter;
                });
            if (it2 == sensor2_buffer[i].end())
            {
                RCLCPP_WARN(this->get_logger(), "Could not sync sensor2");
                return;
            }

            local_sensor2_vector.insert(
                local_sensor2_vector.end(), std::get<3>(*it2).begin(),
                std::get<3>(*it2).end()); // Add sorted centers (for equations)
            *local_sensor2_cloud +=
                std::get<2>(*it2); // Add centers cloud (for registration)
            used_sensor2 = std::get<1>(*it2);
            total_sensor2 = std::get<0>(*it2);
        }
        RCLCPP_INFO(this->get_logger(), "Synchronizing cluster centroids");
    }
    else
    { // Add clouds (per sensor) from every position using last available
        // detection
        for (unsigned i = 0; i < TARGET_POSITIONS_COUNT + 1; ++i)
        {
            // Sensor 1
            local_sensor1_vector.insert(
                local_sensor1_vector.end(),
                std::get<3>(sensor1_buffer[i].back()).begin(),
                std::get<3>(sensor1_buffer[i].back())
                    .end()); // Add sorted centers (for equations)
            *local_sensor1_cloud += std::get<2>(
                sensor1_buffer[i].back()); // Add centers cloud (for registration)
            used_sensor1 = std::get<1>(sensor2_buffer[i].back());

            // Sensor 2
            local_sensor2_vector.insert(
                local_sensor2_vector.end(),
                std::get<3>(sensor2_buffer[i].back()).begin(),
                std::get<3>(sensor2_buffer[i].back())
                    .end()); // Add sorted centers (for equations)
            *local_sensor2_cloud += std::get<2>(
                sensor2_buffer[i].back()); // Add centers cloud (for registration)
        }
    }

    if (DEBUG)
    {
        sensor_msgs::msg::PointCloud2 ros_cloud;
        pcl::toROSMsg(*local_sensor2_cloud, ros_cloud);
        ros_cloud.header.frame_id = sensor2_rotated_frame_id;
        clusters_sensor2_pub->publish(ros_cloud);

        pcl::toROSMsg(*local_sensor1_cloud, ros_cloud);
        ros_cloud.header.frame_id = sensor1_frame_id;
        clusters_sensor1_pub->publish(ros_cloud);
    }

    // SVD code
    pcl::PointCloud<pcl::PointXYZ>::Ptr sorted_centers1(
        new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr sorted_centers2(
        new pcl::PointCloud<pcl::PointXYZ>());

    for (unsigned i = 0; i < local_sensor1_vector.size(); ++i)
    {
        sorted_centers1->push_back(local_sensor1_vector[i]);
        sorted_centers2->push_back(local_sensor2_vector[i]);
    }

    Eigen::Matrix4f final_transformation;
    const pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,
                                                         pcl::PointXYZ>
        trans_est_svd(true);
    trans_est_svd.estimateRigidTransformation(*sorted_centers1, *sorted_centers2,
                                              final_transformation);

    tf2::Matrix3x3 tf3d;
    tf3d.setValue(final_transformation(0, 0), final_transformation(0, 1),
                  final_transformation(0, 2), final_transformation(1, 0),
                  final_transformation(1, 1), final_transformation(1, 2),
                  final_transformation(2, 0), final_transformation(2, 1),
                  final_transformation(2, 2));

    tf2::Quaternion tfqt;
    tf3d.getRotation(tfqt);

    tf2::Vector3 origin;
    origin.setValue(final_transformation(0, 3), final_transformation(1, 3),
                    final_transformation(2, 3));

    transf.setOrigin(origin);
    transf.setRotation(tfqt);

    static auto br = tf2_ros::StaticTransformBroadcaster(this);
    rclcpp::Time now = this->get_clock()->now();

    tf_sensor1_sensor2.header.stamp = now;
    tf_sensor1_sensor2.header.frame_id = sensor1_final_transformation_frame;
    tf_sensor1_sensor2.child_frame_id = sensor2_final_transformation_frame;
    tf2::convert(transf.inverse(), tf_sensor1_sensor2.transform);

    if (publish_tf_)
        br.sendTransform(tf_sensor1_sensor2);

    tf2::Stamped<tf2::Transform> tf_transform;
    tf2::fromMsg(tf_sensor1_sensor2, tf_transform);

    tf2::Transform inverse = tf_transform.inverse();
    double roll, pitch, yaw;
    double xt = inverse.getOrigin().getX(), yt = inverse.getOrigin().getY(),
           zt = inverse.getOrigin().getZ();
    inverse.getBasis().getRPY(roll, pitch, yaw);

    if (save_to_file_)
    {
        savefile << seek_iter << ", " << xt << ", " << yt << ", " << zt << ", "
                 << roll << ", " << pitch << ", " << yaw << ", " << used_sensor1
                 << ", " << used_sensor2 << ", " << total_sensor1 << ", "
                 << total_sensor2 << endl;
    }

    cout << setprecision(4) << std::fixed;
    cout << "Calibration finished succesfully." << endl;
    cout << "Extrinsic parameters:" << endl;
    cout << "x = " << xt << "\ty = " << yt << "\tz = " << zt << endl;
    cout << "roll = " << roll << "\tpitch = " << pitch << "\tyaw = " << yaw << endl;

    sensor1Received = false;
    sensor2Received = false;
}

void Registration::sensor1_callback(const calibration_interfaces::msg::ClusterCentroids::ConstSharedPtr sensor1_centroids)
{
    RCLCPP_INFO(this->get_logger(), "Sensor 1 callback");
    std::cout << "It's here!";
    sensor1_frame_id = sensor1_centroids->header.frame_id;
    if (!S1_WARMUP_DONE)
    {
        RCLCPP_INFO(this->get_logger(), "Sensor 1.1 callback");
        S1_WARMUP_COUNT++;
        std::cout << "It's here!2";
        cout << "Clusters from " << sensor1_frame_id << ": " << S1_WARMUP_COUNT
             << "/10" << '\r' << flush;
        if (S1_WARMUP_COUNT >= 10) // TODO: Change to param?
        {
            cout << endl;
            sensor1_sub.reset();
            sensor2_sub.reset();

            cout << "Clusters from " << sensor1_frame_id
                 << " received. Is the warmup done? [Y/n]" << endl;
            string answer;
            answer.clear();
            getline(cin, answer);
            if (answer == "y" || answer == "Y" || answer == "")
            {
                S1_WARMUP_DONE = !S1_WARMUP_DONE;

                if (!S2_WARMUP_DONE)
                {
                    cout << "Filters for sensor 1 are adjusted now. Please, proceed with "
                            "the other sensor."
                         << endl;
                }
                else
                { // Both sensors adjusted
                    cout << "Warmup phase completed. Starting calibration phase." << endl;
                    std_msgs::msg::Empty myMsg;
                    sensor_switch_pub->publish(myMsg); //
                }
            }
            else
            { // Reset counter to allow further warmup
                S1_WARMUP_COUNT = 0;
            }
            sensor1_sub = this->create_subscription<calibration_interfaces::msg::ClusterCentroids>(
                "cloud1", 100, std::bind(&Registration::sensor1_callback, this, std::placeholders::_1));
            sensor2_sub = this->create_subscription<calibration_interfaces::msg::ClusterCentroids>(
                "cloud2", 100, std::bind(&Registration::sensor2_callback, this, std::placeholders::_1));
        }
        return;
    }

    if (!S2_WARMUP_DONE)
    {
        return;
    }

    if (DEBUG)
        RCLCPP_INFO(this->get_logger(), "sensor1 (%s) pattern ready!", sensor1_frame_id.c_str());

    if (sensor1_buffer.size() == TARGET_POSITIONS_COUNT)
    {
        sensor1_buffer.resize(TARGET_POSITIONS_COUNT + 1);
    }

    if (is_sensor1_cam)
    {
        std::ostringstream sstream;
        geometry_msgs::msg::TransformStamped transformStamped;
        sstream << "rotated_" << sensor1_frame_id;
        sensor1_rotated_frame_id = sstream.str();

        pcl::PointCloud<pcl::PointXYZ>::Ptr xy_sensor1_cloud(
            new pcl::PointCloud<pcl::PointXYZ>());

        fromROSMsg(sensor1_centroids->cloud, *xy_sensor1_cloud);

        geometry_msgs::msg::TransformStamped transform;
        tf2_ros::TransformReadyCallback callback; // Investigate this
        try
        {
            tf_buffer_.waitForTransform(sensor1_rotated_frame_id, sensor1_frame_id,
                                        tf2::TimePointZero, tf2::durationFromSec(20.0), callback);
            transformStamped = tf_buffer_.lookupTransform(sensor2_rotated_frame_id, sensor2_frame_id,
                                                          tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "TF exception:\n%s", ex.what());
            return;
        }

        tf2::Stamped<tf2::Transform> tf_transform;
        tf2::fromMsg(transform, tf_transform);
        tf2::Transform inverse = tf_transform.inverse();
        double roll, pitch, yaw;
        inverse.getBasis().getRPY(roll, pitch, yaw);

        pcl_ros::transformPointCloud(*xy_sensor1_cloud, *sensor1_cloud, transform);
    }
    else
    {
        fromROSMsg(sensor1_centroids->cloud, *sensor1_cloud);
    }

    sensor1Received = true;

    sortPatternCenters(sensor1_cloud, sensor1_vector);
    if (DEBUG)
    {
        colourCenters(sensor1_vector, isensor1_cloud);

        sensor_msgs::msg::PointCloud2 colour_cloud;
        pcl::toROSMsg(*isensor1_cloud, colour_cloud);
        colour_cloud.header.frame_id =
            is_sensor1_cam ? sensor1_rotated_frame_id : sensor1_frame_id;
        colour_sensor1_pub->publish(colour_cloud);
    }

    sensor1_buffer[TARGET_POSITIONS_COUNT].push_back(
        std::tuple<int, int, pcl::PointCloud<pcl::PointXYZ>,
                   std::vector<pcl::PointXYZ>>(
            sensor1_centroids->total_iterations,
            sensor1_centroids->cluster_iterations, *sensor1_cloud,
            sensor1_vector));
    sensor1_count = sensor1_centroids->total_iterations;

    if (DEBUG)
        RCLCPP_INFO(this->get_logger(), "[V2C] sensor1");

    for (vector<pcl::PointXYZ>::iterator it = sensor1_vector.begin();
         it < sensor1_vector.end(); ++it)
    {
        if (DEBUG)
            cout << "l" << it - sensor1_vector.begin() << "="
                 << "[" << (*it).x << " " << (*it).y << " " << (*it).z << "]" << endl;
    }

    // sync_iterations is designed to extract a calibration result every single
    // frame, so we cannot wait until TARGET_ITERATIONS
    if (sync_iterations)
    {
        if (sensor2_count >= sensor1_count)
        {
            calibrateExtrinsics(sensor1_count);
        }
        else
        {
            if (tf_sensor1_sensor2.header.frame_id != "" &&
                tf_sensor1_sensor2.child_frame_id != "")
            {
                static auto br = tf2_ros::StaticTransformBroadcaster(this);
                tf_sensor1_sensor2.header.stamp = this->get_clock()->now();
                if (publish_tf_)
                    br.sendTransform(tf_sensor1_sensor2);
            }
        }
        return;
    }

    // Normal operation (sync_iterations=false)
    if (sensor1Received && sensor2Received)
    {
        cout << min(sensor1_count, sensor2_count) << "/30 iterations" << '\r' << flush;

        std_msgs::msg::Int32 it;
        it.data = min(sensor1_count, sensor2_count);
        iterations_pub->publish(it);

        if (sensor1_count >= TARGET_ITERATIONS &&
            sensor2_count >= TARGET_ITERATIONS)
        {
            cout << endl;
            sensor1_sub.reset();
            sensor2_sub.reset();

            string answer;
            if (single_pose_mode)
            {
                answer = "n";
            }
            else
            {
                cout << "Target iterations reached. Do you need another target "
                        "location? [y/N]"
                     << endl;
                getline(cin, answer);
            }
            if (answer == "n" || answer == "N" || answer == "")
            {
                calibrateExtrinsics(-1);
                calibration_ended = true;
            }
            else
            { // Move the target and start over
                if (results_every_pose)
                    calibrateExtrinsics(-1);
                TARGET_POSITIONS_COUNT++;
                cout << "Please, move the target to its new position and adjust the "
                        "filters for each sensor before the calibration starts."
                     << endl;
                // Start over if other position of the target is required
                std_msgs::msg::Empty myMsg;
                sensor_switch_pub->publish(myMsg); // Set sensor nodes to warmup phase
                S1_WARMUP_DONE = false;
                S1_WARMUP_COUNT = 0;
                S2_WARMUP_DONE = false;
                S2_WARMUP_COUNT = 0;
                sensor1Received = false;
                sensor2Received = false;
                sensor1_count = 0;
                sensor2_count = 0;
            }
            sensor1_sub = this->create_subscription<calibration_interfaces::msg::ClusterCentroids>(
                "cloud1", 100, std::bind(&Registration::sensor1_callback, this, std::placeholders::_1));
            sensor2_sub = this->create_subscription<calibration_interfaces::msg::ClusterCentroids>(
                "cloud2", 100, std::bind(&Registration::sensor2_callback, this, std::placeholders::_1));
            return;
        }
    }
    else
    {
        if (tf_sensor1_sensor2.header.frame_id != "" &&
            tf_sensor1_sensor2.child_frame_id != "")
        {
            static auto br = tf2_ros::StaticTransformBroadcaster(this);
            tf_sensor1_sensor2.header.stamp = this->get_clock()->now();
            if (publish_tf_)
                br.sendTransform(tf_sensor1_sensor2);
        }
    }
}

void Registration::sensor2_callback(const calibration_interfaces::msg::ClusterCentroids::ConstSharedPtr sensor2_centroids)
{
    RCLCPP_INFO(this->get_logger(), "Sensor 2 callback");
    sensor2_frame_id = sensor2_centroids->header.frame_id;
    if (!S2_WARMUP_DONE && S1_WARMUP_DONE)
    {
        S2_WARMUP_COUNT++;
        cout << "Clusters from " << sensor2_frame_id << ": " << S2_WARMUP_COUNT
             << "/10" << '\r' << flush;
        if (S2_WARMUP_COUNT >= 10) // TODO: Change to param?
        {
            cout << endl;
            sensor1_sub.reset();
            sensor2_sub.reset();

            cout << "Clusters from " << sensor2_frame_id
                 << " received. Is the warmup done? (you can also reset this "
                    "position) [Y/n/r]"
                 << endl;
            string answer;
            getline(cin, answer);
            if (answer == "y" || answer == "Y" || answer == "")
            {
                S2_WARMUP_DONE = !S2_WARMUP_DONE;

                if (!S1_WARMUP_DONE)
                {
                    cout << "Filters for sensor 2 are adjusted now. Please, proceed with "
                            "the other sensor."
                         << endl;
                }
                else
                { // Both sensors adjusted
                    cout << "Warmup phase completed. Starting calibration phase." << endl;
                    std_msgs::msg::Empty myMsg;
                    sensor_switch_pub->publish(myMsg); //
                }
            }
            else if (answer == "r" ||
                     answer == "R")
            { // Reset this position and
                // go back to Sensor 1 warmup
                S1_WARMUP_DONE = false;
                S1_WARMUP_COUNT = 0;
                S2_WARMUP_DONE = false;
                S2_WARMUP_COUNT = 0;
                sensor1Received = false;
                sensor2Received = false;
                sensor1_count = 0;
                sensor2_count = 0;
                cout << "Please, adjust the filters for each sensor before the "
                        "calibration starts."
                     << endl;
            }
            else
            { // Reset counter to allow further warmup
                S2_WARMUP_COUNT = 0;
            }
            sensor1_sub = this->create_subscription<calibration_interfaces::msg::ClusterCentroids>(
                "cloud1", 100, std::bind(&Registration::sensor1_callback, this, std::placeholders::_1));
            sensor2_sub = this->create_subscription<calibration_interfaces::msg::ClusterCentroids>(
                "cloud2", 100, std::bind(&Registration::sensor2_callback, this, std::placeholders::_1));
        }
        return;
    }
    else if (!S2_WARMUP_DONE)
    {
        return;
    }
    if (DEBUG)
        RCLCPP_INFO(this->get_logger(), "sensor2 (%s) pattern ready!", sensor2_frame_id.c_str());

    if (sensor2_buffer.size() == TARGET_POSITIONS_COUNT)
    {
        sensor2_buffer.resize(TARGET_POSITIONS_COUNT + 1);
    }

    if (is_sensor2_cam)
    {
        std::ostringstream sstream;
        geometry_msgs::msg::TransformStamped transformStamped;
        sstream << "rotated_" << sensor2_frame_id;
        sensor2_rotated_frame_id = sstream.str();

        pcl::PointCloud<pcl::PointXYZ>::Ptr xy_sensor2_cloud(
            new pcl::PointCloud<pcl::PointXYZ>());

        fromROSMsg(sensor2_centroids->cloud, *xy_sensor2_cloud);

        geometry_msgs::msg::TransformStamped transform;
        tf2_ros::TransformReadyCallback callback; // Investigate this
        try
        {
            tf_buffer_.waitForTransform(sensor1_rotated_frame_id, sensor1_frame_id,
                                        tf2::TimePointZero, tf2::durationFromSec(20.0), callback);
            transformStamped = tf_buffer_.lookupTransform(sensor2_rotated_frame_id, sensor2_frame_id,
                                                          tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "TF exception:\n%s", ex.what());
            return;
        }

        tf2::Stamped<tf2::Transform> tf_transform;
        tf2::fromMsg(transform, tf_transform);
        tf2::Transform inverse = tf_transform.inverse();
        double roll, pitch, yaw;
        inverse.getBasis().getRPY(roll, pitch, yaw);

        pcl_ros::transformPointCloud(*xy_sensor2_cloud, *sensor2_cloud, transform);
    }
    else
    {
        fromROSMsg(sensor2_centroids->cloud, *sensor2_cloud);
    }

    sensor2Received = true;

    sortPatternCenters(sensor2_cloud, sensor2_vector);

    if (DEBUG)
    {
        colourCenters(sensor2_vector, isensor2_cloud);

        sensor_msgs::msg::PointCloud2 colour_cloud;
        pcl::toROSMsg(*isensor2_cloud, colour_cloud);
        colour_cloud.header.frame_id =
            is_sensor2_cam ? sensor2_rotated_frame_id : sensor2_frame_id;
        colour_sensor2_pub->publish(colour_cloud);
    }

    sensor2_buffer[TARGET_POSITIONS_COUNT].push_back(
        std::tuple<int, int, pcl::PointCloud<pcl::PointXYZ>,
                   std::vector<pcl::PointXYZ>>(
            sensor2_centroids->total_iterations,
            sensor2_centroids->cluster_iterations, *sensor2_cloud,
            sensor2_vector));
    sensor2_count = sensor2_centroids->total_iterations;

    if (DEBUG)
        RCLCPP_INFO(this->get_logger(), "[V2C] sensor2");

    for (vector<pcl::PointXYZ>::iterator it = sensor2_vector.begin();
         it < sensor2_vector.end(); ++it)
    {
        if (DEBUG)
            cout << "c" << it - sensor2_vector.begin() << "="
                 << "[" << (*it).x << " " << (*it).y << " " << (*it).z << "]" << endl;
    }

    // sync_iterations is designed to extract a calibration result every single
    // frame, so we cannot wait until TARGET_ITERATIONS
    if (sync_iterations)
    {
        if (sensor1_count >= sensor2_count)
        {
            calibrateExtrinsics(sensor2_count);
        }
        else
        {
            if (tf_sensor1_sensor2.header.frame_id != "" &&
                tf_sensor1_sensor2.child_frame_id != "")
            {
                static auto br = tf2_ros::StaticTransformBroadcaster(this);
                tf_sensor1_sensor2.header.stamp = this->get_clock()->now();
                if (publish_tf_)
                    br.sendTransform(tf_sensor1_sensor2);
            }
        }
        return;
    }

    // Normal operation (sync_iterations=false)
    if (sensor1Received && sensor2Received)
    {
        cout << min(sensor1_count, sensor2_count) << "/30 iterations" << '\r' << flush;

        std_msgs::msg::Int32 it;
        it.data = min(sensor1_count, sensor2_count);
        iterations_pub->publish(it);

        if (sensor1_count >= TARGET_ITERATIONS &&
            sensor2_count >= TARGET_ITERATIONS)
        {
            cout << endl;
            sensor1_sub.reset();
            sensor2_sub.reset();

            string answer;
            if (single_pose_mode)
            {
                answer = "n";
            }
            else
            {
                cout << "Target iterations reached. Do you need another target "
                        "location? [y/N]"
                     << endl;
                getline(cin, answer);
            }
            if (answer == "n" || answer == "N" || answer == "")
            {
                calibrateExtrinsics(-1);
                calibration_ended = true;
            }
            else
            { // Move the target and start over
                if (results_every_pose)
                    calibrateExtrinsics(-1);
                TARGET_POSITIONS_COUNT++;
                cout << "Please, move the target to its new position and adjust the "
                        "filters for each sensor before the calibration starts."
                     << endl;
                // Start over if other position of the target is required
                std_msgs::msg::Empty myMsg;
                sensor_switch_pub->publish(myMsg); // Set sensor nodes to warmup phase
                S1_WARMUP_DONE = false;
                S1_WARMUP_COUNT = 0;
                S2_WARMUP_DONE = false;
                S2_WARMUP_COUNT = 0;
                sensor1Received = false;
                sensor2Received = false;
                sensor1_count = 0;
                sensor2_count = 0;
            }
            sensor1_sub = this->create_subscription<calibration_interfaces::msg::ClusterCentroids>(
                "cloud1", 100, std::bind(&Registration::sensor1_callback, this, std::placeholders::_1));
            sensor2_sub = this->create_subscription<calibration_interfaces::msg::ClusterCentroids>(
                "cloud2", 100, std::bind(&Registration::sensor2_callback, this, std::placeholders::_1));
            return;
        }
    }
    else
    {
        if (tf_sensor1_sensor2.header.frame_id != "" &&
            tf_sensor1_sensor2.child_frame_id != "")
        {
            static auto br = tf2_ros::StaticTransformBroadcaster(this);
            tf_sensor1_sensor2.header.stamp = this->get_clock()->now();
            if (publish_tf_)
                br.sendTransform(tf_sensor1_sensor2);
        }
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<Registration>();
    std::promise<void> stop_async_spinner;
    std::thread async_spinner_thread(
        [stop_token = stop_async_spinner.get_future(), nh]()
        {
            rclcpp::executors::SingleThreadedExecutor executor;
            executor.add_node(nh);
            executor.spin_until_future_complete(stop_token);
        });

    // constexpr char empty_service_name[] = "empty_service";

    // auto client = nh->create_client<std_srvs::srv::Empty>(empty_service_name);
    // auto shared_request = std::make_shared<std_srvs::srv::Empty::Request>();
    // auto future = client->async_send_request(shared_request);
    // spin_until_future_complete(nh, future, std::chrono::seconds(2));

    // rclcpp::spin_until_future_complete(nh, future, std::chrono::seconds(3));
    stop_async_spinner.set_value();
    async_spinner_thread.join();
    rclcpp::Rate loop_rate(30);
    while (rclcpp::ok() && !nh.get()->isFinished())
    {
        rclcpp::spin(nh);
        loop_rate.sleep();
    }

    return 0;
}