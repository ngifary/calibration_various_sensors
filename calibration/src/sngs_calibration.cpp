/*
  laser2cam_calibration: Perform the registration step
*/

#include "sngs_calibration.hpp"

Registration::Registration(const rclcpp::NodeOptions &options = rclcpp::NodeOptions{}) : Node("registration", options),
                                                                                         tf_buffer_(this->get_clock()),
                                                                                         tf_listener_(tf_buffer_)
{
    RCLCPP_INFO(this->get_logger(), "Calibration Starting....");

    initializeParams();

    // Sensor 1 Definition
    sensor1_cloud_ =
        pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    isensor1_cloud_ =
        pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

    // Sensor 2 Definition
    sensor2_cloud_ =
        pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    isensor2_cloud_ =
        pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

    both_centroids_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("both_centers_cloud", 1);

    sensor1_sub_.subscribe(this, "cloud1", cloudQoS().get_rmw_qos_profile());
    sensor2_sub_.subscribe(this, "cloud2", cloudQoS().get_rmw_qos_profile());

    sync_inputs_a_ = std::make_shared<message_filters::Synchronizer<approximate_policy>>(max_queue_size_);
    sync_inputs_a_->connectInput(sensor1_sub_, sensor2_sub_);
    sync_inputs_a_->registerCallback(std::bind(&Registration::callback, this, std::placeholders::_1, std::placeholders::_2));

    // Saving results to file
    if (save_to_file_)
    {
        std::ostringstream os;
        os << getenv("HOME") << "/l2c_experiments/" << csv_name_;
        if (save_to_file_)
        {
            if (DEBUG)
                RCLCPP_INFO(this->get_logger(), "Opening %s", os.str().c_str());
            savefile_.open(os.str().c_str());
            savefile_ << "it, x, y, z, r, p, y, used_sen1, used_sen2, total_sen1, "
                         "total_sen2"
                      << std::endl;
        }
    }
}

Registration::~Registration()
{
    sensor1_sub_.unsubscribe();
    sensor2_sub_.unsubscribe();

    if (save_to_file_)
        savefile_.close();

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

    std::string backuppath = path + "/launch/calibrated_tf_" + str + ".launch";
    path = path + "/launch/calibrated_tf.launch";

    std::cout << std::endl
              << "Creating .launch file with calibrated TF in: " << std::endl
              << path.c_str() << std::endl;
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

    std::string sensor2_final_transformation_frame = sensor2_frame_id_;
    if (is_sensor2_cam_)
    {
        sensor2_final_transformation_frame = sensor2_rotated_frame_id_;
        std::ostringstream sensor2_rot_stream_pub_;
        sensor2_rot_stream_pub_ << "0 0 0 -1.57079632679 0 -1.57079632679 "
                                << sensor2_rotated_frame_id_ << " "
                                << sensor2_frame_id_ << " 10";
        std::string sensor2_rotation = sensor2_rot_stream_pub_.str();

        tinyxml2::XMLElement *sensor2_rotation_node = doc.NewElement("node");
        sensor2_rotation_node->SetAttribute("pkg", "tf");
        sensor2_rotation_node->SetAttribute("type", "static_transform_publisher");
        sensor2_rotation_node->SetAttribute("name", "sensor2_rot_tf");
        sensor2_rotation_node->SetAttribute("args", sensor2_rotation.c_str());
        root->LinkEndChild(sensor2_rotation_node);
    }

    std::string sensor1_final_transformation_frame = sensor1_frame_id_;
    if (is_sensor1_cam_)
    {
        sensor1_final_transformation_frame = sensor1_rotated_frame_id_;
        std::ostringstream sensor1_rot_stream_pub_;
        sensor1_rot_stream_pub_ << "0 0 0 -1.57079632679 0 -1.57079632679 "
                                << sensor1_rotated_frame_id_ << " "
                                << sensor1_frame_id_ << " 10";
        std::string sensor1_rotation = sensor1_rot_stream_pub_.str();

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
    std::string tf_args = sstream.str();

    tinyxml2::XMLElement *node = doc.NewElement("node");
    node->SetAttribute("pkg", "tf");
    node->SetAttribute("type", "static_transform_publisher");
    node->SetAttribute("name", "laser2cam_tf");
    node->SetAttribute("args", tf_args.c_str());
    root->LinkEndChild(node);

    // Save XML file and copy
    doc.SaveFile(path.c_str());
    doc.SaveFile(backuppath.c_str());

    if (DEBUG)
        RCLCPP_INFO(this->get_logger(), "Calibration process finished.");

    RCLCPP_INFO(this->get_logger(), "Shutting down....");
}

void Registration::initializeParams()
{
    rcl_interfaces::msg::ParameterDescriptor desc;

    desc.name = "sync_iterations";
    desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    desc.description = "";
    sync_iterations_ = declare_parameter(desc.name, false);

    desc.name = "save_to_file";
    desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    desc.description = "";
    save_to_file_ = declare_parameter(desc.name, false);

    desc.name = "publish_tf";
    desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    desc.description = "";
    publish_tf_ = declare_parameter(desc.name, true);

    desc.name = "is_sensor2_cam";
    desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    desc.description = "";
    is_sensor2_cam_ = declare_parameter(desc.name, false);

    desc.name = "is_sensor1_cam";
    desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    desc.description = "";
    is_sensor1_cam_ = declare_parameter(desc.name, false);

    desc.name = "skip_warmup";
    desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    desc.description = "";
    skip_warmup_ = declare_parameter(desc.name, false);

    desc.name = "single_pose_mode";
    desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    desc.description = "";
    single_pose_mode_ = declare_parameter(desc.name, false);

    desc.name = "results_every_pose";
    desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    desc.description = "";
    results_every_pose_ = declare_parameter(desc.name, false);

    desc.name = "csv_name";
    desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    desc.description = "";
    csv_name_ = declare_parameter(desc.name, "registration_" + currentDateTime() + ".csv");
}

tf2::Transform Registration::calibrateExtrinsics(std::vector<tf2::Vector3> sensor1_pts, std::vector<tf2::Vector3> sensor2_pts)
{
    tf2::Transform tf_sensor1_sensor2;

    // tf2::Vector3 translation;
    // // tf2Scalar x, y, z;

    // for (ushort i = 0; i < 4; i++)
    // {
    //     translation += sensor2_pts[i] - sensor2_pts[i];
    //     // x += sensor2_pts[i].getX() - sensor1_pts[i].getX();
    //     // y += sensor2_pts[i].getY() - sensor1_pts[i].getY();
    //     // z += sensor2_pts[i].getZ() - sensor1_pts[i].getZ();
    // }
    // translation /= 4;
    // // translation.setX(x/4);
    // // translation.setY(y/4);
    // // translation.setZ(z/4);
    // tf_sensor1_sensor2.setOrigin(translation);

    tf2::Vector3 translation;
    tf2::Vector3 rotation;
    // Get rotation by comparing the normal vector of a planar triangle (4C3) from each sensor
    for (ushort j = 0; j < 4; j++) // 012 123 230 301 with modulus 4
    {
        ushort first = j%4, second = (j+1)%4 , third = (j+2)%4;

        tf2::Vector3 triangle1_pt1 = sensor1_pts[first];
        tf2::Vector3 triangle1_pt2 = sensor1_pts[second];
        tf2::Vector3 triangle1_pt3 = sensor1_pts[third];

        tf2::Vector3 triangle2_pt1 = sensor2_pts[first];
        tf2::Vector3 triangle2_pt2 = sensor2_pts[second];
        tf2::Vector3 triangle2_pt3 = sensor2_pts[third];

        // Get translation
        translation += (triangle2_pt1 + triangle2_pt2 + triangle2_pt3) - (triangle1_pt1 + triangle1_pt2 + triangle1_pt3);

        tf2::Vector3 triangle1_normal = (triangle1_pt1 - triangle1_pt2).cross(triangle1_pt1 - triangle1_pt3);
        tf2::Vector3 triangle2_normal = (triangle2_pt1 - triangle2_pt2).cross(triangle2_pt1 - triangle2_pt3);

        // Get quaternion cos(theta/2) + sin(theta/2)*(x'i + y'j + z'k) = w + xi + yj + zk
        double theta = triangle1_normal.angle(triangle2_normal); // theta
        tf2::Vector3 axis = triangle1_normal.cross(triangle2_normal).normalize(); // x'i + y'j + z'k

        tf2::Quaternion quaternion;
        quaternion.setW(std::cos(theta/2));
        quaternion.setX(std::sin(theta/2)*axis.getX());
        quaternion.setY(std::sin(theta/2)*axis.getY());
        quaternion.setZ(std::sin(theta/2)*axis.getZ());

        double roll, pitch, yaw;
        tf2::impl::getEulerYPR(quaternion, yaw, pitch, roll);
        rotation.setX(rotation.getX() + roll);
        rotation.setY(rotation.getY() + pitch);
        rotation.setZ(rotation.getZ() + yaw);
    }
    translation /= 36;
    rotation /= 4;
    tf2::Quaternion quaternion;
    quaternion.setRPY(rotation.getX(), rotation.getY(), rotation.getZ());
    tf_sensor1_sensor2.setRotation(quaternion);

    return tf_sensor1_sensor2;
}

void Registration::camera_to_lidar(sensor_msgs::msg::PointCloud2::ConstPtr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out)
{
    std::string frame_id = cloud_in->header.frame_id;
    // std::ostringstream sstream;
    // sstream << "rotated_" << frame_id;
    // std::string rotated_frame_id = sstream.str();
    std::string rotated_frame_id = "rotated_" + frame_id;
    // sensor1_rotated_frame_id_ = sstream.str();

    pcl::PointCloud<pcl::PointXYZ>::Ptr xy_cloud_(
        new pcl::PointCloud<pcl::PointXYZ>());

    pcl::fromROSMsg(*cloud_in, *xy_cloud_);

    geometry_msgs::msg::TransformStamped transform;

    try
    {
        transform = tf_buffer_.lookupTransform(rotated_frame_id, frame_id,
                                               tf2::TimePointZero, tf2::durationFromSec(20.0));
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "TF exception:\n%s", ex.what());
        return;
    }

    tf2::Transform tf_transform;
    tf2::fromMsg(transform, tf_transform);
    tf2::Transform inverse = tf_transform.inverse();
    double roll, pitch, yaw;
    inverse.getBasis().getRPY(roll, pitch, yaw);

    pcl_ros::transformPointCloud(*xy_cloud_, *cloud_out, transform);
}

void Registration::callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr sensor1_centroids, const sensor_msgs::msg::PointCloud2::ConstSharedPtr sensor2_centroids)
{

    RCLCPP_INFO(this->get_logger(), "Cluster pair correspondence received!.");
    RCLCPP_INFO(get_logger(), "[Time stamp] Sensor 1: [%i, %i] and Sensor 2: [%i, %i]", sensor1_centroids->header.stamp.sec, sensor1_centroids->header.stamp.nanosec, sensor2_centroids->header.stamp.sec, sensor2_centroids->header.stamp.nanosec);

    std::string sensor1_frame_id = sensor1_centroids->header.frame_id;
    std::string sensor2_frame_id = sensor2_centroids->header.frame_id;

    if (is_sensor1_cam_)
    {
        camera_to_lidar(sensor1_centroids, sensor1_cloud_);
    }
    else
    {
        fromROSMsg(*sensor1_centroids, *sensor1_cloud_);
    }

    if (is_sensor2_cam_)
    {
        camera_to_lidar(sensor2_centroids, sensor2_cloud_);
    }
    else
    {
        fromROSMsg(*sensor2_centroids, *sensor2_cloud_);
    }

    sortPatternCenters(sensor1_cloud_, sensor1_vector_);
    sortPatternCenters(sensor2_cloud_, sensor2_vector_);

    if (DEBUG)
    {
        sensor_msgs::msg::PointCloud2 colour_cloud_;

        colourCenters(sensor1_vector_, isensor1_cloud_);
        pcl::toROSMsg(*isensor1_cloud_, colour_cloud_);
        colour_cloud_.header.frame_id =
            is_sensor1_cam_ ? "rotated_" + sensor1_frame_id : sensor1_frame_id;
        colour_sensor1_pub_->publish(colour_cloud_);

        colourCenters(sensor2_vector_, isensor2_cloud_);
        pcl::toROSMsg(*isensor2_cloud_, colour_cloud_);
        colour_cloud_.header.frame_id =
            is_sensor2_cam_ ? "rotated_" + sensor2_frame_id : sensor2_frame_id;
        colour_sensor2_pub_->publish(colour_cloud_);
    }

    // if (DEBUG)
    // {
    //     for (std::vector<pcl::PointXYZ>::iterator it = sensor1_vector_.begin();
    //          it < sensor1_vector_.end(); ++it)
    //     {

    //         std::cout << "l" << it - sensor1_vector_.begin() << "="
    //                   << "[" << (*it).x << " " << (*it).y << " " << (*it).z << "]" << std::endl;
    //     }
    // }

    // // sync_iterations_ is designed to extract a calibration result every single
    // // frame, so we cannot wait until TARGET_ITERATIONS
    // if (sync_iterations_)
    // {
    //     if (sensor2_count_ >= sensor1_count_)
    //     {
    //         calibrateExtrinsics(sensor1_count_);
    //     }
    //     else
    //     {
    //         if (tf_sensor1_sensor2_.header.frame_id != "" &&
    //             tf_sensor1_sensor2_.child_frame_id != "")
    //         {
    //             static auto br = tf2_ros::StaticTransformBroadcaster(this);
    //             tf_sensor1_sensor2_.header.stamp = this->get_clock()->now();
    //             if (publish_tf_)
    //                 br.sendTransform(tf_sensor1_sensor2_);
    //         }
    //     }
    //     return;
    // }



    // pcl::PointCloud<pcl::PointXYZ>::Ptr sensor1Centroids(new pcl::PointCloud<pcl::PointXYZ>),
    //     sensor2Centroids(new pcl::PointCloud<pcl::PointXYZ>),
    //     combinedCentroids(new pcl::PointCloud<pcl::PointXYZ>);
    // sensor_msgs::msg::PointCloud2 combined_centroids;

    // pcl::fromROSMsg(*sensor1_centroids, *sensor1Centroids);
    // pcl::fromROSMsg(*sensor2_centroids, *sensor2Centroids);
    // *combinedCentroids = *sensor1Centroids;
    // *combinedCentroids += *sensor2Centroids;
    // pcl::toROSMsg(*combinedCentroids, combined_centroids);
    // combined_centroids.header = sensor1_centroids->header;

    // both_centroids_pub_->publish(combined_centroids);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<Registration>();

    rclcpp::spin(nh);
    return 0;
}