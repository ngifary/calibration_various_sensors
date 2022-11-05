/*
  calibration: Derive transformation from the cloud points correspondences
*/

#include "calibration_core.hpp"

Calibration::Calibration(const rclcpp::NodeOptions &options = rclcpp::NodeOptions{}) : Node("registration", options),
                                                                                       tf_buffer_(this->get_clock()),
                                                                                       tf_listener_(tf_buffer_)
{
    RCLCPP_INFO(this->get_logger(), "Calibration Starting....");

    initializeParams();

    colour_sensor1_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("calibration_ready/sensor1", 1);
    colour_sensor2_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("calibration_ready/sensor2", 1);

    sensor1_sub_.subscribe(this, "cloud1", cloudQoS().get_rmw_qos_profile());
    sensor2_sub_.subscribe(this, "cloud2", cloudQoS().get_rmw_qos_profile());

    sync_inputs_a_ = std::make_shared<message_filters::Synchronizer<ApproxSync>>(max_queue_size_);
    sync_inputs_a_->connectInput(sensor1_sub_, sensor2_sub_);
    sync_inputs_a_->registerCallback(std::bind(&Calibration::callback, this, std::placeholders::_1, std::placeholders::_2));
}

Calibration::~Calibration()
{
    sensor1_sub_.unsubscribe();
    sensor2_sub_.unsubscribe();

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
    if (sensor2_is_cam_)
    {
        sensor2_final_transformation_frame = "rotated_" + sensor2_frame_id_;
        std::ostringstream sensor2_rot_stream_pub_;
        sensor2_rot_stream_pub_ << "0 0 0 -1.57079632679 0 -1.57079632679 "
                                << "rotated_" + sensor2_frame_id_ << " "
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
    if (sensor1_is_cam_)
    {
        sensor1_final_transformation_frame = "rotated_" + sensor1_frame_id_;
        std::ostringstream sensor1_rot_stream_pub_;
        sensor1_rot_stream_pub_ << "0 0 0 -1.57079632679 0 -1.57079632679 "
                                << "rotated_" + sensor1_frame_id_ << " "
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

void Calibration::initializeParams()
{
    rcl_interfaces::msg::ParameterDescriptor desc;

    desc.name = "sensor1_is_cam";
    desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    desc.description = "";
    sensor1_is_cam_ = declare_parameter(desc.name, sensor1_is_cam_);

    desc.name = "sensor2_is_cam";
    desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    desc.description = "";
    sensor2_is_cam_ = declare_parameter(desc.name, sensor2_is_cam_);

    desc.name = "publish_tf";
    desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    desc.description = "";
    publish_tf_ = declare_parameter(desc.name, true);

    desc.name = "csv_name";
    desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    desc.description = "";
    csv_name_ = declare_parameter(desc.name, "registration_" + currentDateTime() + ".csv");
}

Eigen::Affine3f Calibration::calibrateExtrinsics(std::vector<pcl::PointXYZ> &sensor1_pcl, std::vector<pcl::PointXYZ> &sensor2_pcl)
{
    std::vector<Eigen::Vector3f> sensor1_pts, sensor2_pts;
    for (ushort i = 0; i < TARGET_NUM_CIRCLES; i++)
    {
        Eigen::Vector3f point;

        point = sensor1_pcl[i].getVector3fMap();
        sensor1_pts.push_back(point);

        point = sensor2_pcl[i].getVector3fMap();
        sensor2_pts.push_back(point);
    }

    // Get the board centroids
    Eigen::Vector3f board1_centroids(0.0, 0.0, 0.0);
    Eigen::Vector3f board2_centroids(0.0, 0.0, 0.0);
    for (ushort i = 0; i < TARGET_NUM_CIRCLES; i++)
    {
        board1_centroids += sensor1_pts.at(i);
        board2_centroids += sensor2_pts.at(i);
    }
    board1_centroids /= TARGET_NUM_CIRCLES;
    board2_centroids /= TARGET_NUM_CIRCLES;

    // Relative rotation of sensor 1 in sensor 2 using SVD
    Eigen::Vector3f sensor_rotation(0.0, 0.0, 0.0);
    for (ushort j = 0; j < TARGET_NUM_CIRCLES; j++)
    {
        Eigen::Vector3f vector1_point = sensor1_pts[j] - board1_centroids;
        Eigen::Vector3f vector2_point = sensor2_pts[j] - board2_centroids;

        Eigen::Quaternionf quaternion;
        quaternion.setFromTwoVectors(vector1_point, vector2_point);

        sensor_rotation += quaternion.toRotationMatrix().eulerAngles(0, 1, 2);
    }
    sensor_rotation /= TARGET_NUM_CIRCLES;

    Eigen::Quaternionf quaternion;

    quaternion = Eigen::AngleAxisf(sensor_rotation[0], Eigen::Vector3f::UnitX()) *
                 Eigen::AngleAxisf(sensor_rotation[1], Eigen::Vector3f::UnitY()) *
                 Eigen::AngleAxisf(sensor_rotation[2], Eigen::Vector3f::UnitZ());

    // Relative translation of sensor 1 in sensor 2
    Eigen::Vector3f sensor_translation(0.0, 0.0, 0.0);
    for (ushort j = 0; j < TARGET_NUM_CIRCLES; j++)
    {
        sensor_translation += board2_centroids - (quaternion * board1_centroids);
    }
    sensor_translation /= TARGET_NUM_CIRCLES;

    Eigen::Matrix4f mat;
    mat.setIdentity();
    mat.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
    mat.block<3, 1>(0, 3) = sensor_translation;

    Eigen::Affine3f tf_sensor1_sensor2(mat);

    if (DEBUG)
    {
        Eigen::VectorXf pose(6, 1);
        pose.block<3, 1>(0, 0) = sensor_translation;
        pose.block<3, 1>(3, 0) = sensor_rotation;

        RCLCPP_INFO(get_logger(), "The transformation of sensor 1 to sensor 2 is: [%f, %f, %f, %f, %f, %f]", pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);

        Eigen::Affine3f tf_sensor2_sensor1 = tf_sensor1_sensor2.inverse();
        pose.block<3, 1>(0, 0) = tf_sensor2_sensor1.translation();
        pose.block<3, 1>(3, 0) = tf_sensor2_sensor1.rotation().eulerAngles(0, 1, 2);

        RCLCPP_INFO(get_logger(), "The transformation of sensor 2 to sensor 1 is: [%f, %f, %f, %f, %f, %f]", pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
    }

    return tf_sensor1_sensor2;
}

void Calibration::callback(const calibration::msg::CircleCentroids::ConstSharedPtr sensor1_centroids, const calibration::msg::CircleCentroids::ConstSharedPtr sensor2_centroids)
{

    RCLCPP_INFO(this->get_logger(), "Cluster pair correspondence received!.");

    sensor1_frame_id_ = sensor1_centroids->header.frame_id;
    sensor2_frame_id_ = sensor2_centroids->header.frame_id;

    pcl::PointCloud<pcl::PointXYZ>::Ptr sensor1_cloud(new pcl::PointCloud<pcl::PointXYZ>),
        sensor2_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr isensor1_cloud(new pcl::PointCloud<pcl::PointXYZI>),
        isensor2_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    fromROSMsg(sensor1_centroids->centers, *sensor1_cloud);
    fromROSMsg(sensor2_centroids->centers, *sensor2_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr aux(new pcl::PointCloud<pcl::PointXYZ>);

    std::string sensor1_real_frame_id, sensor2_real_frame_id;
    if (sensor1_centroids->sensor_type != sensor1_centroids->LIDAR)
    {
        camera_to_lidar(sensor1_cloud, aux);
        sensor1_cloud.swap(aux);
        sensor1_real_frame_id = "rotated_" + sensor1_frame_id_;
    }
    else
    {
        sensor1_real_frame_id = sensor1_frame_id_;
    }

    if (sensor2_centroids->sensor_type != sensor1_centroids->LIDAR)
    {
        camera_to_lidar(sensor2_cloud, aux);
        sensor2_cloud.swap(aux);
        sensor2_real_frame_id = "rotated_" + sensor2_frame_id_;
    }
    else
    {
        sensor2_real_frame_id = sensor2_frame_id_;
    }

    std::vector<pcl::PointXYZ> *sensor1_vector(new std::vector<pcl::PointXYZ>);
    std::vector<pcl::PointXYZ> *sensor2_vector(new std::vector<pcl::PointXYZ>);

    sortPatternCenters(sensor1_cloud, *sensor1_vector);
    sortPatternCenters(sensor2_cloud, *sensor2_vector);

    if (DEBUG)
    {
        sensor_msgs::msg::PointCloud2 colour_cloud;

        colourCenters(*sensor1_vector, isensor1_cloud);
        pcl::toROSMsg(*isensor1_cloud, colour_cloud);
        colour_cloud.header.frame_id = sensor1_real_frame_id;
        colour_sensor1_pub_->publish(colour_cloud);

        colourCenters(*sensor2_vector, isensor2_cloud);
        pcl::toROSMsg(*isensor2_cloud, colour_cloud);
        colour_cloud.header.frame_id = sensor2_real_frame_id;
        colour_sensor2_pub_->publish(colour_cloud);
    }

    Eigen::Affine3f tf_sensor1_sensor2 = calibrateExtrinsics(*sensor1_vector, *sensor2_vector);

    if (publish_tf_)
    {
        geometry_msgs::msg::TransformStamped transform;

        // Broadcast transform
        transform = tf2::eigenToTransform((Eigen::Affine3d)tf_sensor1_sensor2);
        transform.header.stamp = get_clock()->now();
        RCLCPP_INFO(get_logger(), "Broadcasting transformation from %s to %s", sensor2_real_frame_id.c_str(), sensor1_real_frame_id.c_str());

        transform.header.frame_id = sensor2_real_frame_id.c_str();
        transform.child_frame_id = sensor1_real_frame_id.c_str();

        static auto tf_broadcaster = tf2_ros::StaticTransformBroadcaster(this);
        tf_broadcaster.sendTransform(transform);
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<Calibration>();

    rclcpp::spin(nh);
    return 0;
}