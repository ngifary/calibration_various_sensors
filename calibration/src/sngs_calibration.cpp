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

void Registration::calibrateExtrinsics(int seek_iter = -1)
{
    std::vector<pcl::PointXYZ> local_sensor1_vector_, local_sensor2_vector_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr local_sensor1_cloud_(
        new pcl::PointCloud<pcl::PointXYZ>),
        local_sensor2_cloud_(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> local_l_cloud_, local_c_cloud_;

    int used_sensor2, used_sensor1;

    // Get final frame names for TF broadcaster
    std::string sensor1_final_transformation_frame = sensor1_frame_id_;
    if (is_sensor1_cam_)
    {
        sensor1_final_transformation_frame = sensor1_rotated_frame_id_;
    }
    std::string sensor2_final_transformation_frame = sensor2_frame_id_;
    if (is_sensor2_cam_)
    {
        sensor2_final_transformation_frame = sensor2_rotated_frame_id_;
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
                            i + 1, std::get<0>(sensor2_buffer_[i].back()),
                            std::get<0>(sensor1_buffer_[i].back()));
            // Sensor 1
            auto it1 = std::find_if(
                sensor1_buffer_[i].begin(), sensor1_buffer_[i].end(),
                [&seek_iter](
                    const std::tuple<int, int, pcl::PointCloud<pcl::PointXYZ>,
                                     std::vector<pcl::PointXYZ>> &e)
                {
                    return std::get<0>(e) == seek_iter;
                });
            if (it1 == sensor1_buffer_[i].end())
            {
                RCLCPP_WARN(this->get_logger(), "Could not sync sensor1");
                return;
            }

            local_sensor1_vector_.insert(
                local_sensor1_vector_.end(), std::get<3>(*it1).begin(),
                std::get<3>(*it1).end()); // Add sorted centers (for equations)
            *local_sensor1_cloud_ +=
                std::get<2>(*it1); // Add centers cloud (for registration)
            used_sensor1 = std::get<1>(*it1);
            total_sensor1 = std::get<0>(*it1);

            // Sensor 2
            auto it2 = std::find_if(
                sensor2_buffer_[i].begin(), sensor2_buffer_[i].end(),
                [&seek_iter](
                    const std::tuple<int, int, pcl::PointCloud<pcl::PointXYZ>,
                                     std::vector<pcl::PointXYZ>> &e)
                {
                    return std::get<0>(e) == seek_iter;
                });
            if (it2 == sensor2_buffer_[i].end())
            {
                RCLCPP_WARN(this->get_logger(), "Could not sync sensor2");
                return;
            }

            local_sensor2_vector_.insert(
                local_sensor2_vector_.end(), std::get<3>(*it2).begin(),
                std::get<3>(*it2).end()); // Add sorted centers (for equations)
            *local_sensor2_cloud_ +=
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
            local_sensor1_vector_.insert(
                local_sensor1_vector_.end(),
                std::get<3>(sensor1_buffer_[i].back()).begin(),
                std::get<3>(sensor1_buffer_[i].back())
                    .end()); // Add sorted centers (for equations)
            *local_sensor1_cloud_ += std::get<2>(
                sensor1_buffer_[i].back()); // Add centers cloud (for registration)
            used_sensor1 = std::get<1>(sensor2_buffer_[i].back());

            // Sensor 2
            local_sensor2_vector_.insert(
                local_sensor2_vector_.end(),
                std::get<3>(sensor2_buffer_[i].back()).begin(),
                std::get<3>(sensor2_buffer_[i].back())
                    .end()); // Add sorted centers (for equations)
            *local_sensor2_cloud_ += std::get<2>(
                sensor2_buffer_[i].back()); // Add centers cloud (for registration)
        }
    }

    if (DEBUG)
    {
        sensor_msgs::msg::PointCloud2 ros_cloud_;
        pcl::toROSMsg(*local_sensor2_cloud_, ros_cloud_);
        ros_cloud_.header.frame_id = sensor2_rotated_frame_id_;
        clusters_sensor2_pub_->publish(ros_cloud_);

        pcl::toROSMsg(*local_sensor1_cloud_, ros_cloud_);
        ros_cloud_.header.frame_id = sensor1_frame_id_;
        clusters_sensor1_pub_->publish(ros_cloud_);
    }

    // SVD code
    pcl::PointCloud<pcl::PointXYZ>::Ptr sorted_centers1(
        new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr sorted_centers2(
        new pcl::PointCloud<pcl::PointXYZ>());

    for (unsigned i = 0; i < local_sensor1_vector_.size(); ++i)
    {
        sorted_centers1->push_back(local_sensor1_vector_[i]);
        sorted_centers2->push_back(local_sensor2_vector_[i]);
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

    transf_.setOrigin(origin);
    transf_.setRotation(tfqt);

    static auto br = tf2_ros::StaticTransformBroadcaster(this);
    rclcpp::Time now = this->get_clock()->now();

    tf_sensor1_sensor2_.header.stamp = now;
    tf_sensor1_sensor2_.header.frame_id = sensor1_final_transformation_frame;
    tf_sensor1_sensor2_.child_frame_id = sensor2_final_transformation_frame;
    tf2::convert(transf_.inverse(), tf_sensor1_sensor2_.transform);

    if (publish_tf_)
        br.sendTransform(tf_sensor1_sensor2_);

    tf2::Stamped<tf2::Transform> tf_transform;
    tf2::fromMsg(tf_sensor1_sensor2_, tf_transform);

    tf2::Transform inverse = tf_transform.inverse();
    double roll, pitch, yaw;
    double xt = inverse.getOrigin().getX(), yt = inverse.getOrigin().getY(),
           zt = inverse.getOrigin().getZ();
    inverse.getBasis().getRPY(roll, pitch, yaw);

    if (save_to_file_)
    {
        savefile_ << seek_iter << ", " << xt << ", " << yt << ", " << zt << ", "
                  << roll << ", " << pitch << ", " << yaw << ", " << used_sensor1
                  << ", " << used_sensor2 << ", " << total_sensor1 << ", "
                  << total_sensor2 << std::endl;
    }

    std::cout << std::setprecision(4) << std::fixed;
    std::cout << "Calibration finished succesfully." << std::endl;
    std::cout << "Extrinsic parameters:" << std::endl;
    std::cout << "x = " << xt << "\ty = " << yt << "\tz = " << zt << std::endl;
    std::cout << "roll = " << roll << "\tpitch = " << pitch << "\tyaw = " << yaw << std::endl;

    sensor1Received_ = false;
    sensor2Received_ = false;
}

void Registration::callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr sensor1_centroids, const sensor_msgs::msg::PointCloud2::ConstSharedPtr sensor2_centroids)
{

    RCLCPP_INFO(this->get_logger(), "Cluster pair correspondence received!.");
    RCLCPP_INFO(get_logger(), "[Time stamp] Sensor 1: [%i, %i] and Sensor 2: [%i, %i]", sensor1_centroids->header.stamp.sec, sensor1_centroids->header.stamp.nanosec, sensor2_centroids->header.stamp.sec, sensor2_centroids->header.stamp.nanosec);

    sensor1_frame_id_ = sensor1_centroids->header.frame_id;
    sensor2_frame_id_ = sensor2_centroids->header.frame_id;

    if (is_sensor1_cam_)
    {
        std::ostringstream sstream;
        sstream << "rotated_" << sensor1_frame_id_;
        sensor1_rotated_frame_id_ = sstream.str();

        pcl::PointCloud<pcl::PointXYZ>::Ptr xy_sensor1_cloud_(
            new pcl::PointCloud<pcl::PointXYZ>());

        fromROSMsg(*sensor1_centroids, *xy_sensor1_cloud_);

        geometry_msgs::msg::TransformStamped transform;

        try
        {
            transform = tf_buffer_.lookupTransform(sensor2_rotated_frame_id_, sensor2_frame_id_,
                                                   tf2::TimePointZero, tf2::durationFromSec(20.0));
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

        pcl_ros::transformPointCloud(*xy_sensor1_cloud_, *sensor1_cloud_, transform);
    }
    else
    {
        fromROSMsg(*sensor1_centroids, *sensor1_cloud_);
    }

    sortPatternCenters(sensor1_cloud_, sensor1_vector_);
    if (DEBUG)
    {
        colourCenters(sensor1_vector_, isensor1_cloud_);

        sensor_msgs::msg::PointCloud2 colour_cloud_;
        pcl::toROSMsg(*isensor1_cloud_, colour_cloud_);
        colour_cloud_.header.frame_id =
            is_sensor1_cam_ ? sensor1_rotated_frame_id_ : sensor1_frame_id_;
        colour_sensor1_pub_->publish(colour_cloud_);
    }

    if (DEBUG)
        RCLCPP_INFO(this->get_logger(), "[L2C] sensor1");

    for (std::vector<pcl::PointXYZ>::iterator it = sensor1_vector_.begin();
         it < sensor1_vector_.end(); ++it)
    {
        if (DEBUG)
            std::cout << "l" << it - sensor1_vector_.begin() << "="
                      << "[" << (*it).x << " " << (*it).y << " " << (*it).z << "]" << std::endl;
    }

    // sync_iterations_ is designed to extract a calibration result every single
    // frame, so we cannot wait until TARGET_ITERATIONS
    if (sync_iterations_)
    {
        if (sensor2_count_ >= sensor1_count_)
        {
            calibrateExtrinsics(sensor1_count_);
        }
        else
        {
            if (tf_sensor1_sensor2_.header.frame_id != "" &&
                tf_sensor1_sensor2_.child_frame_id != "")
            {
                static auto br = tf2_ros::StaticTransformBroadcaster(this);
                tf_sensor1_sensor2_.header.stamp = this->get_clock()->now();
                if (publish_tf_)
                    br.sendTransform(tf_sensor1_sensor2_);
            }
        }
        return;
    }

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