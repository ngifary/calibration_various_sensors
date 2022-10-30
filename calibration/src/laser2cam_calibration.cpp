/*
  laser2cam_calibration: Perform the registration step
*/

#include "laser2cam_calibration.hpp"

Registration::Registration(const rclcpp::NodeOptions &options = rclcpp::NodeOptions{}) : Node("registration", options),
                                                                                         tf_buffer_(this->get_clock()),
                                                                                         tf_listener_(tf_buffer_)
{
    RCLCPP_INFO(this->get_logger(), "Calibration Starting....");

    initializeParams();

    // Sensor 1 Definition
    sensor1Received_ = false;
    sensor1_cloud_ =
        pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    isensor1_cloud_ =
        pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

    // Sensor 2 Definition
    sensor2Received_ = false;
    sensor2_cloud_ =
        pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    isensor2_cloud_ =
        pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

    // Subs Definition
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    sensor1_sub_ = this->create_subscription<calibration_interfaces::msg::ClusterCentroids>(
        "cloud1", 100, std::bind(&Registration::sensor1_callback, this, std::placeholders::_1));
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    sensor2_sub_ = this->create_subscription<calibration_interfaces::msg::ClusterCentroids>(
        "cloud2", 100, std::bind(&Registration::sensor2_callback, this, std::placeholders::_1));

    // Pubs Definition
    if (DEBUG)
    {
        clusters_sensor2_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("clusters_sensor2", 1);
        clusters_sensor1_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("clusters_sensor1", 1);
        colour_sensor2_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("colour_sensor2", 1);
        colour_sensor1_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("colour_sensor1", 1);
    }
    sensor_switch_pub_ = this->create_publisher<std_msgs::msg::Empty>("warmup_switch", 1);
    iterations_pub_ = this->create_publisher<std_msgs::msg::Int32>("iterations", 1);

    // Internal State
    calibration_ended_ = false;

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

    // Check warmup condition
    if (skip_warmup_)
    {
        S1_WARMUP_DONE = true;
        S2_WARMUP_DONE = true;
        RCLCPP_WARN(this->get_logger(), "Skipping warmup");
    }
    else
    {
        std::cout << "Please, adjust the filters for each sensor before the calibration "
                     "starts."
                  << std::endl;
    }
}

Registration::~Registration()
{
    unsubscribe();

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
        std::cout << "Calibration process finished." << std::endl;

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

void Registration::sensor1_callback(const calibration_interfaces::msg::ClusterCentroids::ConstSharedPtr sensor1_centroids)
{
    sensor1_frame_id_ = sensor1_centroids->header.frame_id;
    if (!S1_WARMUP_DONE)
    {
        S1_WARMUP_COUNT++;
        std::cout << "Clusters from " << sensor1_frame_id_ << ": " << S1_WARMUP_COUNT
                  << "/10" << '\r' << std::flush;
        if (S1_WARMUP_COUNT >= 10) // TODO: Change to param?
        {
            std::cout << std::endl;
            sensor1_sub_.reset();
            sensor2_sub_.reset();

            std::cout << "Clusters from " << sensor1_frame_id_
                      << " received. Is the warmup done? [Y/n]" << std::endl;
            std::string answer;
            answer.clear();
            std::getline(std::cin, answer);
            if (answer == "y" || answer == "Y" || answer == "")
            {
                S1_WARMUP_DONE = !S1_WARMUP_DONE;

                if (!S2_WARMUP_DONE)
                {
                    std::cout << "Filters for sensor 1 are adjusted now. Please, proceed with "
                                 "the other sensor."
                              << std::endl;
                }
                else
                { // Both sensors adjusted
                    std::cout << "Warmup phase completed. Starting calibration phase." << std::endl;
                    std_msgs::msg::Empty myMsg;
                    sensor_switch_pub_->publish(myMsg); //
                }
            }
            else
            { // Reset counter to allow further warmup
                S1_WARMUP_COUNT = 0;
            }
            sensor1_sub_ = this->create_subscription<calibration_interfaces::msg::ClusterCentroids>(
                "cloud1", 100, std::bind(&Registration::sensor1_callback, this, std::placeholders::_1));
            sensor2_sub_ = this->create_subscription<calibration_interfaces::msg::ClusterCentroids>(
                "cloud2", 100, std::bind(&Registration::sensor2_callback, this, std::placeholders::_1));
        }
        return;
    }

    if (!S2_WARMUP_DONE)
    {
        return;
    }

    if (DEBUG)
        RCLCPP_INFO(this->get_logger(), "sensor1 (%s) pattern ready!", sensor1_frame_id_.c_str());

    if (sensor1_buffer_.size() == TARGET_POSITIONS_COUNT)
    {
        sensor1_buffer_.resize(TARGET_POSITIONS_COUNT + 1);
    }

    if (is_sensor1_cam_)
    {
        std::ostringstream sstream;
        geometry_msgs::msg::TransformStamped transformStamped;
        sstream << "rotated_" << sensor1_frame_id_;
        sensor1_rotated_frame_id_ = sstream.str();

        pcl::PointCloud<pcl::PointXYZ>::Ptr xy_sensor1_cloud_(
            new pcl::PointCloud<pcl::PointXYZ>());

        fromROSMsg(sensor1_centroids->cloud, *xy_sensor1_cloud_);

        geometry_msgs::msg::TransformStamped transform;
        tf2_ros::TransformReadyCallback callback; // Investigate this
        try
        {
            tf_buffer_.waitForTransform(sensor1_rotated_frame_id_, sensor1_frame_id_,
                                        tf2::TimePointZero, tf2::durationFromSec(20.0), callback);
            transformStamped = tf_buffer_.lookupTransform(sensor2_rotated_frame_id_, sensor2_frame_id_,
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

        pcl_ros::transformPointCloud(*xy_sensor1_cloud_, *sensor1_cloud_, transform);
    }
    else
    {
        fromROSMsg(sensor1_centroids->cloud, *sensor1_cloud_);
    }

    sensor1Received_ = true;

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

    sensor1_buffer_[TARGET_POSITIONS_COUNT].push_back(
        std::tuple<int, int, pcl::PointCloud<pcl::PointXYZ>,
                   std::vector<pcl::PointXYZ>>(
            sensor1_centroids->total_iterations,
            sensor1_centroids->cluster_iterations, *sensor1_cloud_,
            sensor1_vector_));
    sensor1_count_ = sensor1_centroids->total_iterations;

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

    // Normal operation (sync_iterations_=false)
    if (sensor1Received_ && sensor2Received_)
    {
        std::cout << std::min(sensor1_count_, sensor2_count_) << "/30 iterations" << '\r' << std::flush;

        std_msgs::msg::Int32 it;
        it.data = std::min(sensor1_count_, sensor2_count_);
        iterations_pub_->publish(it);

        if (sensor1_count_ >= TARGET_ITERATIONS &&
            sensor2_count_ >= TARGET_ITERATIONS)
        {
            std::cout << std::endl;
            sensor1_sub_.reset();
            sensor2_sub_.reset();

            std::string answer;
            if (single_pose_mode_)
            {
                answer = "n";
            }
            else
            {
                std::cout << "Target iterations reached. Do you need another target "
                             "location? [y/N]"
                          << std::endl;
                getline(std::cin, answer);
            }
            if (answer == "n" || answer == "N" || answer == "")
            {
                calibrateExtrinsics(-1);
                calibration_ended_ = true;
            }
            else
            { // Move the target and start over
                if (results_every_pose_)
                    calibrateExtrinsics(-1);
                TARGET_POSITIONS_COUNT++;
                std::cout << "Please, move the target to its new position and adjust the "
                             "filters for each sensor before the calibration starts."
                          << std::endl;
                // Start over if other position of the target is required
                std_msgs::msg::Empty myMsg;
                sensor_switch_pub_->publish(myMsg); // Set sensor nodes to warmup phase
                S1_WARMUP_DONE = false;
                S1_WARMUP_COUNT = 0;
                S2_WARMUP_DONE = false;
                S2_WARMUP_COUNT = 0;
                sensor1Received_ = false;
                sensor2Received_ = false;
                sensor1_count_ = 0;
                sensor2_count_ = 0;
            }
            sensor1_sub_ = this->create_subscription<calibration_interfaces::msg::ClusterCentroids>(
                "cloud1", 100, std::bind(&Registration::sensor1_callback, this, std::placeholders::_1));
            sensor2_sub_ = this->create_subscription<calibration_interfaces::msg::ClusterCentroids>(
                "cloud2", 100, std::bind(&Registration::sensor2_callback, this, std::placeholders::_1));
            return;
        }
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
}

void Registration::sensor2_callback(const calibration_interfaces::msg::ClusterCentroids::ConstSharedPtr sensor2_centroids)
{
    RCLCPP_INFO(this->get_logger(), "Sensor 2 callback");
    sensor2_frame_id_ = sensor2_centroids->header.frame_id;
    if (!S2_WARMUP_DONE && S1_WARMUP_DONE)
    {
        S2_WARMUP_COUNT++;
        std::cout << "Clusters from " << sensor2_frame_id_ << ": " << S2_WARMUP_COUNT
                  << "/10" << '\r' << std::flush;
        if (S2_WARMUP_COUNT >= 10) // TODO: Change to param?
        {
            std::cout << std::endl;
            sensor1_sub_.reset();
            sensor2_sub_.reset();

            std::cout << "Clusters from " << sensor2_frame_id_
                      << " received. Is the warmup done? (you can also reset this "
                         "position) [Y/n/r]"
                      << std::endl;
            std::string answer;
            getline(std::cin, answer);
            if (answer == "y" || answer == "Y" || answer == "")
            {
                S2_WARMUP_DONE = !S2_WARMUP_DONE;

                if (!S1_WARMUP_DONE)
                {
                    std::cout << "Filters for sensor 2 are adjusted now. Please, proceed with "
                                 "the other sensor."
                              << std::endl;
                }
                else
                { // Both sensors adjusted
                    std::cout << "Warmup phase completed. Starting calibration phase." << std::endl;
                    std_msgs::msg::Empty myMsg;
                    sensor_switch_pub_->publish(myMsg); //
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
                sensor1Received_ = false;
                sensor2Received_ = false;
                sensor1_count_ = 0;
                sensor2_count_ = 0;
                std::cout << "Please, adjust the filters for each sensor before the "
                             "calibration starts."
                          << std::endl;
            }
            else
            { // Reset counter to allow further warmup
                S2_WARMUP_COUNT = 0;
            }
            sensor1_sub_ = this->create_subscription<calibration_interfaces::msg::ClusterCentroids>(
                "cloud1", 100, std::bind(&Registration::sensor1_callback, this, std::placeholders::_1));
            sensor2_sub_ = this->create_subscription<calibration_interfaces::msg::ClusterCentroids>(
                "cloud2", 100, std::bind(&Registration::sensor2_callback, this, std::placeholders::_1));
        }
        return;
    }
    else if (!S2_WARMUP_DONE)
    {
        return;
    }
    if (DEBUG)
        RCLCPP_INFO(this->get_logger(), "sensor2 (%s) pattern ready!", sensor2_frame_id_.c_str());

    if (sensor2_buffer_.size() == TARGET_POSITIONS_COUNT)
    {
        sensor2_buffer_.resize(TARGET_POSITIONS_COUNT + 1);
    }

    if (is_sensor2_cam_)
    {
        std::ostringstream sstream;
        geometry_msgs::msg::TransformStamped transformStamped;
        sstream << "rotated_" << sensor2_frame_id_;
        sensor2_rotated_frame_id_ = sstream.str();

        pcl::PointCloud<pcl::PointXYZ>::Ptr xy_sensor2_cloud_(
            new pcl::PointCloud<pcl::PointXYZ>());

        fromROSMsg(sensor2_centroids->cloud, *xy_sensor2_cloud_);

        geometry_msgs::msg::TransformStamped transform;
        tf2_ros::TransformReadyCallback callback; // Investigate this
        try
        {
            tf_buffer_.waitForTransform(sensor1_rotated_frame_id_, sensor1_frame_id_,
                                        tf2::TimePointZero, tf2::durationFromSec(20.0), callback);
            transformStamped = tf_buffer_.lookupTransform(sensor2_rotated_frame_id_, sensor2_frame_id_,
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

        pcl_ros::transformPointCloud(*xy_sensor2_cloud_, *sensor2_cloud_, transform);
    }
    else
    {
        fromROSMsg(sensor2_centroids->cloud, *sensor2_cloud_);
    }

    sensor2Received_ = true;

    sortPatternCenters(sensor2_cloud_, sensor2_vector_);

    if (DEBUG)
    {
        colourCenters(sensor2_vector_, isensor2_cloud_);

        sensor_msgs::msg::PointCloud2 colour_cloud_;
        pcl::toROSMsg(*isensor2_cloud_, colour_cloud_);
        colour_cloud_.header.frame_id =
            is_sensor2_cam_ ? sensor2_rotated_frame_id_ : sensor2_frame_id_;
        colour_sensor2_pub_->publish(colour_cloud_);
    }

    sensor2_buffer_[TARGET_POSITIONS_COUNT].push_back(
        std::tuple<int, int, pcl::PointCloud<pcl::PointXYZ>,
                   std::vector<pcl::PointXYZ>>(
            sensor2_centroids->total_iterations,
            sensor2_centroids->cluster_iterations, *sensor2_cloud_,
            sensor2_vector_));
    sensor2_count_ = sensor2_centroids->total_iterations;

    if (DEBUG)
        RCLCPP_INFO(this->get_logger(), "[L2C] sensor2");

    for (std::vector<pcl::PointXYZ>::iterator it = sensor2_vector_.begin();
         it < sensor2_vector_.end(); ++it)
    {
        if (DEBUG)
            std::cout << "c" << it - sensor2_vector_.begin() << "="
                      << "[" << (*it).x << " " << (*it).y << " " << (*it).z << "]" << std::endl;
    }

    // sync_iterations_ is designed to extract a calibration result every single
    // frame, so we cannot wait until TARGET_ITERATIONS
    if (sync_iterations_)
    {
        if (sensor1_count_ >= sensor2_count_)
        {
            calibrateExtrinsics(sensor2_count_);
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

    // Normal operation (sync_iterations_=false)
    if (sensor1Received_ && sensor2Received_)
    {
        std::cout << std::min(sensor1_count_, sensor2_count_) << "/30 iterations" << '\r' << std::flush;

        std_msgs::msg::Int32 it;
        it.data = std::min(sensor1_count_, sensor2_count_);
        iterations_pub_->publish(it);

        if (sensor1_count_ >= TARGET_ITERATIONS &&
            sensor2_count_ >= TARGET_ITERATIONS)
        {
            std::cout << std::endl;
            sensor1_sub_.reset();
            sensor2_sub_.reset();

            std::string answer;
            if (single_pose_mode_)
            {
                answer = "n";
            }
            else
            {
                std::cout << "Target iterations reached. Do you need another target "
                             "location? [y/N]"
                          << std::endl;
                getline(std::cin, answer);
            }
            if (answer == "n" || answer == "N" || answer == "")
            {
                calibrateExtrinsics(-1);
                calibration_ended_ = true;
            }
            else
            { // Move the target and start over
                if (results_every_pose_)
                    calibrateExtrinsics(-1);
                TARGET_POSITIONS_COUNT++;
                std::cout << "Please, move the target to its new position and adjust the "
                             "filters for each sensor before the calibration starts."
                          << std::endl;
                // Start over if other position of the target is required
                std_msgs::msg::Empty myMsg;
                sensor_switch_pub_->publish(myMsg); // Set sensor nodes to warmup phase
                S1_WARMUP_DONE = false;
                S1_WARMUP_COUNT = 0;
                S2_WARMUP_DONE = false;
                S2_WARMUP_COUNT = 0;
                sensor1Received_ = false;
                sensor2Received_ = false;
                sensor1_count_ = 0;
                sensor2_count_ = 0;
            }
            sensor1_sub_ = this->create_subscription<calibration_interfaces::msg::ClusterCentroids>(
                "cloud1", 100, std::bind(&Registration::sensor1_callback, this, std::placeholders::_1));
            sensor2_sub_ = this->create_subscription<calibration_interfaces::msg::ClusterCentroids>(
                "cloud2", 100, std::bind(&Registration::sensor2_callback, this, std::placeholders::_1));
            return;
        }
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