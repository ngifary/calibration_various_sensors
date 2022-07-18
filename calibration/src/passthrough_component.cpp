#include "composition/passthrough_component.hpp"
#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using namespace std::chrono_literals;

namespace composition
{
    // Create a PassThrough "component" that does not subclass the generic rclcpp::Node base class.
    /**
     * Note that "components" don't have to derive from rclcpp::Node.
     * In the case that an object does not inherit from rclcpp::Node, then it must implement:
     * - Constructor that takes `const rclcpp::NodeOptions&`
     * - get_node_base_interface() which returns a NodeBaseInterface::SharedPtr
     *
     * This is an example of an object that implements the interface required to be a component.
     */
    PassThrough::PassThrough(const rclcpp::NodeOptions &options)
        : node_(std::make_shared<rclcpp::Node>("passthrough", options)), count_(0), tf_buffer_(this->node_->get_clock()),
          tf_listener_(tf_buffer_)
    {
        setvbuf(stdout, NULL, _IONBF, BUFSIZ);
        // Do not execute if a --help option was provided
        if (help(options.arguments()))
        {
            // TODO(jacobperron): Replace with a mechanism for a node to "unload" itself
            // from a container.
            exit(0);
        }
        parse_parameters();
        initialize();
    }

    void PassThrough::initialize()
    {
        auto qos = rclcpp::QoS(
            rclcpp::QoSInitialization(
                // The history policy determines how messages are saved until taken by
                // the reader.
                // KEEP_ALL saves all messages until they are taken.
                // KEEP_LAST enforces a limit on the number of messages that are saved,
                // specified by the "depth" parameter.
                history_policy_,
                // Depth represents how many messages to store in history when the
                // history policy is KEEP_LAST.
                depth_));
        // The reliability policy can be reliable, meaning that the underlying transport layer will try
        // ensure that every message gets received in order, or best effort, meaning that the transport
        // makes no guarantees about the order or reliability of delivery.
        qos.reliability(reliability_policy_);
        pub_output_ = this->node_->create_publisher<PointCloud2>("output", cloudQoS());

        // TODO(sloretz) subscribe only when there is a subscriber to our output
        subscribe();
        RCLCPP_DEBUG(this->node_->get_logger(), "Node successfully created.");

        // Create a callback function for when messages are received.
        // Variations of this function also exist using, for example, UniquePtr for zero-copy transport.
        // auto callback =
        //     [this](const typename sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void
        // {
        //     // RCLCPP_INFO(this->node_->get_logger(), "I heard: [%s]", msg->data.);
        //     std::flush(std::cout);
        // };

        // Create a subscription to the "chatter" topic which can be matched with one or more
        // compatible ROS publishers.
        // Note that not all publishers on the same topic with the same type will be compatible:
        // they must have compatible Quality of Service policies.
        // sub_ = this->node_->create_subscription<sensor_msgs::msg::PointCloud2>("chatter", 10, callback);

        // Start main timer loop
        timer_ = this->node_->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / freq_)),
            std::bind(&composition::PassThrough::timerCallback, this));
    }

    void PassThrough::timerCallback()
    {
        return;
        // cv::Mat frame;

        // // Initialize a shared pointer to an Image message.
        // auto msg = std::make_unique<sensor_msgs::msg::Image>();
        // msg->is_bigendian = false;

        // // Get the frame from the video capture.
        // if (burger_mode_) {
        //   frame = burger_cap.render_burger(width_, height_);
        // } else {
        //   cap >> frame;
        // }

        // // If no frame was grabbed, return early
        // if (frame.empty()) {
        //   return;
        // }

        // // Conditionally flip the image
        // if (is_flipped_) {
        //   cv::flip(frame, frame, 1);
        // }

        // // Convert to a ROS image
        // convert_frame_to_message(frame, *msg);

        // // Conditionally show image
        // if (show_camera_) {
        //   cv::Mat cvframe = frame;
        //   // Show the image in a window called "cam2image".
        //   cv::imshow("cam2image", cvframe);
        //   // Draw the image to the screen and wait 1 millisecond.
        //   cv::waitKey(1);
        // }

        // // Publish the image message and increment the frame_id.
        // RCLCPP_INFO(this->node_->get_logger(), "Publishing image #%zd", publish_number_++);
        // std::flush(std::cout);
        // pub_->publish(std::move(msg));
    }

    bool PassThrough::help(const std::vector<std::string> args)
    {
        if (std::find(args.begin(), args.end(), "--help") != args.end() ||
            std::find(args.begin(), args.end(), "-h") != args.end())
        {
            std::stringstream ss;
            ss << "Usage: cam2image [-h] [--ros-args [-p param:=value] ...]" << std::endl;
            ss << "Publish images from a camera stream." << std::endl;
            ss << "Example: ros2 run image_tools cam2image --ros-args -p reliability:=best_effort";
            ss << std::endl
               << std::endl;
            ss << "Options:" << std::endl;
            ss << "  -h, --help\tDisplay this help message and exit";
            ss << std::endl
               << std::endl;
            ss << "Parameters:" << std::endl;
            ss << "  reliability\tReliability QoS setting. Either 'reliable' (default) or 'best_effort'";
            ss << std::endl;
            ss << "  history\tHistory QoS setting. Either 'keep_last' (default) or 'keep_all'.";
            ss << std::endl;
            ss << "\t\tIf 'keep_last', then up to N samples are stored where N is the depth";
            ss << std::endl;
            ss << "  depth\t\tDepth of the publisher queue. Only honored if history QoS is 'keep_last'.";
            ss << " Default value is 10";
            ss << std::endl;
            ss << "  frequency\tPublish frequency in Hz. Default value is 30";
            ss << std::endl;
            ss << "  burger_mode\tProduce images of burgers rather than connecting to a camera";
            ss << std::endl;
            ss << "  show_camera\tShow camera stream. Either 'true' or 'false' (default)";
            ss << std::endl;
            ss << "  device_id\tDevice ID of the camera. 0 (default) selects the default camera device.";
            ss << std::endl;
            ss << "  width\t\tWidth component of the camera stream resolution. Default value is 320";
            ss << std::endl;
            ss << "  height\tHeight component of the camera stream resolution. Default value is 240";
            ss << std::endl;
            ss << "  frame_id\t\tID of the sensor frame. Default value is 'camera_frame'";
            ss << std::endl
               << std::endl;
            ss << "Note: try running v4l2-ctl --list-formats-ext to obtain a list of valid values.";
            ss << std::endl;
            std::cout << ss.str();
            return true;
        }
        return false;
    }

    rcl_interfaces::msg::SetParametersResult PassThrough::config_callback(const std::vector<rclcpp::Parameter> &params)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        double filter_min, filter_max;
        impl_.getFilterLimits(filter_min, filter_max);

        for (const rclcpp::Parameter &param : params)
        {
            if (param.get_name() == "filter_field_name")
            {
                // Check the current value for the filter field
                // std::string filter_field = impl_.getFilterFieldName ();
                if (impl_.getFilterFieldName() != param.as_string())
                {
                    // Set the filter field if different
                    impl_.setFilterFieldName(param.as_string());
                    RCLCPP_DEBUG(this->node_->get_logger(), "Setting the filter field name to: %s.", param.as_string().c_str());
                }
            }
            if (param.get_name() == "filter_limit_min")
            {
                // Check the current values for filter min-max
                if (filter_min != param.as_double())
                {
                    filter_min = param.as_double();
                    RCLCPP_DEBUG(this->node_->get_logger(), "Setting the minimum filtering value a point will be considered from to: %f.", filter_min);
                    // Set the filter min-max if different
                    impl_.setFilterLimits(filter_min, filter_max);
                }
            }
            if (param.get_name() == "filter_limit_max")
            {
                // Check the current values for filter min-max
                if (filter_max != param.as_double())
                {
                    filter_max = param.as_double();
                    RCLCPP_DEBUG(this->node_->get_logger(), "Setting the maximum filtering value a point will be considered from to: %f.", filter_max);
                    // Set the filter min-max if different
                    impl_.setFilterLimits(filter_min, filter_max);
                }
            }
            if (param.get_name() == "filter_limit_negative")
            {
                // Check the current value for the negative flag
                if (impl_.getNegative() != param.as_bool())
                {
                    RCLCPP_DEBUG(this->node_->get_logger(), "Setting the filter negative flag to: %s.", param.as_bool() ? "true" : "false");
                    // Call the virtual method in the child
                    impl_.setNegative(param.as_bool());
                }
            }
            if (param.get_name() == "keep_organized")
            {
                // Check the current value for keep_organized
                if (impl_.getKeepOrganized() != param.as_bool())
                {
                    RCLCPP_DEBUG(this->node_->get_logger(), "Setting the filter keep_organized value to: %s.", param.as_bool() ? "true" : "false");
                    // Call the virtual method in the child
                    impl_.setKeepOrganized(param.as_bool());
                }
            }

            // The following parameters are updated automatically for all PCL_ROS2 Nodelet Filters as they are inexistent in PCL
            if (param.get_name() == "input_frame")
            {
                if (tf_input_frame_ != param.as_string())
                {
                    tf_input_frame_ = param.as_string();
                    RCLCPP_DEBUG(this->node_->get_logger(), "Setting the input TF frame to: %s.", tf_input_frame_.c_str());
                }
            }
            if (param.get_name() == "output_frame")
            {
                if (tf_output_frame_ != param.as_string())
                {
                    tf_output_frame_ = param.as_string();
                    RCLCPP_DEBUG(this->node_->get_logger(), "Setting the output TF frame to: %s.", tf_output_frame_.c_str());
                }
            }
        }
        // TODO(sloretz) constraint validation
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }

    void PassThrough::parse_parameters()
    {
        {
            rcl_interfaces::msg::ParameterDescriptor desc;
            desc.name = "max_queue_size";
            desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
            desc.description = "QoS History depth";
            desc.read_only = true;
            max_queue_size_ = this->node_->declare_parameter(desc.name, max_queue_size_, desc);
        }

        {
            rcl_interfaces::msg::ParameterDescriptor desc;
            desc.name = "use_indices";
            desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
            desc.description = "Only process a subset of the point cloud from an indices topic";
            desc.read_only = true;
            use_indices_ = this->node_->declare_parameter(desc.name, use_indices_, desc);
        }

        {
            rcl_interfaces::msg::ParameterDescriptor desc;
            desc.name = "latched_indices";
            desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
            desc.description = "Does indices topic use transient local documentation";
            desc.read_only = true;
            latched_indices_ = this->node_->declare_parameter(desc.name, latched_indices_, desc);
        }

        {
            rcl_interfaces::msg::ParameterDescriptor desc;
            desc.name = "approximate_sync";
            desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
            desc.description = "Match indices and point cloud messages if time stamps are approximatly the same.";
            desc.read_only = true;
            approximate_sync_ = this->node_->declare_parameter(desc.name, approximate_sync_, desc);
        }

        RCLCPP_DEBUG(this->node_->get_logger(), "PCL Node successfully created with the following parameters:\n"
                                                " - approximate_sync : %s\n"
                                                " - use_indices      : %s\n"
                                                " - latched_indices  : %s\n"
                                                " - max_queue_size   : %d",
                     (approximate_sync_) ? "true" : "false",
                     (use_indices_) ? "true" : "false",
                     (latched_indices_) ? "true" : "false",
                     max_queue_size_);

        rcl_interfaces::msg::ParameterDescriptor ffn_desc;
        ffn_desc.name = "filter_field_name";
        ffn_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
        ffn_desc.description = "The field name used for filtering";
        this->node_->declare_parameter(ffn_desc.name, rclcpp::ParameterValue("z"), ffn_desc);

        rcl_interfaces::msg::ParameterDescriptor flmin_desc;
        flmin_desc.name = "filter_limit_min";
        flmin_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        flmin_desc.description = "The minimum allowed field value a point will be considered from";
        rcl_interfaces::msg::FloatingPointRange flmin_range;
        flmin_range.from_value = -100000.0;
        flmin_range.to_value = 100000.0;
        flmin_desc.floating_point_range.push_back(flmin_range);
        this->node_->declare_parameter(flmin_desc.name, rclcpp::ParameterValue(0.0), flmin_desc);

        rcl_interfaces::msg::ParameterDescriptor flmax_desc;
        flmax_desc.name = "filter_limit_max";
        flmax_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        flmax_desc.description = "The maximum allowed field value a point will be considered from";
        rcl_interfaces::msg::FloatingPointRange flmax_range;
        flmax_range.from_value = -100000.0;
        flmax_range.to_value = 100000.0;
        flmax_desc.floating_point_range.push_back(flmax_range);
        this->node_->declare_parameter(flmax_desc.name, rclcpp::ParameterValue(1.0), flmax_desc);

        rcl_interfaces::msg::ParameterDescriptor flneg_desc;
        flneg_desc.name = "filter_limit_negative";
        flneg_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
        flneg_desc.description = "Set to true if we want to return the data outside [filter_limit_min; filter_limit_max].";
        this->node_->declare_parameter(flneg_desc.name, rclcpp::ParameterValue(false), flneg_desc);

        rcl_interfaces::msg::ParameterDescriptor keep_organized_desc;
        keep_organized_desc.name = "keep_organized";
        keep_organized_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
        keep_organized_desc.description = "Set whether the filtered points should be kept and set to NaN, or removed from the PointCloud, thus potentially breaking its organized structure.";
        this->node_->declare_parameter(keep_organized_desc.name, rclcpp::ParameterValue(false), keep_organized_desc);

        rcl_interfaces::msg::ParameterDescriptor input_frame_desc;
        input_frame_desc.name = "input_frame";
        input_frame_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
        input_frame_desc.description = "The input TF frame the data should be transformed into before processing, if input.header.frame_id is different.";
        this->node_->declare_parameter(input_frame_desc.name, rclcpp::ParameterValue(""), input_frame_desc);

        rcl_interfaces::msg::ParameterDescriptor output_frame_desc;
        output_frame_desc.name = "output_frame";
        output_frame_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
        output_frame_desc.description = "The output TF frame the data should be transformed into after processing, if input.header.frame_id is different.";
        this->node_->declare_parameter(output_frame_desc.name, rclcpp::ParameterValue(""), output_frame_desc);

        // this->node_->add_on_set_parameters_callback(std::bind(&PassThrough::config_callback, this, _1));
        callback_handle_ = this->node_->add_on_set_parameters_callback(std::bind(&PassThrough::config_callback, this, std::placeholders::_1));
        std::vector<std::string> param_names{
            ffn_desc.name,
            flmin_desc.name,
            flmax_desc.name,
            flneg_desc.name,
            keep_organized_desc.name,
            input_frame_desc.name,
            output_frame_desc.name};
        auto result = config_callback(this->node_->get_parameters(param_names));
        if (!result.successful)
        {
            throw std::runtime_error(result.reason);
        }
        // Declare and get remaining parameters
        // depth_ = this->declare_parameter("depth", 10);
        // freq_ = this->declare_parameter("frequency", 30.0);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
    PassThrough::computePublish(const PointCloud2::ConstSharedPtr &input, const IndicesPtr &indices)
    {
        PointCloud2 output;
        // Call the virtual method in the child
        filter(input, indices, output);

        PointCloud2::SharedPtr cloud_tf(new PointCloud2(output)); // set the output by default
        // Check whether the user has given a different output TF frame
        if (!tf_output_frame_.empty() && output.header.frame_id != tf_output_frame_)
        {
            RCLCPP_DEBUG(this->node_->get_logger(), "Transforming output dataset from %s to %s.", output.header.frame_id.c_str(), tf_output_frame_.c_str());
            // Convert the cloud into the different frame
            PointCloud2 cloud_transformed;
            if (!composition::transformPointCloud(tf_output_frame_, output, cloud_transformed, tf_buffer_))
            {
                RCLCPP_ERROR(this->node_->get_logger(), "Error converting output dataset from %s to %s.", output.header.frame_id.c_str(), tf_output_frame_.c_str());
                return;
            }
            cloud_tf.reset(new PointCloud2(cloud_transformed));
        }
        if (tf_output_frame_.empty() && output.header.frame_id != tf_input_orig_frame_)
        // no tf_output_frame given, transform the dataset to its original frame
        {
            RCLCPP_DEBUG(this->node_->get_logger(), "Transforming output dataset from %s back to %s.", output.header.frame_id.c_str(), tf_input_orig_frame_.c_str());
            // Convert the cloud into the different frame
            PointCloud2 cloud_transformed;
            if (!composition::transformPointCloud(tf_input_orig_frame_, output, cloud_transformed, tf_buffer_))
            {
                RCLCPP_ERROR(this->node_->get_logger(), "Error converting output dataset from %s back to %s.", output.header.frame_id.c_str(), tf_input_orig_frame_.c_str());
                return;
            }
            cloud_tf.reset(new PointCloud2(cloud_transformed));
        }

        // Copy timestamp to keep it
        cloud_tf->header.stamp = input->header.stamp;

        // Publish a shared ptr
        pub_output_->publish(*cloud_tf);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    void
    PassThrough::subscribe()
    {
        // If we're supposed to look for PointIndices (indices)
        if (use_indices_)
        {
            // Subscribe to the input using a filter
            sub_input_filter_.subscribe(this->node_->shared_from_this(), "input", cloudQoS().get_rmw_qos_profile());
            sub_indices_filter_.subscribe(this->node_->shared_from_this(), "indices", indicesQoS().get_rmw_qos_profile());

            if (approximate_sync_)
            {
                sync_input_indices_a_ = std::make_shared<message_filters::Synchronizer<sync_policies::ApproximateTime<PointCloud2, pcl_msgs::msg::PointIndices>>>(max_queue_size_);
                sync_input_indices_a_->connectInput(sub_input_filter_, sub_indices_filter_);
                auto callback = std::bind(&PassThrough::input_indices_callback, this, std::placeholders::_1, std::placeholders::_2);
                sync_input_indices_a_->registerCallback(callback);
            }
            else
            {
                sync_input_indices_e_ = std::make_shared<message_filters::Synchronizer<sync_policies::ExactTime<PointCloud2, pcl_msgs::msg::PointIndices>>>(max_queue_size_);
                sync_input_indices_e_->connectInput(sub_input_filter_, sub_indices_filter_);
                auto callback = std::bind(&PassThrough::input_indices_callback, this, std::placeholders::_1, std::placeholders::_2);
                sync_input_indices_e_->registerCallback(callback);
            }
        }
        else
        {
            // Workaround ros2/rclcpp#766
            std::function<void(PointCloud2::ConstSharedPtr)> callback =
                std::bind(&PassThrough::input_indices_callback, this, std::placeholders::_1, nullptr);

            // Subscribe in an old fashion to input only (no filters)
            sub_input_ = this->node_->create_subscription<PointCloud2>("input", cloudQoS(), callback);
        }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    void
    PassThrough::unsubscribe()
    {
        if (use_indices_)
        {
            sub_input_filter_.unsubscribe();
            sub_indices_filter_.unsubscribe();
        }
        else
        {
            pub_output_.reset();
        }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    void
    PassThrough::input_indices_callback(const PointCloud2::ConstSharedPtr cloud, const pcl_msgs::msg::PointIndices::ConstSharedPtr indices)
    {
        // If cloud is given, check if it's valid
        if (!isValid(cloud))
        {
            RCLCPP_ERROR(this->node_->get_logger(), "Invalid input!");
            return;
        }
        // If indices are given, check if they are valid
        if (indices && !isValid(indices))
        {
            RCLCPP_ERROR(this->node_->get_logger(), "Invalid indices!");
            return;
        }

        /// DEBUG
        if (indices)
        {
            RCLCPP_DEBUG(this->node_->get_logger(), "[input_indices_callback]\n"
                                                    "                                 - PointCloud with %d data points (%s), stamp %d, and frame %s on topic %s received.\n"
                                                    "                                 - PointIndices with %zu values, stamp %d, and frame %s on topic %s received.",
                         cloud->width * cloud->height, pcl::getFieldsList(*cloud).c_str(), cloud->header.stamp.sec, cloud->header.frame_id.c_str(), "input",
                         indices->indices.size(), indices->header.stamp.sec, indices->header.frame_id.c_str(), "indices");
        }
        else
        {
            RCLCPP_DEBUG(this->node_->get_logger(), "PointCloud with %d data points and frame %s on topic %s received.", cloud->width * cloud->height, cloud->header.frame_id.c_str(), "input");
        }
        ///

        // Check whether the user has given a different input TF frame
        tf_input_orig_frame_ = cloud->header.frame_id;
        PointCloud2::ConstSharedPtr cloud_tf;
        if (!tf_input_frame_.empty() && cloud->header.frame_id != tf_input_frame_)
        {
            RCLCPP_DEBUG(this->node_->get_logger(), "Transforming input dataset from %s to %s.", cloud->header.frame_id.c_str(), tf_input_frame_.c_str());
            // Save the original frame ID
            // Convert the cloud into the different frame
            PointCloud2 cloud_transformed;
            if (!composition::transformPointCloud(tf_input_frame_, *cloud, cloud_transformed, tf_buffer_))
            {
                RCLCPP_ERROR(this->node_->get_logger(), "Error converting input dataset from %s to %s.", cloud->header.frame_id.c_str(), tf_input_frame_.c_str());
                return;
            }
            cloud_tf = std::make_shared<PointCloud2>(cloud_transformed);
        }
        else
        {
            cloud_tf = cloud;
        }

        // Need setInputCloud () here because we have to extract x/y/z
        IndicesPtr vindices;
        if (indices)
        {
            vindices.reset(new std::vector<int>(indices->indices));
        }

        computePublish(cloud_tf, vindices);
    }

} // namespace composition

#include "rclcpp_components/register_node_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(composition::PassThrough)

// int main (int argc, char** argv)
// {
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

// //   // Fill in the cloud data
// //   cloud->width  = 5;
// //   cloud->height = 1;
// //   cloud->points.resize (cloud->width * cloud->height);

// //   for (auto& point: *cloud)
// //   {
// //     point.x = 1024 * rand () / (RAND_MAX + 1.0f);
// //     point.y = 1024 * rand () / (RAND_MAX + 1.0f);
// //     point.z = 1024 * rand () / (RAND_MAX + 1.0f);
// //   }

// //   std::cerr << "Cloud before filtering: " << std::endl;
// //   for (const auto& point: *cloud)
// //     std::cerr << "    " << point.x << " "
// //                         << point.y << " "
// //                         << point.z << std::endl;

//   // Create the filtering object
//   pcl::PassThrough<pcl::PointXYZ> pass;
//   pass.setInputCloud (cloud);
//   pass.setFilterFieldName ("z");
//   pass.setFilterLimits (0.0, 1.0);
//   //pass.setFilterLimitsNegative (true);
//   pass.filter (*cloud_filtered);

// //   std::cerr << "Cloud after filtering: " << std::endl;
// //   for (const auto& point: *cloud_filtered)
// //     std::cerr << "    " << point.x << " "
// //                         << point.y << " "
// //                         << point.z << std::endl;

//   return (0);
// }