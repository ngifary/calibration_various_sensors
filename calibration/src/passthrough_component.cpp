
#include "composition/passthrough_component.hpp"
#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "pcl/point_types.h"
#include "pcl/filters/passthrough.h"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;


namespace composition
{
    // Create a PassThrough "component" that subclasses the generic rclcpp::Node base class.
    // Components get built into shared libraries and as such do not write their own main functions.
    // The process using the component's shared library will instantiate the class as a ROS node.
    PassThrough::PassThrough(const rclcpp::NodeOptions &options)
        : Node("passthrough", options), count_(0)
    {
        // Create a publisher of "std_mgs/String" messages on the "chatter" topic.
        pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);

        // Use a timer to schedule periodic message publishing.
        timer_ = create_wall_timer(1s, std::bind(&PassThrough::on_timer, this));
    }

    void PassThrough::on_timer()
    {
        auto msg = std::make_unique<std_msgs::msg::String>();
        msg->data = "Hello World: " + std::to_string(++count_);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg->data.c_str());
        std::flush(std::cout);

        // Put the message into a queue to be processed by the middleware.
        // This call is non-blocking.
        pub_->publish(std::move(msg));
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