#include "composition/extract_indices_component.hpp"
#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "pcl/point_types.h"
#include "pcl/filters/extract_indices.h"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace composition
{
    // Create a ExtractIndices "component" that subclasses the generic rclcpp::Node base class.
    // Components get built into shared libraries and as such do not write their own main functions.
    // The process using the component's shared library will instantiate the class as a ROS node.
    ExtractIndices::ExtractIndices(const rclcpp::NodeOptions &options)
        : Node("extract_indices", options)
    {
        // Create a callback function for when messages are received.
        // Variations of this function also exist using, for example, UniquePtr for zero-copy transport.
        auto callback =
            [this](const typename std_msgs::msg::String::SharedPtr msg) -> void
        {
            RCLCPP_INFO(this->get_logger(), "I heard: [%s]", msg->data.c_str());
            std::flush(std::cout);
        };

        // Create a subscription to the "chatter" topic which can be matched with one or more
        // compatible ROS publishers.
        // Note that not all publishers on the same topic with the same type will be compatible:
        // they must have compatible Quality of Service policies.
        sub_ = create_subscription<std_msgs::msg::String>("chatter", 10, callback);
    }

} // namespace composition

#include "rclcpp_components/register_node_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(composition::ExtractIndices)