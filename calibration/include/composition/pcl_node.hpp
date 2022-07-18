#ifndef PCL_NODE_HPP_
#define PCL_NODE_HPP_

#include "composition/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/component_manager.hpp"
#include "std_msgs/msg/string.hpp"

// PCL includes
#include <pcl_msgs/msg/point_indices.hpp>
#include <pcl_msgs/msg/model_coefficients.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
// ROS Node includes
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

// Include TF
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

//
namespace composition
{
    class PCLNode
    {
    public:
        typedef sensor_msgs::msg::PointCloud2 PointCloud2;

        typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
        typedef PointCloud::Ptr PointCloudPtr;
        typedef PointCloud::ConstPtr PointCloudConstPtr;

        typedef pcl_msgs::msg::PointIndices PointIndices;
        typedef PointIndices::SharedPtr PointIndicesPtr;
        typedef PointIndices::ConstSharedPtr PointIndicesConstPtr;

        typedef pcl_msgs::msg::ModelCoefficients ModelCoefficients;
        typedef ModelCoefficients::SharedPtr ModelCoefficientsPtr;
        typedef ModelCoefficients::ConstSharedPtr ModelCoefficientsConstPtr;

        typedef std::shared_ptr<std::vector<int>> IndicesPtr;
        typedef std::shared_ptr<const std::vector<int>> IndicesConstPtr;

        /** \brief Empty constructor. */
        PCLNode(std::string node_name, const rclcpp::NodeOptions &options)
            : node_(std::make_shared<rclcpp::Node>(node_name, options))
        {
            // {
            //     rcl_interfaces::msg::ParameterDescriptor desc;
            //     desc.name = "max_queue_size";
            //     desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
            //     desc.description = "QoS History depth";
            //     desc.read_only = true;
            //     max_queue_size_ = this->node_->declare_parameter(desc.name, max_queue_size_, desc);
            // }

            // {
            //     rcl_interfaces::msg::ParameterDescriptor desc;
            //     desc.name = "use_indices";
            //     desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
            //     desc.description = "Only process a subset of the point cloud from an indices topic";
            //     desc.read_only = true;
            //     use_indices_ = this->node_->declare_parameter(desc.name, use_indices_, desc);
            // }

            // {
            //     rcl_interfaces::msg::ParameterDescriptor desc;
            //     desc.name = "latched_indices";
            //     desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
            //     desc.description = "Does indices topic use transient local documentation";
            //     desc.read_only = true;
            //     latched_indices_ = this->node_->declare_parameter(desc.name, latched_indices_, desc);
            // }

            // {
            //     rcl_interfaces::msg::ParameterDescriptor desc;
            //     desc.name = "approximate_sync";
            //     desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
            //     desc.description = "Match indices and point cloud messages if time stamps are approximatly the same.";
            //     desc.read_only = true;
            //     approximate_sync_ = this->node_->declare_parameter(desc.name, approximate_sync_, desc);
            // }

            RCLCPP_DEBUG(this->node_->get_logger(), "PCL Node successfully created with the following parameters:\n"
                                             " - approximate_sync : %s\n"
                                             " - use_indices      : %s\n"
                                             " - latched_indices  : %s\n"
                                             " - max_queue_size   : %d",
                         (approximate_sync_) ? "true" : "false",
                         (use_indices_) ? "true" : "false",
                         (latched_indices_) ? "true" : "false",
                         max_queue_size_);
        }

    COMPOSITION_PUBLIC
        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
        get_node_base_interface() const;

protected:
    /** \brief Set to true if point indices are used.
     *
     * When receiving a point cloud, if use_indices_ is false, the entire
     * point cloud is processed for the given operation. If use_indices_ is
     * true, then the ~indices topic is read to get the vector of point
     * indices specifying the subset of the point cloud that will be used for
     * the operation. In the case where use_indices_ is true, the ~input and
     * ~indices topics must be synchronised in time, either exact or within a
     * specified jitter. See also @ref latched_indices_ and approximate_sync.
     **/
    bool use_indices_ = false;
    /** \brief Set to true if the indices topic has transient_local durability.
     *
     * If use_indices_ is true, the ~input and ~indices topics generally must
     * be synchronised in time. By setting this flag to true, the most recent
     * value from ~indices can be used instead of requiring a synchronised
     * message.
     **/
    bool latched_indices_ = false;

    /** \brief The message filter subscriber for PointCloud2. */
    message_filters::Subscriber<PointCloud> sub_input_filter_;

    /** \brief The message filter subscriber for PointIndices. */
    message_filters::Subscriber<PointIndices> sub_indices_filter_;

    /** \brief The output PointCloud publisher. */
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_output_;

    /** \brief The maximum queue size (default: 3). */
    int max_queue_size_ = 3;

    /** \brief True if we use an approximate time synchronizer versus an exact one (false by default). */
    bool approximate_sync_ = false;

    /** \brief TF listener object. */
    // tf2_ros::Buffer tf_buffer_;
    // tf2_ros::TransformListener tf_listener_;

private:
    /* data */
    rclcpp::Node::SharedPtr node_;
};
}

#endif // PCL_NODE_HPP_