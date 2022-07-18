#ifndef COMPOSITION__PASSTHROUGH_COMPONENT_HPP_
#define COMPOSITION__PASSTHROUGH_COMPONENT_HPP_

#include <mutex>

#include "composition/visibility_control.h"
#include "composition/ptr_helper.h"
#include "composition/transforms.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl/filters/filter.h"
#include "pcl/filters/passthrough.h"
#include "pcl/conversions.h"
#include "pcl_conversions/pcl_conversions.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <sstream>
#include <utility>
#include <vector>

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

namespace composition
{
  namespace sync_policies = message_filters::sync_policies;

  class PassThrough
  {
  public:
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef PointCloud::Ptr PointCloudPtr;
    typedef PointCloud::ConstPtr PointCloudConstPtr;

    typedef pcl_msgs::msg::PointIndices PointIndices;
    typedef PointIndices::SharedPtr PointIndicesPtr;
    typedef PointIndices::ConstSharedPtr PointIndicesConstPtr;

    typedef pcl_msgs::msg::ModelCoefficients ModelCoefficients;
    typedef ModelCoefficients::SharedPtr ModelCoefficientsPtr;
    typedef ModelCoefficients::ConstSharedPtr ModelCoefficientsConstPtr;

    typedef sensor_msgs::msg::PointCloud2 PointCloud2;

    typedef std::shared_ptr<std::vector<int>> IndicesPtr;
    typedef std::shared_ptr<const std::vector<int>> IndicesConstPtr;

    COMPOSITION_PUBLIC
    explicit PassThrough(const rclcpp::NodeOptions &options);

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

    /** \brief The message filter subscriber for PointIndices. */
    message_filters::Subscriber<PointIndices> sub_indices_filter_;

    /** \brief The output PointCloud publisher. */
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_output_;

    /** \brief The maximum queue size (default: 3). */
    int max_queue_size_ = 3;

    /** \brief True if we use an approximate time synchronizer versus an exact one (false by default). */
    bool approximate_sync_ = false;

    /** \brief TF listener object. */
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    /** \brief Test whether a given PointCloud message is "valid" (i.e., has points, and width and height are non-zero).
     * \param cloud the point cloud to test
     * \param topic_name an optional topic name (only used for printing, defaults to "input")
     */
    inline bool
    isValid(const PointCloud2::ConstSharedPtr &cloud, const std::string &topic_name = "input")
    {
      if (cloud->width * cloud->height * cloud->point_step != cloud->data.size())
      {
        RCLCPP_WARN(this->node_->get_logger(), "Invalid PointCloud (data = %zu, width = %d, height = %d, step = %d) with stamp %d, and frame %s on topic %s received!", cloud->data.size(), cloud->width, cloud->height, cloud->point_step, cloud->header.stamp.sec, cloud->header.frame_id.c_str(), topic_name.c_str());

        return (false);
      }
      return (true);
    }

    /** \brief Test whether a given PointCloud message is "valid" (i.e., has points, and width and height are non-zero).
     * \param cloud the point cloud to test
     * \param topic_name an optional topic name (only used for printing, defaults to "input")
     */
    inline bool
    isValid(const PointCloudConstPtr &cloud, const std::string &topic_name = "input")
    {
      if (cloud->width * cloud->height != cloud->points.size())
      {
        RCLCPP_WARN(this->node_->get_logger(), "Invalid PointCloud (points = %zu, width = %d, height = %d) with stamp %d, and frame %s on topic %s received!", cloud->points.size(), cloud->width, cloud->height, pcl_conversions::fromPCL(cloud->header).stamp.sec, cloud->header.frame_id.c_str(), topic_name.c_str());

        return (false);
      }
      return (true);
    }

    /** \brief Test whether a given PointIndices message is "valid" (i.e., has values).
     * \param indices the point indices message to test
     * \param topic_name an optional topic name (only used for printing, defaults to "indices")
     */
    inline bool
    isValid(const PointIndicesConstPtr &indices, const std::string &topic_name = "indices")
    {
      if (indices->indices.empty())
      {
        RCLCPP_WARN(
            this->node_->get_logger(), "Empty indices (values = %zu) with stamp %d, and frame %s on topic %s received!",
            indices->indices.size(), indices->header.stamp.sec, indices->header.frame_id.c_str(), topic_name.c_str());
        return true;
      }
      return true;
    }

    /** \brief Test whether a given ModelCoefficients message is "valid" (i.e., has values).
     * \param model the model coefficients to test
     * \param topic_name an optional topic name (only used for printing, defaults to "model")
     */
    inline bool
    isValid(const ModelCoefficientsConstPtr &model, const std::string &topic_name = "model")
    {
      if (model->values.empty())
      {
        RCLCPP_WARN(
            this->node_->get_logger(), "Empty model (values = %zu) with stamp %d, and frame %s on topic %s received!",
            model->values.size(), model->header.stamp.sec, model->header.frame_id.c_str(), topic_name.c_str());
        return false;
      }
      return true;
    }

    /* \brief Return QoS settings for indices topic */
    rclcpp::QoS
    indicesQoS() const
    {
      rclcpp::QoS qos(max_queue_size_);
      if (latched_indices_)
      {
        qos.transient_local();
      }
      return qos;
    }

    /* \brief Return QoS settings for point cloud topic */
    rclcpp::QoS
    cloudQoS() const
    {
      rclcpp::QoS qos(max_queue_size_);
      return qos;
    }

    /** \brief Call the actual filter.
     * \param input the input point cloud dataset
     * \param indices the input set of indices to use from \a input
     * \param output the resultant filtered dataset
     */
    void filter(const PointCloud2::ConstSharedPtr &input, const IndicesPtr &indices, PointCloud2 &output)
    {
      std::unique_lock<std::mutex> lock(mutex_);
      pcl::PCLPointCloud2::Ptr pcl_input(new pcl::PCLPointCloud2);
      pcl_conversions::toPCL(*(input), *(pcl_input));
      impl_.setInputCloud(pcl_input);
      auto indices_boost = to_boost_ptr(indices);
      impl_.setIndices(indices_boost);
      pcl::PCLPointCloud2 pcl_output;
      impl_.filter(pcl_output);
      pcl_conversions::moveFromPCL(pcl_output, output);
    }

    /** \brief Parameter callback
     * \param params parameter values to set
     */
    rcl_interfaces::msg::SetParametersResult
    config_callback(const std::vector<rclcpp::Parameter> &params);

    void initialize();
    bool help(const std::vector<std::string> args);
    void parse_parameters();
    void timerCallback();

    /** \brief The input PointCloud subscriber. */
    rclcpp::Subscription<PointCloud2>::SharedPtr sub_input_;
    message_filters::Subscriber<PointCloud2> sub_input_filter_;

    /** \brief The desired user filter field name. */
    std::string filter_field_name_;

    /** \brief The minimum allowed filter value a point will be considered from. */
    double filter_limit_min_;

    /** \brief The maximum allowed filter value a point will be considered from. */
    double filter_limit_max_;

    /** \brief Set to true if we want to return the data outside (\a filter_limit_min_;\a filter_limit_max_). Default: false. */
    bool filter_limit_negative_;

    /** \brief The input TF frame the data should be transformed into, if input.header.frame_id is different. */
    std::string tf_input_frame_;

    /** \brief The original data input TF frame. */
    std::string tf_input_orig_frame_;

    /** \brief The output TF frame the data should be transformed into, if input.header.frame_id is different. */
    std::string tf_output_frame_;

    // /** \brief Virtual abstract filter method. To be implemented by every child.
    //  * \param input the input point cloud dataset.
    //  * \param indices a pointer to the vector of point indices to use.
    //  * \param output the resultant filtered PointCloud2
    //  */
    // virtual void
    // filter(const PointCloud2::ConstSharedPtr &input, const IndicesPtr &indices,
    //        PointCloud2 &output) = 0;

    /** \brief Lazy transport subscribe routine. */
    virtual void
    subscribe();

    /** \brief Lazy transport unsubscribe routine. */
    virtual void
    unsubscribe();

    /** \brief Call the child filter () method, optionally transform the result, and publish it.
     * \param input the input point cloud dataset.
     * \param indices a pointer to the vector of point indices to use.
     */
    void
    computePublish(const PointCloud2::ConstSharedPtr &input, const IndicesPtr &indices);

  private:
    /** \brief Synchronized input, and indices.*/
    std::shared_ptr<message_filters::Synchronizer<sync_policies::ExactTime<PointCloud2, PointIndices>>> sync_input_indices_e_;
    std::shared_ptr<message_filters::Synchronizer<sync_policies::ApproximateTime<PointCloud2, PointIndices>>> sync_input_indices_a_;
    /** \brief PointCloud2 + Indices data callback. */
    void
    input_indices_callback(const PointCloud2::ConstSharedPtr cloud,
                           const PointIndices::ConstSharedPtr indices);
    size_t count_;
    pcl::PassThrough<pcl::PCLPointCloud2> impl_;
    rclcpp::Subscription<PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<PointCloud2>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    /** \brief Internal mutex. */
    std::mutex mutex_;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;

    // ROS parameters
    rclcpp::Node::SharedPtr node_;
    size_t depth_;
    double freq_;
    bool keep_organized_;
    std::string input_frame_, output_frame_;
    rmw_qos_reliability_policy_t reliability_policy_;
    rmw_qos_history_policy_t history_policy_;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace composition

#endif // COMPOSITION__PASSTHROUGH_COMPONENT_HPP_