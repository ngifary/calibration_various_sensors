#ifndef COMPOSITION__PASSTHROUGH_COMPONENT_HPP_
#define COMPOSITION__PASSTHROUGH_COMPONENT_HPP_

#include "composition/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace composition
{

class PassThrough : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit PassThrough(const rclcpp::NodeOptions & options);

protected:
  void on_timer();

private:
  size_t count_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace composition

#endif  // COMPOSITION__PASSTHROUGH_COMPONENT_HPP_
