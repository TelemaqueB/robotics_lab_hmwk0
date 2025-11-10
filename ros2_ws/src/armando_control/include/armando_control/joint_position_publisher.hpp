#ifndef ARMANDO_CONTROL__JOINT_POSITION_PUBLISHER_HPP_
#define ARMANDO_CONTROL__JOINT_POSITION_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class JointPositionPublisher : public rclcpp::Node
{
public:
  JointPositionPublisher();

private:
  void timer_callback();
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  size_t count_;
};

#endif  // ARMANDO_CONTROL__JOINT_POSITION_PUBLISHER_HPP_
