#include <chrono>
#include <memory>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class JointPositionPublisher : public rclcpp::Node
{
public:
  JointPositionPublisher()
  : Node("joint_position_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/position_controller/commands", 10);
    timer_ = this->create_wall_timer(
      2000ms, std::bind(&JointPositionPublisher::timer_callback, this));
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&JointPositionPublisher::joint_state_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "joint position publisher started");
    RCLCPP_INFO(this->get_logger(), "publication on: /position_controller/commands");
  }

private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
	{
	  RCLCPP_INFO(this->get_logger(), "--- JOINT STATES RECEIVED ---");
	  
	  for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
	    RCLCPP_INFO(this->get_logger(), 
	      "  Joint '%s': position = %.4f rad (%.2f°)", 
	      msg->name[i].c_str(), 
	      msg->position[i],
	      msg->position[i] * 180.0 / M_PI);
	  }
	}
  void timer_callback()
  {
    auto message = std_msgs::msg::Float64MultiArray();
   
    switch(count_ % 2)
    {
      case 0:
      message.data = {0.0, 0.0, 0.0, 0.0};
      RCLCPP_INFO(this->get_logger(), "Position 1: Home [0, 0, 0, 0]");
      break;
    
    case 1:
      message.data = {1.5, 0.0, 0.0, 0.0};
      RCLCPP_INFO(this->get_logger(), "Position 2: Base rotation [1.5, 0, 0, 0]");
      break;
    
    case 2:
      message.data = {1.5, 1.0, 0.5, 0.0};
      RCLCPP_INFO(this->get_logger(), "Position 3: Arm up [1.5, 1.0, 0.5, 0]");
      break;
    
    case 3:
      message.data = {0.0, 1.5, 1.5, 1.0};
      RCLCPP_INFO(this->get_logger(), "Position 4: Extended [0, 1.5, 1.5, 1.0]");
      break;
    
    case 4:
      message.data = {-1.5, -1.0, -0.5, -1.0};
      RCLCPP_INFO(this->get_logger(), "Position 5: Negative [-1.5, -1.0, -0.5, -1.0]");
      break;
    }
    
    publisher_->publish(message);
    count_++;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointPositionPublisher>());
  rclcpp::shutdown();
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  return 0;
}
