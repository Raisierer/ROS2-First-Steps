#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node
{
private:
  int counter;
  rclcpp::TimerBase::SharedPtr timer_;

public:
  MyNode() : Node("cpp_test"), counter(0)
  {
    RCLCPP_INFO(this->get_logger(), "Hello Cpp Node");
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&MyNode::timerCallback, this));
  }

private:
  void timerCallback()
  {
    counter++;
    RCLCPP_INFO(this->get_logger(), "Hello ROS2 the %d.", counter);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MyNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}