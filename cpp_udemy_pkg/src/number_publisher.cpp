#include "rclcpp/rclcpp.hpp"

#include "example_interfaces/msg/int64.hpp"

class NumberPublisherNode : public rclcpp::Node
{
private:
  int number;

  rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

public:
  NumberPublisherNode() : Node("number_publisher"), number(2)
  {
    publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number", 10);
    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&NumberPublisherNode::publishNumber, this));

    RCLCPP_INFO(this->get_logger(), "Number Publisher has been started.");
  }

private:
  void publishNumber()
  {
    auto msg = example_interfaces::msg::Int64();
    msg.data = number;
    publisher_->publish(msg);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<NumberPublisherNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}