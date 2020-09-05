#include "rclcpp/rclcpp.hpp"

#include "example_interfaces/msg/int64.hpp"

class NumberCounterNode : public rclcpp::Node
{
private:
  int counter;
  rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;
  rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;

public:
  NumberCounterNode() : Node("number_counter"), counter(0)
  {
    subscriber_ = this->create_subscription<example_interfaces::msg::Int64>(
        "number", 10, std::bind(&NumberCounterNode::callbackNumber, this, std::placeholders::_1));
    publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_counter", 10);

    RCLCPP_INFO(this->get_logger(), "Number Counter has been started.");
  }

private:
  void callbackNumber(const example_interfaces::msg::Int64::SharedPtr msg)
  {
    // RCLCPP_INFO(this->get_logger(), "%d", msg->data);
    counter += msg->data;
    auto new_msg = example_interfaces::msg::Int64();
    new_msg.data = counter;
    publisher_->publish(new_msg);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<NumberCounterNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}