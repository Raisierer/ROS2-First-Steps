#include "rclcpp/rclcpp.hpp"

#include "example_interfaces/srv/add_two_ints.hpp"

class AddTwoIntsClientNode : public rclcpp::Node
{
private:
  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
  std::thread thread1_;
  // TODO: add Thread Pool:
  // https://stackoverflow.com/questions/15752659/thread-pooling-in-c11

public:
  AddTwoIntsClientNode() : Node("add_two_ints_client")
  {
    RCLCPP_INFO(this->get_logger(), "Add Two Ints Client has been started.");

    callAddTwoInts(5, 7);
  }

  void callAddTwoInts(int a, int b)
  {
    thread1_ = std::thread(std::bind(&AddTwoIntsClientNode::callbackAddTwoInts, this, a, b));
  }

private:
  void callbackAddTwoInts(int a, int b)
  {
    client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
    while (!client_->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_WARN(this->get_logger(), "Waiting for Add Two Ins Service...");
    }

    auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    request->a = a;
    request->b = b;

    auto future = client_->async_send_request(request);

    try
    {
      auto response = future.get();
      RCLCPP_INFO(this->get_logger(), "%d + %d = %d", request->a, request->b, response.get()->sum);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Service call failed!");
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<AddTwoIntsClientNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}