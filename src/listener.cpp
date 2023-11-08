#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "beginner_tutorials/msg/custom.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<beginner_tutorials::msg::Custom>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const beginner_tutorials::msg::Custom::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: %s", msg->text.c_str());
  }
  rclcpp::Subscription<beginner_tutorials::msg::Custom>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}