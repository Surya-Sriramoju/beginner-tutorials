#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "beginner_tutorials/msg/custom.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<beginner_tutorials::msg::Custom>("topic", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = beginner_tutorials::msg::Custom();
    message.text = "This is my custom message";                                  
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.text.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<beginner_tutorials::msg::Custom>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}