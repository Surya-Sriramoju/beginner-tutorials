/**
 * @file talker.cpp
 * @author Sai Surya Sriramoju (saisurya@umd.edu)
 * @brief Publishes the custom string message
 * @version 0.1
 * @date 2023-11-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "beginner_tutorials/msg/custom.hpp"

using namespace std::chrono_literals;

/**
 * @brief Class for the publisher node
 * 
 */

class MinimalPublisher : public rclcpp::Node {
 public:
/**
 * @brief Construct a new Minimal Publisher object: creating a publisher node
 * 
 */
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0) {
    publisher_ = this->create_publisher<beginner_tutorials::msg::Custom>
    ("topic", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

 private:
/**
 * @brief call back function which publishes the string message
 * 
 */
  void timer_callback() {
    auto message = beginner_tutorials::msg::Custom();
    message.text = "This is my custom message";
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.text.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<beginner_tutorials::msg::Custom>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
