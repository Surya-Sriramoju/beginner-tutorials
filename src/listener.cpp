/**
 * @file listener.cpp
 * @author Sai Surya Sriramoju (saisurya@umd.edu)
 * @brief a simple node which subscribes to a certain topic and gets a string message
 * @version 0.1
 * @date 2023-11-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "beginner_tutorials/msg/custom.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node {
 public:
/**
 * @brief Construct a new Minimal Subscriber object: creates subscriber
 * 
 */
  MinimalSubscriber()
  : Node("minimal_subscriber") {
    subscription_ = this->create_subscription
    <beginner_tutorials::msg::Custom>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

 private:
/**
 * @brief callback function which prints the text which is published
 * 
 * @param msg 
 */
  void topic_callback(const
  beginner_tutorials::msg::Custom::SharedPtr msg) const {
    RCLCPP_INFO(this->get_logger(), "I heard: %s", msg->text.c_str());
  }
  rclcpp::Subscription
  <beginner_tutorials::msg::Custom>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
