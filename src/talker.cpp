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
// #include "beginner_tutorials/msg/custom.hpp"
#include "std_msgs/msg/string.hpp"
#include "beginner_tutorials/srv/change_string.hpp"

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
  : Node("minimal_publisher"), base_string_("This is the base string!") {

    publisher_ = this->create_publisher<std_msgs::msg::String>
    ("topic", 10);

    service_ = this->create_service<beginner_tutorials::srv::ChangeString>(
      "change_string", std::bind(&MinimalPublisher::changeStringRequest,
      this, std::placeholders::_1, std::placeholders::_2)
    );

    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

 private:
/**
 * @brief call back function which publishes the string message
 * 
 */
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = base_string_;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  void changeStringRequest(const std::shared_ptr
                    <beginner_tutorials::srv::ChangeString::Request>
                    request, std::shared_ptr
                    <beginner_tutorials::srv::ChangeString::Response>
                    response) {

              if(request->change_string.empty()) {
                RCLCPP_ERROR(this->get_logger(),
                "Received an empty string");
                  response->status = false;
                  return;
              }
              base_string_ = request->change_string;
              response->status = true;
              RCLCPP_WARN_STREAM(this->get_logger(),
                    "Base string changed to: " << base_string_);
                    }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<beginner_tutorials::srv::ChangeString>::SharedPtr service_;
  std::string base_string_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
