/**
 * @file talker.cpp
 * @author Sai Surya Sriramoju (saisurya@umd.edu)
 * @brief Publishes the a message and the message will be changed on request
 * @version 0.1
 * @date 2023-11-07
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <chrono>
#include <memory>

#include "beginner_tutorials/srv/change_string.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

// using namespace std::chrono_literals;

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
    // declaring the params
    this->declare_parameter("message", "This is the base string!");
    this->declare_parameter("pub_freq", 1000);

    base_string_ = this->get_parameter("message").as_string();
    int pub_frq = this->get_parameter("pub_freq").as_int();
    // created a publisher
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

    service_ = this->create_service<beginner_tutorials::srv::ChangeString>(
        "change_string",
        std::bind(&MinimalPublisher::changeStringRequest, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Node initialized with base string: " << base_string_);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(pub_frq),
        std::bind(&MinimalPublisher::timer_callback, this));
    MinimalPublisher::tf_static_broadcaster_ =
    std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    this->broadcast_transforms();
  }

 private:
  /**
   * @brief call back function which publishes the string message
   *
   */
  void timer_callback() {
    if (base_string_ == "None") {
      RCLCPP_FATAL(this->get_logger(),
                   "Base string is empty. Send a request again!");
      return;
    }
    auto message = std_msgs::msg::String();
    message.data = base_string_;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  /**
   * @brief method to broadcast frames from world to talk frame
   * 
   */


  void broadcast_transforms() {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = "talk";
    t.transform.translation.x = 2;
    t.transform.translation.y = 2;
    t.transform.translation.z = 0;
    tf2::Quaternion q;
    q.setRPY(0, 0, 10);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    tf_static_broadcaster_->sendTransform(t);
    RCLCPP_INFO_STREAM(this->get_logger(),
    "Broadcasted transform from world to talk");
  }

   /**
   * @brief change_string service
   *
   * @param request
   * @param response
   */

  void changeStringRequest(
      const std::shared_ptr<beginner_tutorials::srv::ChangeString::Request>
          request,
      std::shared_ptr<beginner_tutorials::srv::ChangeString::Response>
          response) {
    if (request->change_string.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Received an empty string");
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
  std_msgs::msg::String param_message_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
