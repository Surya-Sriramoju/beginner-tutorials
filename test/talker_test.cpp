/**
 * @file talker_test.cpp
 * @author Sai Surya Sriramoju
 * @date 11/21/2023
 * @version 1.0
 * 
 * @brief Integration tests for beginner_tutorials package
 * The tests are performed to make sure the messages are published
 * on the topic "topic"
 * @copyright Copyright (c) 2023
 */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

using namespace std::chrono_literals;

/**
 * @class IntegrationTestNode
 * @brief Test fixture for the Talker node.
 * 
 * Setting up a ros2 node for testing if the talker node is publishing a message
 */
class IntegrationTestNode : public testing::Test {
 protected:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    bool resultData_ = false;

    void SetUp() override {
        rclcpp::init(0, nullptr);
        node_ = std::make_shared<rclcpp::Node>("integration_test");
        subscription_ = node_->create_subscription<std_msgs::msg::String>(
            "topic", 10,
            [this](const std_msgs::msg::String& msg) {
                RCLCPP_INFO(node_->get_logger(),
                "I heard: '%s'", msg.data.c_str());
                resultData_ = true;
            });
    }

    void TearDown() override {
        subscription_.reset();  // Reset the subscription
        node_.reset();         // Reset the node
        rclcpp::shutdown();    // Shutdown ROS2
    }
};

/**
 * @brief Tests if talker is publishing messages
 * 
 * makes sure that the Talker node publishes messages to the "topic" topic.
 */
TEST_F(IntegrationTestNode, TalkerPublishesMessage) {
    auto start = node_->now();
    while ((node_->now() - start) < 5s && !resultData_) {
        rclcpp::spin_some(node_);
    }

    EXPECT_TRUE(resultData_);
}

/**
 * @brief Main function for running the tests.
 * 
 * 
 * @return int Execution status of the tests
 * 
 * runs the tests
 */
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
