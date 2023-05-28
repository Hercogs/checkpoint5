#include "rcutils/logging.h"
#include <chrono>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <thread>

using geometry_msgs::msg::Twist;
using sensor_msgs::msg::LaserScan;

class PreApproach : public rclcpp::Node {
public:
  PreApproach() : Node("pre_approach_node") {
    // Set logging level
    auto log = rcutils_logging_set_logger_level(this->get_logger().get_name(),
                                                RCUTILS_LOG_SEVERITY_DEBUG);
    (void)log;

    RCLCPP_DEBUG(this->get_logger(), "Pre Approach node created");

    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};

    // Obstacle paramater
    param_desc.description = "Set distance to obstacle";
    this->declare_parameter("obstacle", 0.0, param_desc);
    // Degrees paramater
    param_desc.description = "Set degrees to turn";
    this->declare_parameter<int>("degrees", 0, param_desc);

    this->get_parameter("obstacle", obstacle_param);
    RCLCPP_DEBUG(this->get_logger(), "Obstacle parameter: %.2f",
                 obstacle_param);
    this->get_parameter("degrees", degrees_param);
    RCLCPP_DEBUG(this->get_logger(), "Degrees parameter: %d", degrees_param);

    // Creatu subscriber
    this->sub_laser = this->create_subscription<LaserScan>(
        "/scan", 1,
        std::bind(&PreApproach::laser_sub_clb, this, std::placeholders::_1));

    // Creatu publisher
    this->pub_twist = this->create_publisher<Twist>("/robot/cmd_vel", 2);
  }

private:
  float obstacle_param;
  int degrees_param;

  Twist twist_msg;

  rclcpp::Subscription<LaserScan>::SharedPtr sub_laser;
  rclcpp::Publisher<Twist>::SharedPtr pub_twist;

  void laser_sub_clb(const LaserScan::SharedPtr msg) {
    static bool turn_needed = true;
    float distance_front = (msg->ranges[539]+ msg->ranges[540] + msg->ranges[541]) / 3;

    // RCLCPP_DEBUG(this->get_logger(), "msg: %.2f", distance_front);
    // RCLCPP_DEBUG(this->get_logger(), "msg cnt: %ld", msg->ranges.size());

    if (distance_front > this->obstacle_param && turn_needed) {
      RCLCPP_DEBUG(this->get_logger(), "msg: %.2f", distance_front);
      this->twist_msg.linear.x = 0.3;
    } else {
      this->twist_msg.linear.x = 0.0;

      if (turn_needed) {
        // Start thread to turn robot
        turn_needed = false;
        std::thread(
            std::bind(&PreApproach::execute_turn, this, std::placeholders::_1),
            this->degrees_param)
            .detach();
      }
    }

    this->pub_twist->publish(this->twist_msg);
  }

  void execute_turn(int degrees) {
    RCLCPP_INFO(this->get_logger(), "Start turning");

    // Make small sleep
    std::this_thread::sleep_for(std::chrono::seconds(1));

    this->twist_msg.angular.z = 0.35 * (degrees / abs(degrees));
    this->pub_twist->publish(this->twist_msg);
    int sleep_time = abs(degrees) * 1000 / 15;

    RCLCPP_INFO(this->get_logger(), "Sleep time: %d seconds,", sleep_time);

    std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));

    this->twist_msg.angular.z = 0.0;
    this->pub_twist->publish(this->twist_msg);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    RCLCPP_INFO(this->get_logger(), "Finish turning");
  }
};

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PreApproach>());
  rclcpp::shutdown();

  return 0;
}