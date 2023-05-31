#include "attach_shelf_interfaces/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/utilities.hpp"
#include "std_msgs/msg/empty.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using attach_shelf_interfaces::srv::GoToLoading;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Twist;
using sensor_msgs::msg::LaserScan;
using std::placeholders::_1;
using std::placeholders::_2;
using std_msgs::msg::Empty;

std::mutex mu1;

class ApproachServerNode : public rclcpp::Node {
public:
  ApproachServerNode() : Node("approach_server_node") {
    RCLCPP_INFO(this->get_logger(), "Server created");

    // Cretae service
    this->srv = this->create_service<GoToLoading>(
        "/approach_shelf",
        std::bind(&ApproachServerNode::srv_clb, this, _1, _2));

    rclcpp::SubscriptionOptions option1;
    this->timer_cb_group =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    option1.callback_group = this->timer_cb_group;

    // Create laser subscriber
    this->sub_laser = this->create_subscription<LaserScan>(
        "/scan", 1, std::bind(&ApproachServerNode::laser_sub_clb, this, _1),
        option1);

    // Create timer
    this->timer = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ApproachServerNode::timer_clb, this), timer_cb_group);
    this->timer->cancel();

    // Create TF broadcaster
    this->tf_broadcaster =
        std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Setup tf listener
    this->tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tf_listener =
        std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer);

    // Creatu publisher
    this->pub_twist = this->create_publisher<Twist>("/robot/cmd_vel", 2);
    this->pub_table_up = this->create_publisher<Empty>("/elevator_up", 2);
  }

private:
  rclcpp::Service<GoToLoading>::SharedPtr srv;
  rclcpp::Subscription<LaserScan>::SharedPtr sub_laser;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::CallbackGroup::SharedPtr timer_cb_group;

  rclcpp::Publisher<Twist>::SharedPtr pub_twist;
  rclcpp::Publisher<Empty>::SharedPtr pub_table_up;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;

  LaserScan::SharedPtr laser_scan_msg;

  // Service callback
  void srv_clb(const GoToLoading::Request::SharedPtr req,
               const GoToLoading::Response::SharedPtr res) {
    RCLCPP_INFO(this->get_logger(), "Service called: %d", req->attach_to_shelf);

    res->complete = false;

    bool status;
    int number_of_legs = 0;
    int laser_index_of_legs[3] = {0};

    status = get_shelf_pos(number_of_legs, laser_index_of_legs);

    if (!status) {
      RCLCPP_INFO(this->get_logger(), "Erro in searching for legs");
      return;
      this->timer->cancel();
    }

    RCLCPP_INFO(this->get_logger(), "Total number of legs: %d", number_of_legs);

    if (number_of_legs != 2) {
      res->complete = false;
      this->timer->cancel();
      return;
    }

    // Publish TF by starting timer
    this->timer->reset();

    if (!req->attach_to_shelf) {
      // return Sucess
      res->complete = true;
      return;
    }

    // Sleep a bit for starting TF publisher
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Compute TF and and move robot under table

    geometry_msgs::msg::TransformStamped t;

    std::string reference_frame = "robot_base_link";
    std::string target_frame = "cart_frame";

    float err_threshold = 0.15;

    static const float kp_yaw = 0.5;
    static const float kp_distance = 0.2;

    float error_yaw = 0, error_distance = 99.0;

    while (error_distance > err_threshold) {

      try {
        t = this->tf_buffer->lookupTransform(reference_frame, target_frame,
                                             tf2::TimePointZero);
      } catch (const tf2::TransformException &ex) {
        RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
                    reference_frame.c_str(), target_frame.c_str(), ex.what());
        res->complete = false;
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Dist: %.2f, %.2f",
                  t.transform.translation.x, t.transform.translation.y);

      // Calculate distance and angular err
      error_yaw = atan2(t.transform.translation.y, t.transform.translation.x);
      error_distance = sqrt(pow(t.transform.translation.x, 2) +
                            pow(t.transform.translation.y, 2));

      Twist msg;
      msg.angular.z = kp_yaw * error_yaw;
      msg.linear.x = kp_distance * error_distance;
      msg.linear.x += (0.15 * abs(msg.angular.x));

      this->pub_twist->publish(msg);

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    Twist msg;
    msg.angular.z = 0.0;
    msg.linear.x = 0.0;
    this->pub_twist->publish(msg);

    RCLCPP_INFO(this->get_logger(), "Finish Dist: %.2f, %.2f",
                t.transform.translation.x, t.transform.translation.y);

    this->timer->cancel(); // Stop publishing TF


    msg.linear.x = 0.3;
    msg.angular.z = 0.0;
    this->pub_twist->publish(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(2500));
    msg.linear.x = 0.0;
    msg.angular.z = 0.0;
    this->pub_twist->publish(msg);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    RCLCPP_INFO(this->get_logger(), "Lifting table");
    Empty empty_msg;
    this->pub_table_up->publish(empty_msg);

    res->complete = true;

  } // END: Service callback

  // Laser clb
  void laser_sub_clb(const LaserScan::SharedPtr msg) {
    mu1.lock();
    this->laser_scan_msg = msg;
    mu1.unlock();
  } // END: Laser clb

  // Timer clb -> Used for publish tf frame
  void timer_clb() {
    bool status;
    int number_of_legs = 0;
    int laser_index_of_legs[3] = {0};

    try {
      // Get table pos
      status = get_shelf_pos(number_of_legs, laser_index_of_legs);
    } catch (const std::overflow_error &e) {
      RCLCPP_ERROR(this->get_logger(), "Overflow");
      return;
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "Another error");
      return;
    }

    if (!status) {
      RCLCPP_ERROR(this->get_logger(), "Erro in searching for legs");
      return;
    }

    if (laser_index_of_legs[0] >
            ((int)this->laser_scan_msg->ranges.size() - 1) ||
        laser_index_of_legs[1] >
            ((int)this->laser_scan_msg->ranges.size() - 1)) {
      RCLCPP_ERROR(this->get_logger(), "Error in laser_index_of_legs");
      return;
    }

    // RCLCPP_INFO(this->get_logger(),
    //             "Total number of legs: %d, idx1: %d, idx2: %d",
    //             number_of_legs, laser_index_of_legs[0],
    //             laser_index_of_legs[1]);

    if (number_of_legs != 2) {
      return;
    }
    // Publish TF frame "cart_frame"

    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    // t.header.stamp = this->laser_scan_msg->header.stamp;
    t.header.frame_id = "robot_front_laser_base_link";
    t.child_frame_id = "cart_frame";

    // Set coordinates

    float angle1, angle2, radius1, radius2;
    try {
      angle1 = -135 + laser_index_of_legs[0] * 0.25;
      angle2 = -135 + laser_index_of_legs[1] * 0.25;
      radius1 = this->laser_scan_msg->ranges[laser_index_of_legs[0]];
      radius2 = this->laser_scan_msg->ranges[laser_index_of_legs[1]];
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "Another error, set coord");
    }

    Point p1, p2;

    p1.x = radius1 * cos(angle1 * 3.1416 / 180);
    p1.y = radius1 * sin(angle1 * 3.1416 / 180);
    p2.x = radius2 * cos(angle2 * 3.1416 / 180);
    p2.y = radius2 * sin(angle2 * 3.1416 / 180);

    // Convert to cartesian
    t.transform.translation.x = (p1.x + p2.x) / 2;
    t.transform.translation.y = (p1.y + p2.y) / 2;
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, 0);

    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    // Send transformation

    this->tf_broadcaster->sendTransform(t);
  } // END: Timer clb

  bool get_shelf_pos(int &number_of_legs, int leg_index[]) {

    mu1.lock();

    // Count number of legs
    number_of_legs = 0;
    int leg_len = 0;

    float intensity_threshold = 1000.0;

    for (auto it = this->laser_scan_msg->intensities.begin();
         it != this->laser_scan_msg->intensities.end(); it++) {

      if (*it > intensity_threshold) {
        leg_len++;
      } else if (leg_len > 1) {
        leg_index[number_of_legs] =
            it - this->laser_scan_msg->intensities.begin() - 1 - (leg_len / 2);
        // RCLCPP_INFO(this->get_logger(), "Leg %d created at pos %d",
        //             number_of_legs, leg_index[number_of_legs]);
        number_of_legs++;
        leg_len = 0;
      } else {
        leg_len = 0;
      }

      if (number_of_legs > 2) {
        RCLCPP_ERROR(this->get_logger(), "Too many legs:");

        mu1.unlock();
        return false;
      }
    }

    mu1.unlock();
    return true;
  }
};

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  std::shared_ptr<ApproachServerNode> node =
      std::make_shared<ApproachServerNode>();

  rclcpp::executors::MultiThreadedExecutor exe;
  exe.add_node(node);
  exe.spin();
  rclcpp::shutdown();

  return 0;
}