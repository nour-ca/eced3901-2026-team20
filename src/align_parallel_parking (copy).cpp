#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

class AlignParallelParking : public rclcpp::Node
{
public:
  AlignParallelParking() : Node("align_parallel_parking")
  {
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&AlignParallelParking::odom_cb, this, _1));

    left_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/tof_left_mm", 10, std::bind(&AlignParallelParking::left_cb, this, _1));

    right_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/tof_right_mm", 10, std::bind(&AlignParallelParking::right_cb, this, _1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    timer_ = this->create_wall_timer(100ms, std::bind(&AlignParallelParking::tick, this));

    RCLCPP_INFO(this->get_logger(), "Parallel alignment node started");
  }

private:

  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    auto q = msg->pose.pose.orientation;
    tf2::Quaternion qt(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 m(qt);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    a_now_ = yaw;
    odom_ready_ = true;
  }

  void left_cb(const std_msgs::msg::Float32::SharedPtr msg)
  {
    left_buf_.push_back(msg->data);

    if (left_buf_.size() > avg_samples_)
      left_buf_.erase(left_buf_.begin());

    left_ready_ = true;
  }

  void right_cb(const std_msgs::msg::Float32::SharedPtr msg)
  {
    right_buf_.push_back(msg->data);

    if (right_buf_.size() > avg_samples_)
      right_buf_.erase(right_buf_.begin());

    right_ready_ = true;
  }

  float avg(const std::vector<float>& v)
  {
    float s = 0;
    for (auto x : v) s += x;
    return s / v.size();
  }

  void stop_robot()
  {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = 0;
    msg.angular.z = 0;
    cmd_pub_->publish(msg);
  }

  void tick()
  {
    if (!odom_ready_ || !left_ready_ || !right_ready_) {
      stop_robot();
      return;
    }

    if (left_buf_.size() < avg_samples_ || right_buf_.size() < avg_samples_) {
      stop_robot();
      return;
    }

    float L = avg(left_buf_);
    float R = avg(right_buf_);

    float diff = L - R;

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      500,
      "L_avg=%.1f R_avg=%.1f diff=%.1f",
      L, R, diff
    );

    if (std::abs(diff) <= tol_mm_) {
      stop_robot();
      RCLCPP_INFO(this->get_logger(), "Aligned within %f mm", tol_mm_);
      rclcpp::shutdown();
      return;
    }

    geometry_msgs::msg::Twist msg;

    if (diff > 0) {
      msg.angular.z = -w_vel_;
    } else {
      msg.angular.z = w_vel_;
    }

    msg.linear.x = 0;
    cmd_pub_->publish(msg);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr left_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr right_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool odom_ready_{false};
  bool left_ready_{false};
  bool right_ready_{false};

  double a_now_{0};

  std::vector<float> left_buf_;
  std::vector<float> right_buf_;

  int avg_samples_{5};
  double tol_mm_{3.0};
  double w_vel_{0.07};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AlignParallelParking>());
  rclcpp::shutdown();
  return 0;
}
