#include <chrono>
#include <cmath>
#include <memory>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

class AlignParallelParking2 : public rclcpp::Node
{
public:
  AlignParallelParking2() : Node("align_parallel_parking2")
  {
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&AlignParallelParking2::odom_cb, this, _1));

    left_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/tof_left_mm", 10, std::bind(&AlignParallelParking2::left_cb, this, _1));

    right_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/tof_right_mm", 10, std::bind(&AlignParallelParking2::right_cb, this, _1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    timer_ = this->create_wall_timer(50ms, std::bind(&AlignParallelParking2::tick, this));

    RCLCPP_INFO(this->get_logger(), "align_parallel_parking2 custom-method controller started");
  }

  ~AlignParallelParking2()
  {
    stop_robot();
  }

private:
  enum class Mode
  {
    SAMPLE,
    TURN_90,
    MOVE_SIDE,
    TURN_BACK_90,
    TURN_30_TO_LOW,
    FINAL_FORWARD,
    DONE
  };

  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    x_now_ = msg->pose.pose.position.x;
    y_now_ = msg->pose.pose.position.y;

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
    if (msg->data > 0.0f) {
      push_sample(left_buf_, msg->data);
      left_ready_ = true;
    }
  }

  void right_cb(const std_msgs::msg::Float32::SharedPtr msg)
  {
    if (msg->data > 0.0f) {
      push_sample(right_buf_, msg->data);
      right_ready_ = true;
    }
  }

  void push_sample(std::vector<float> & buf, float value)
  {
    buf.push_back(value);
    if (buf.size() > static_cast<size_t>(avg_samples_)) {
      buf.erase(buf.begin());
    }
  }

  double average(const std::vector<float> & buf) const
  {
    double sum = 0.0;
    for (float x : buf) {
      sum += x;
    }
    return sum / static_cast<double>(buf.size());
  }

  double wrap_angle(double angle) const
  {
    angle = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (angle < 0.0) {
      angle += 2.0 * M_PI;
    }
    return angle - M_PI;
  }

  double move_distance_done() const
  {
    return std::hypot(x_now_ - x_start_, y_now_ - y_start_);
  }

  void stop_robot()
  {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = 0.0;
    msg.angular.z = 0.0;
    cmd_pub_->publish(msg);
  }

  void tick()
  {
    if (!odom_ready_ || !left_ready_ || !right_ready_) {
      stop_robot();
      return;
    }

    if (left_buf_.size() < static_cast<size_t>(avg_samples_) ||
        right_buf_.size() < static_cast<size_t>(avg_samples_)) {
      stop_robot();
      return;
    }

    switch (mode_) {
      case Mode::SAMPLE:
        do_sample();
        break;

      case Mode::TURN_90:
      case Mode::TURN_BACK_90:
      case Mode::TURN_30_TO_LOW:
        do_rotate();
        break;

      case Mode::MOVE_SIDE:
      case Mode::FINAL_FORWARD:
        do_move();
        break;

      case Mode::DONE:
        stop_robot();
        break;
    }
  }

  void do_sample()
{
  stop_robot();

  const double L = average(left_buf_);
  const double R = average(right_buf_);
  const double farther = std::max(L, R);
  const double smaller = std::min(L, R);
  const double delta = farther - smaller;
  const double mid = delta / 2.0;

  RCLCPP_INFO(
    this->get_logger(),
    "Left = %.1f mm | Right = %.1f mm | Difference = %.1f mm | Mid = %.1f mm",
    L, R, delta, mid);

  if (L < min_safe_mm_ || R < min_safe_mm_) {
    mode_ = Mode::DONE;
    stop_robot();
    RCLCPP_WARN(
      this->get_logger(),
      "Too close to cargo. Stopping. Left = %.1f mm | Right = %.1f mm | Difference = %.1f mm",
      L, R, delta);
    return;
  }

  if (delta <= tol_mm_) {
    mode_ = Mode::DONE;
    stop_robot();
    RCLCPP_INFO(
      this->get_logger(),
      "Within %.1f mm. Stopping. Left = %.1f mm | Right = %.1f mm | Difference = %.1f mm",
      tol_mm_, L, R, delta);
    return;
  }

  // Your method:
  // move_dist = sin(45 deg) * smaller
  move_dist_m_ = ((std::sin(M_PI / 4.0) * smaller) + 15) / 1000.0;

  // Clamp for sanity
  if (move_dist_m_ < min_move_m_) {
    move_dist_m_ = min_move_m_;
  }
  if (move_dist_m_ > max_move_m_) {
    move_dist_m_ = max_move_m_;
  }

  right_is_higher_ = (R > L);

  RCLCPP_INFO(
    this->get_logger(),
    "Computed move_dist = %.3f m | right_is_higher = %s",
    move_dist_m_,
    right_is_higher_ ? "true" : "false");

  // Turn 90 degrees toward higher side
  // If this is flipped on your robot, swap the signs below
  double turn_90 = right_is_higher_ ? -M_PI_2 : +M_PI_2;
  rot_target_ = wrap_angle(a_now_ + turn_90);
  mode_ = Mode::TURN_90;
}
  void do_rotate()
  {
    const double err = wrap_angle(rot_target_ - a_now_);

    if (std::abs(err) <= rot_tol_rad_) {
      stop_robot();

      if (mode_ == Mode::TURN_90) {
        x_start_ = x_now_;
        y_start_ = y_now_;
        move_target_m_ = move_dist_m_;
        move_sign_ = -1.0;   // flipped so it moves the correct way after the 90° turn
        mode_ = Mode::MOVE_SIDE;
      }
      else if (mode_ == Mode::TURN_BACK_90) {
        // Turn 30 degrees toward lower side
        double turn_30 = right_is_higher_ ? +M_PI / 6.3 : -M_PI / 6.3;
        rot_target_ = wrap_angle(a_now_ + turn_30);
        mode_ = Mode::TURN_30_TO_LOW;
      }
      else if (mode_ == Mode::TURN_30_TO_LOW) {
        // Move forward 10 mm before taking the next reading
        x_start_ = x_now_;
        y_start_ = y_now_;
        move_target_m_ = 0.12;   // 10 mm
        move_sign_ = -1.0;
        mode_ = Mode::FINAL_FORWARD;
      }

      return;
    }

    geometry_msgs::msg::Twist msg;
    msg.linear.x = 0.0;
    msg.angular.z = (err > 0.0) ? w_vel_ : -w_vel_;
    cmd_pub_->publish(msg);
  }

  void do_move()
{
  const double L = average(left_buf_);
  const double R = average(right_buf_);
  const double delta = std::abs(R - L);

  if (L < min_safe_mm_ || R < min_safe_mm_) {
    mode_ = Mode::DONE;
    stop_robot();
    RCLCPP_WARN(
      this->get_logger(),
      "Too close during move. Stopping. Left = %.1f mm | Right = %.1f mm | Difference = %.1f mm",
      L, R, delta);
    return;
  }

  const double d = move_distance_done();

  if (d >= move_target_m_) {
    stop_robot();

    if (mode_ == Mode::MOVE_SIDE) {
      // Turn back 90 to original face
      double turn_back_90 = right_is_higher_ ? +M_PI_2 : -M_PI_2;
      rot_target_ = wrap_angle(a_now_ + turn_back_90);
      mode_ = Mode::TURN_BACK_90;
    }
    else if (mode_ == Mode::FINAL_FORWARD) {
      mode_ = Mode::SAMPLE;
    }

    return;
  }

  geometry_msgs::msg::Twist msg;
  msg.linear.x = move_sign_ * x_vel_;
  msg.angular.z = 0.0;
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
  bool right_is_higher_{true};

  double x_now_{0.0};
  double y_now_{0.0};
  double a_now_{0.0};

  double x_start_{0.0};
  double y_start_{0.0};
  double rot_target_{0.0};

  double move_target_m_{0.0};
  double move_dist_m_{0.0};
  double move_sign_{1.0};

  std::vector<float> left_buf_;
  std::vector<float> right_buf_;

  Mode mode_{Mode::SAMPLE};

  int avg_samples_{5};
  double tol_mm_{10.0};
  double min_safe_mm_{70.0};

  double min_move_m_{0.02};
  double max_move_m_{0.25};

  double rot_tol_rad_{0.02};
  double x_vel_{0.09};
  double w_vel_{0.15};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AlignParallelParking2>());
  rclcpp::shutdown();
  return 0;
}
