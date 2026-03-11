/*
Code for DT1
V. Sieben
Version 1.0
Date: Feb 4, 2023
License: GNU GPLv3
*/

#include <chrono>
#include <functional>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

class SquareRoutine : public rclcpp::Node
{
public:
  SquareRoutine() : Node("Square_Routine")
  {
    // IMPORTANT for Gazebo/RViz timing:
    // This makes this node use /clock time if it's available.
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));

    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&SquareRoutine::topic_callback, this, _1));

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Faster loop helps reduce late stopping (overshoot)
    timer_ = this->create_wall_timer(50ms, std::bind(&SquareRoutine::timer_callback, this));
  }

private:
  enum class Mode { IDLE, MOVE, ROTATE };

  void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    x_now = msg->pose.pose.position.x;
    y_now = msg->pose.pose.position.y;

    qx = msg->pose.pose.orientation.x;
    qy = msg->pose.pose.orientation.y;
    qz = msg->pose.pose.orientation.z;
    qw = msg->pose.pose.orientation.w;

    odom_ready = true;
  }

  void timer_callback()
  {
    if (!odom_ready) return;

    geometry_msgs::msg::Twist cmd;

    // Quaternion -> yaw
    tf2::Quaternion q(qx, qy, qz, qw);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    a_now = yaw;

    // Distance traveled since last move start
    d_now = std::hypot(x_now - x_init, y_now - y_init);

    if (mode_ == Mode::MOVE)
    {
      if (d_now < d_aim)
      {
        cmd.linear.x = x_vel;
        cmd.angular.z = 0.0;
      }
      else
      {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        last_state_complete = 1;
      }
    }
    else if (mode_ == Mode::ROTATE)
    {
      // Error to absolute goal
      const double err = wrap_angle(a_goal - a_now);

      // --- You asked: "make the check M_PI/72 less than the aim always" ---
      // Base tolerance is pi/72, but if the target angle is smaller than that,
      // we force the tolerance to be slightly smaller than the target so tol < |aim| always.
      const double base_tol = M_PI / 72.0;            // ~2.5 deg
      const double aim_mag  = std::abs(a_aim);
      const double tol = (aim_mag > base_tol) ? base_tol : std::max(0.0, aim_mag - 1e-6);

      // --- Overshoot compensation (you said it overshoots by ~10 degrees consistently) ---
      // Stop early by 10 degrees so the physical robot ends up close to the real goal.
      const double stop_early = overshoot_comp_rad;   // default 10 deg in radians

      if (std::abs(err) > (tol + stop_early))
      {
        cmd.linear.x = 0.0;

        // Constant turn speed at 0.1 rad/s (your request)
        cmd.angular.z = (err > 0.0) ? w_vel : -w_vel;
      }
      else
      {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        last_state_complete = 1;
      }
    }
    else
    {
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
    }

    publisher_->publish(cmd);
    sequence_statemachine();
  }

  void sequence_statemachine()
  {
    if (last_state_complete != 1) return;

    switch (count_)
    {
      case 0:  move_distance(1.38); break;
      case 1:  rotate_angle(M_PI / 2); break;
      case 2:  move_distance(0.47); break;
      case 3:  rotate_angle(3 * M_PI / 2); break;
      case 4:  move_distance(0.9); break;
      case 5:  rotate_angle(3 * M_PI / 2); break;
      case 6:  move_distance(0.47); break;
      case 7:  rotate_angle(M_PI / 2); break;
      case 8:  move_distance(1.0); break;
      case 9:  rotate_angle(M_PI); break;
	  case 10: rclcpp::shutdown(); break;
      default: break;
    }
  }

  void move_distance(double distance)
  {
    mode_ = Mode::MOVE;
    d_aim = distance;

    x_init = x_now;
    y_init = y_now;

    count_++;
    last_state_complete = 0;
  }

  void rotate_angle(double angle)
  {
    mode_ = Mode::ROTATE;
    a_aim = angle;

    a_init = a_now;
    a_goal = wrap_angle(a_init + a_aim);  // absolute target heading (wrapped)

    count_++;
    last_state_complete = 0;
  }

  // Wrap to [-pi, pi]
  double wrap_angle(double angle)
  {
    angle = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (angle < 0.0) angle += 2.0 * M_PI;
    return angle - M_PI;
  }

  // ROS interfaces
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Tuning
  double x_vel = 0.10;        // m/s
  double w_vel = 0.10;        // rad/s (your request)

  // Stop-early compensation (10 degrees)
  const double overshoot_comp_rad = 10.0 * M_PI / 180.0;

  // State
  Mode mode_ = Mode::IDLE;
  bool odom_ready = false;

  // Pose
  double x_now = 0.0, y_now = 0.0;
  double x_init = 0.0, y_init = 0.0;

  double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;

  double a_now = 0.0;
  double a_init = 0.0;
  double a_aim  = 0.0;
  double a_goal = 0.0;

  double d_now = 0.0;
  double d_aim = 0.0;

  size_t count_ = 0;
  int last_state_complete = 1;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SquareRoutine>());
  rclcpp::shutdown();
  return 0;
}