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

class AlignParallelParking : public rclcpp::Node
{
public:
  AlignParallelParking() : Node("align_parallel_parking")
  {
    // If you are on the real robot, set false; if sim, set true.
    this->declare_parameter<bool>("use_sim_time", false);
    this->set_parameter(rclcpp::Parameter("use_sim_time", this->get_parameter("use_sim_time").as_bool()));

    // Params (tune later)
    this->declare_parameter<int>("samples", 10);
    this->declare_parameter<double>("tol_mm", 15.0);          // near-equal threshold
    this->declare_parameter<double>("x_vel", 0.10);           // m/s
    this->declare_parameter<double>("w_vel", 0.10);           // rad/s
    this->declare_parameter<double>("back_m", 0.10);
    this->declare_parameter<double>("fwd_m", 0.10);
    this->declare_parameter<double>("turn_rad", 0.25);        // ~14 deg
    this->declare_parameter<int>("max_cycles", 50);

    samples_   = this->get_parameter("samples").as_int();
    tol_mm_    = this->get_parameter("tol_mm").as_double();
    x_vel_     = this->get_parameter("x_vel").as_double();
    w_vel_     = this->get_parameter("w_vel").as_double();
    back_m_    = this->get_parameter("back_m").as_double();
    fwd_m_     = this->get_parameter("fwd_m").as_double();
    turn_rad_  = this->get_parameter("turn_rad").as_double();
    max_cycles_= this->get_parameter("max_cycles").as_int();

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&AlignParallelParking::odom_cb, this, _1));

    left_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/tof_left_mm", 10, std::bind(&AlignParallelParking::left_cb, this, _1));

    right_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/tof_right_mm", 10, std::bind(&AlignParallelParking::right_cb, this, _1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    timer_ = this->create_wall_timer(50ms, std::bind(&AlignParallelParking::tick, this));

    RCLCPP_INFO(this->get_logger(), "align_parallel_parking started. Waiting for odom + ToF...");
  }

private:
  enum class Mode { IDLE, SAMPLE, MOVE, ROTATE, DONE };

  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    x_now_ = msg->pose.pose.position.x;
    y_now_ = msg->pose.pose.position.y;

    const auto &q = msg->pose.pose.orientation;
    tf2::Quaternion qt(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 m(qt);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    a_now_ = yaw;

    odom_ready_ = true;
  }

  void left_cb(const std_msgs::msg::Float32::SharedPtr msg)
  {
    left_mm_now_ = msg->data;
    left_ready_ = true;
  }

  void right_cb(const std_msgs::msg::Float32::SharedPtr msg)
  {
    right_mm_now_ = msg->data;
    right_ready_ = true;
  }

  static double wrap_angle(double a)
  {
    a = std::fmod(a + M_PI, 2.0 * M_PI);
    if (a < 0.0) a += 2.0 * M_PI;
    return a - M_PI;
  }

  void publish_stop()
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    cmd_pub_->publish(cmd);
  }

  void start_move(double distance_m)
  {
    mode_ = Mode::MOVE;
    d_aim_ = std::abs(distance_m);
    move_sign_ = (distance_m >= 0.0) ? 1.0 : -1.0;
    x_init_ = x_now_;
    y_init_ = y_now_;
    state_done_ = false;
  }

  void start_rotate(double angle_rad)
  {
    mode_ = Mode::ROTATE;
    a_aim_ = angle_rad;
    a_init_ = a_now_;
    a_goal_ = wrap_angle(a_init_ + a_aim_);
    state_done_ = false;
  }

  void tick()
  {
    if (!odom_ready_ || !left_ready_ || !right_ready_) {
      publish_stop();
      return;
    }

    geometry_msgs::msg::Twist cmd;

    // distance traveled since move start
    double d_now = std::hypot(x_now_ - x_init_, y_now_ - y_init_);

    if (mode_ == Mode::MOVE)
    {
      if (d_now < d_aim_)
      {
        cmd.linear.x = move_sign_ * x_vel_;
        cmd.angular.z = 0.0;
      }
      else
      {
        publish_stop();
        state_done_ = true;
      }
    }
    else if (mode_ == Mode::ROTATE)
    {
      const double err = wrap_angle(a_goal_ - a_now_);
      const double tol = M_PI / 90.0; // ~2 deg

      if (std::abs(err) > tol)
      {
        cmd.linear.x = 0.0;
        cmd.angular.z = (err > 0.0) ? w_vel_ : -w_vel_;
      }
      else
      {
        publish_stop();
        state_done_ = true;
      }
    }
    else
    {
      // SAMPLE / IDLE / DONE: default stop
      publish_stop();
    }

    // publish cmd if MOVE/ROTATE
    if (mode_ == Mode::MOVE || mode_ == Mode::ROTATE)
      cmd_pub_->publish(cmd);

    // advance state machine
    sequence();
  }

  void sequence()
  {
    if (mode_ == Mode::DONE) return;

    // SAMPLE state: robot must be stopped; collect N samples
    if (mode_ == Mode::SAMPLE || mode_ == Mode::IDLE)
    {
      publish_stop();

      // collect samples
      left_buf_.push_back(left_mm_now_);
      right_buf_.push_back(right_mm_now_);

      if ((int)left_buf_.size() < samples_) {
        mode_ = Mode::SAMPLE;
        return;
      }

      // compute average
      auto avg = [](const std::vector<float>& v){
        double s = 0.0;
        for (auto x : v) s += x;
        return s / (double)v.size();
      };

      const double L = avg(left_buf_);
      const double R = avg(right_buf_);
      left_buf_.clear();
      right_buf_.clear();

      const double diff = L - R;

      RCLCPP_INFO(this->get_logger(), "SAMPLE: L=%.1f mm R=%.1f mm diff=%.1f mm", L, R, diff);

      if (std::abs(diff) <= tol_mm_) {
        RCLCPP_INFO(this->get_logger(), "Aligned within tol=%.1f mm. Stopping.", tol_mm_);
        publish_stop();
        mode_ = Mode::DONE;
        rclcpp::shutdown();
        return;
      }

      if (cycle_count_++ >= max_cycles_) {
        RCLCPP_WARN(this->get_logger(), "Max cycles reached. Stopping for safety.");
        publish_stop();
        mode_ = Mode::DONE;
        rclcpp::shutdown();
        return;
      }

      // decide turn direction
      // diff>0 => left farther => rotate left; diff<0 => rotate right
      turn_dir_ = (diff > 0.0) ? +1.0 : -1.0;

      // start adjustment sequence
      step_ = 0;
      start_move(-back_m_);
      return;
    }

    // MOVE/ROTATE complete => advance steps
    if (!state_done_) return;

    // sequence: back -> rotate1 -> forward -> rotate2 -> sample
    if (step_ == 0) { step_ = 1; start_rotate(turn_dir_ * turn_rad_); return; }
    if (step_ == 1) { step_ = 2; start_move(+fwd_m_); return; }
    if (step_ == 2) { step_ = 3; start_rotate(-turn_dir_ * turn_rad_); return; }

    // done one adjustment cycle, go sample again
    step_ = 0;
    mode_ = Mode::SAMPLE;
  }

  // ROS interfaces
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr left_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr right_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // readiness
  bool odom_ready_{false}, left_ready_{false}, right_ready_{false};

  // odom state
  double x_now_{0}, y_now_{0}, a_now_{0};
  double x_init_{0}, y_init_{0}, a_init_{0};
  double a_goal_{0};

  // motion targets
  double d_aim_{0};
  double a_aim_{0};
  double move_sign_{1.0};

  // sensor state
  float left_mm_now_{0.0f}, right_mm_now_{0.0f};
  std::vector<float> left_buf_, right_buf_;

  // state machine
  Mode mode_{Mode::IDLE};
  bool state_done_{true};
  int step_{0};
  int cycle_count_{0};
  double turn_dir_{+1.0};

  // params
  int samples_{10};
  double tol_mm_{15.0};
  double x_vel_{0.10};
  double w_vel_{0.10};
  double back_m_{0.10};
  double fwd_m_{0.10};
  double turn_rad_{0.25};
  int max_cycles_{50};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AlignParallelParking>());
  rclcpp::shutdown();
  return 0;
}
