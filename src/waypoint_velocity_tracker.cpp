// waypoint_velocity_tracker.cpp
#include "waypoint_velocity_tracker/waypoint_velocity_tracker.h"

#include <fstream>
#include <sstream>
#include <cmath>
#include <rclcpp/qos.hpp>

using namespace std::chrono_literals;
template <typename T>
T clamp(const T& v, const T& lo, const T& hi)
{
    return std::min(std::max(v, lo), hi);
}

Waypoint_vel_tracker::Waypoint_vel_tracker() : Node("pa_control"),
current_index_(0), offboard_active_(false), has_pose_(false) {
  declare_parameter<std::string>("mission_csv", "mission.csv");
  get_parameter("mission_csv", mission_csv_path_);
  RCLCPP_INFO(this->get_logger(), "Mission CSV path: %s", mission_csv_path_.c_str());

  vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    "/mavros/setpoint_velocity/cmd_vel", rclcpp::SensorDataQoS());

  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/mavros/local_position/pose",
    rclcpp::SensorDataQoS(), 
    std::bind(&Waypoint_vel_tracker::pose_callback, this, std::placeholders::_1));

    rclcpp::QoS state_qos(10);
    state_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
        "/mavros/state", state_qos,
        std::bind(&Waypoint_vel_tracker::state_callback, this, std::placeholders::_1));

  load_waypoints();
  arming_cli_ = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
  timer_ = create_wall_timer(20ms, std::bind(&Waypoint_vel_tracker::control_loop, this));
}

void Waypoint_vel_tracker::load_waypoints() {
  std::ifstream file(mission_csv_path_);
  if (!file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open mission CSV at: %s", mission_csv_path_.c_str());
    rclcpp::shutdown(); 
    return;
  }
  std::string line;
  std::getline(file, line); // skip header
  while (std::getline(file, line)) {
    std::stringstream ss(line);
    std::string token;
    Waypoint wp;
    std::getline(ss, token, ','); wp.x = std::stod(token);
    std::getline(ss, token, ','); wp.y = std::stod(token);
    std::getline(ss, token, ','); wp.z = std::stod(token);
    waypoints_.push_back(wp);
  }
}

void Waypoint_vel_tracker::state_callback(const mavros_msgs::msg::State::SharedPtr msg) {
  current_state_ = *msg;

  if (!offboard_active_ && msg->mode == "OFFBOARD") {
    RCLCPP_INFO(this->get_logger(), "Detected OFFBOARD mode via QGC, arming...");

    for (int i = 0; i < 10; ++i) {
      geometry_msgs::msg::TwistStamped dummy;
      dummy.header.stamp = this->now();
      dummy.twist.linear.x = 0.0;
      dummy.twist.linear.y = 0.0;
      dummy.twist.linear.z = 0.0;
      vel_pub_->publish(dummy);
      rclcpp::sleep_for(10ms); 
    }

    auto arm_req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    arm_req->value = true;

    arming_cli_->async_send_request(arm_req,
      [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture result) {
        if (result.get()->success) {
          RCLCPP_INFO(this->get_logger(), "Drone armed by code.");
          this->offboard_active_ = true;
        } else {
          RCLCPP_ERROR(this->get_logger(), "Arming failed.");
        }
      });
  }
}

void Waypoint_vel_tracker::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  current_pose_ = *msg;
  has_pose_ = true;

  if (!initial_pose_saved_ && offboard_active_) {
    initial_pose_ = *msg;
    initial_pose_saved_ = true;
    RCLCPP_INFO(this->get_logger(), "Initial pose saved: (%.2f, %.2f, %.2f)",
      initial_pose_.pose.position.x,
      initial_pose_.pose.position.y,
      initial_pose_.pose.position.z);
  }
}

void Waypoint_vel_tracker::control_loop() {
  // if (!has_pose_ || current_index_ >= waypoints_.size()) return;

  geometry_msgs::msg::TwistStamped dummy;
  dummy.header.stamp = now();
  dummy.twist.linear.x = 0.0;
  dummy.twist.linear.y = 0.0;
  dummy.twist.linear.z = 0.0;
  if (!offboard_active_) {
    vel_pub_->publish(dummy);
    return;
  }
  if(current_index_ >= waypoints_.size()){
    RCLCPP_INFO(get_logger(), "All waypoints completed.");
      vel_pub_->publish(dummy);
      return;
  }
  Waypoint target = waypoints_[current_index_];

  double target_x = initial_pose_.pose.position.x + target.x;
  double target_y = initial_pose_.pose.position.y + target.y;
  double target_z = initial_pose_.pose.position.z + target.z;

  double dx = target_x - current_pose_.pose.position.x;
  double dy = target_y - current_pose_.pose.position.y;
  double dz = target_z - current_pose_.pose.position.z;

  double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

  if (dist < 0.2) {
    RCLCPP_INFO(get_logger(), "Reached waypoint %ld", current_index_);
    current_index_++;
    vel_pub_->publish(dummy);
    if(current_index_ >= waypoints_.size()) {
      RCLCPP_INFO(get_logger(), "All waypoints completed.");
      vel_pub_->publish(dummy);
    }
    return;
  }

  geometry_msgs::msg::TwistStamped vel;
  vel.header.stamp = now();

  double vx = dx / dist;
  double vy = dy / dist;
  
  double speed = 0.4; // m/s
  vel.twist.linear.x = vx * speed;
  vel.twist.linear.y = vy * speed;
  vel.twist.linear.z = clamp(dz* 0.5, -0.4, 0.4); // vertical speed control

  double dxy = std::sqrt(dx * dx + dy * dy);

if (dxy > 0.3) {
  double desired_yaw = std::atan2(dy, dx);
  tf2::Quaternion q;
  tf2::fromMsg(current_pose_.pose.orientation, q);
  double roll, pitch, current_yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, current_yaw);

  double yaw_err = desired_yaw - current_yaw;
  if (yaw_err > M_PI) yaw_err -= 2 * M_PI;
  if (yaw_err < -M_PI) yaw_err += 2 * M_PI;

  vel.twist.angular.z = clamp(yaw_err, -0.3,0.3); // proportional control
} else {
  vel.twist.angular.z = 0.0;  // yaw control off when only going vertically
}


  vel_pub_->publish(vel);
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Waypoint_vel_tracker>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}