#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <mavros_msgs/msg/state.hpp>

#include <vector>
#include <string>

struct Waypoint {
  double x, y, z;
};

class Waypoint_vel_tracker : public rclcpp::Node {
public:
    Waypoint_vel_tracker();

private:
  void load_waypoints();
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void control_loop();
  void state_callback(const mavros_msgs::msg::State::SharedPtr msg); 

  std::string mission_csv_path_;
  std::vector<Waypoint> waypoints_;
  geometry_msgs::msg::PoseStamped current_pose_;
  bool has_pose_;
  bool offboard_active_;
  size_t current_index_;
  mavros_msgs::msg::State current_state_;

  geometry_msgs::msg::PoseStamped initial_pose_;
  bool initial_pose_saved_ = false;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_cli_;

  rclcpp::TimerBase::SharedPtr timer_;
};
