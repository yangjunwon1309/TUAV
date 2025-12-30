
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <vector>
#include <string>

#include <deque>
#include <Eigen/Dense>
#include <memory>


struct Waypoint {
    double x, y, z,yaw_deg;
  };

class AprilTagLandVel : public rclcpp::Node {
public:
    AprilTagLandVel();

    //added
    void addTagPosition(const Eigen::Vector3d& pos);
    Eigen::Vector3d getSmoothedTagPosition() const;

private:

    // added
    std::deque<Eigen::Vector3d> tag_position_window_;
    const size_t window_size_ = 5;  // adjust size as needed


    enum class State { Arming,Takeoff, Search, Approach, Descend, Finished };
    State state_;
    void switchToState(State new_state);
    std::string mission_csv_path_;
    std::vector<Waypoint> waypoints_;
    size_t current_index_;
    void load_waypoints();
    geometry_msgs::msg::PoseStamped initial_pose_;
    bool initial_pose_saved_ = false;
    bool offboard_active_;

    void tagCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg);
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void publishVelocitySetpoint(double vx, double vy, double vz, double yaw = 0.0);
    void state_callback(const mavros_msgs::msg::State::SharedPtr msg); 
    void timerCallback();
    void tryArmingViaMavros();
   
    void handleTakeoff();
    void handleSearch();
    void handleApproach();
    void handleDescend();

    bool tagDetectedRecently();
    bool positionReachedXY(const Eigen::Vector3d &target);
    Eigen::Vector3d computeVelocityCommand(const Eigen::Vector3d &target);

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_cli_;
    mavros_msgs::msg::State current_state_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_cli_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tag_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;

    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr tag_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_position_sub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    uint8_t last_nav_state_ = 0;
    int offboard_setpoint_counter_ = 0;
    int arming_attempt_count_ = 0;
    bool armed_ = false;

    bool takeoff_origin_set_ = false;
    Eigen::Vector3d takeoff_origin_;

    std::shared_ptr<rclcpp::Clock> clock_;
    rclcpp::Time last_tag_time_;
    Eigen::Vector3d current_position_;
    Eigen::Quaterniond current_orientation_;
    Eigen::Vector3d current_velocity_;
    Eigen::Vector3d latest_tag_position_;
    bool tag_detected_ = false;

    double p_gain_, max_vel_, descent_vel_, pos_thresh_, timeout_sec_, yaw_gain_;
    double takeoff_height_, takeoff_thresh_;
    double search_step_;
    Eigen::Vector3d search_origin_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    double tag_x_, tag_y_;
    Eigen::Vector3d landing_origin_;    



    // Flag to indicate if descend has been initialized (first control loop iteration)
    bool descend_initialized_ = false;

    // Proportional gains for PD controller (x, y, z)
    Eigen::Vector3d Kp_{1.0, 1.0, 2.0};  // Tune these gains as needed

    // Flag to indicate if final slow descent phase has started
    bool final_descend_started_ = false;

    // Last recorded position (for velocity estimation)
    Eigen::Vector3d last_position_{0.0, 0.0, 0.0};

    // Last recorded time (for velocity estimation)
    rclcpp::Time last_time_;

    // Derivative gains for PD controller (x, y, z)
    Eigen::Vector3d Kd_{0.1, 0.1, 0.1};  // Tune these gains as needed

    // Flag to indicate if landing origin (target position) has been set after detecting tag
    bool landing_origin_initialized_ = false;


    // auto landing by px4
    rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr command_long_client_;   
    

};