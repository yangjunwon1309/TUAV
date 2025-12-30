
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <vector>
#include <string>
#include <mavros_msgs/srv/command_long.hpp>
//added
#include <deque>
#include <Eigen/Dense>
#include <memory>

//added for changed modecallback
#include <std_msgs/msg/int32.hpp>

//added for updating attitude level
#include <mavros_msgs/msg/attitude_target.hpp>
#include <std_msgs/msg/float64.hpp>

//added for ext_force
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/imu.hpp>


struct Waypoint {
    double x, y, z,yaw_deg;
  };

class PiTagLandVel : public rclcpp::Node {
public:
    PiTagLandVel();

    //added
    void addTagPosition(const Eigen::Vector3d& pos, const geometry_msgs::msg::Quaternion& orientation);
    Eigen::Quaterniond getSmoothedTagOrientation() const;
    Eigen::Vector3d getSmoothedTagPosition() const;
    Eigen::Vector3d getDummyTagPosition();

private:

	// ======= IMU & direction for external force ======= //
	
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    Eigen::Vector3d imu_accel_body_{Eigen::Vector3d::Zero()};
    bool has_imu_accel_{false};

    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr ext_force_dir_pub_;

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);	


    // added
    std::deque<Eigen::Vector3d> tag_position_window_;
    std::deque<Eigen::Quaterniond> tag_orientation_window_;
    
    const size_t window_size_ = 5;  // adjust size as needed

    enum class State { Arming,Takeoff, Search, Approach, Descend, Finished };
    State state_;
    void switchToState(State new_state);
    std::string mission_csv_path_;
    std::vector<Waypoint> waypoints_;
    size_t current_index_;
    void load_waypoints();
    geometry_msgs::msg::PoseStamped initial_pose_;
    double initial_yaw_;
    bool initial_pose_saved_ = false;
    bool offboard_active_;

    void tagCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg);
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void publishVelocitySetpoint(double vx, double vy, double vz, double yaw = 0.0);
    
    // atttidue-level publish
	void publishAttitudeTargetFromAccel(const Eigen::Vector3d& a_cmd,
                                      double desired_yaw,
                                      double hover_thrust);    
	  // v_des -> a_cmd (ASMC 3D) (NEW)
	Eigen::Vector3d asmc3d_from_vdes(const Eigen::Vector3d& target_pos,
		                               const Eigen::Vector3d& v_des);                                      
                                                                            
    
    void state_callback(const mavros_msgs::msg::State::SharedPtr msg); 
    void timerCallback();
    void tryArmingViaMavros();
    // Add the function for vel_callback
    void vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);    
    // Add the function for mode_callback
    void modeCallback(const std_msgs::msg::Int32::SharedPtr msg);
    
    // add the function for disarm
    void requestDisarm();
    
    
    // setup for vel
    geometry_msgs::msg::TwistStamped current_vel_; 
    bool has_vel_ = false;
    
    // previous    
    double dt_{0.02};
    
    // changed timer (0.01 or 0.008 etc..)
    //double dt_{0.01}; 
    
    //ASMC parameter for search 
    double lambda_search_{0.3},  gamma_search_{1.0},  kmin_search_{0.03}, kmax_search_{1.0}, phi_search_{0.6};
    Eigen::Vector2d k_xy_search_{0.1, 0.1};  
    
    //ASMC parameter for approach
    // prev
    //double lambda_approach_{0.65};
    // try
    double lambda_approach_{1.0};
    double gamma_approach_{0.8},  kmin_approach_{0.03}, kmax_approach_{1.2}, phi_approach_{0.8};
    Eigen::Vector2d k_xy_approach_{0.1, 0.1};      
    
    //ASMC parameter for descend
    double lambda_descend_{1.0};
    double gamma_descend_{1.0},  kmin_descend_{0.03}, kmax_descend_{1.2}, phi_descend_{1.0};
    Eigen::Vector2d k_xy_descend_{0.1, 0.1};
    
    //Add for outlier
    double initial_sigma_{0.15};
    double outlier_threshold_sigma_{2.5};    
    
    // dummy tag position parameters
    double accumulated_time_ = 0.0; // total t
    //int dummy_mode_ = 0;            // 0: sin oscilliation, 1: constant v oscilliation, 2: circle
    
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
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_, drone_pose_from_tag_pub_, drone_start_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;

    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr tag_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_position_sub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_dummy_pos_;

	// publisher for attitude
	rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr att_pub_; 
	
	// check cos tilt from throttle
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cos_tilt_pub_;

    // subscribe current velocity
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;

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
    geometry_msgs::msg::Pose latest_tag_pose_map_;  // position + orientation in map frame
    geometry_msgs::msg::Pose latest_robot_pose_map_;  // robot position + orientation in map frame
    bool tag_detected_ = false;
    int current_mode_ = 0;  // Current mode from /decon_uav/mode topic
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr latest_tag_position_map_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr robot_position_map_pub_;

    double p_gain_, max_vel_, descent_vel_, pos_thresh_, timeout_sec_, yaw_gain_;
    double takeoff_height_, takeoff_thresh_;
    double search_step_;
    Eigen::Vector3d search_origin_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    double tag_x_april_, tag_y_april_;



    // Flag to indicate if descend has been initialized (first control loop iteration)
    bool descend_initialized_ = false;

    // Proportional gains for PD controller (x, y, z)
    Eigen::Vector3d Kp_{1.0, 1.0, 2.0};  // Tune these gains as needed
    
    // Proportional gains for descending
    Eigen::Vector3d Kp_descend_{1.5, 1.5, 2.0};  

    // Last recorded position (for velocity estimation)
    Eigen::Vector3d last_position_{0.0, 0.0, 0.0};

    // Last recorded time (for velocity estimation)
    rclcpp::Time last_time_;

    // Derivative gains for PD controller (x, y, z)
    Eigen::Vector3d Kd_{0.1, 0.1, 0.1};  // Tune these gains as needed

    // Flag to indicate if landing origin (target position) has been set after detecting tag
    bool landing_origin_initialized_ = false;
    
    // for geometric smoothing
    Eigen::Vector3d ref_vel_smooth_ = Eigen::Vector3d::Zero();

    // compensation for vxy
//	Eigen::Vector2d asmc_xy_(
//		const Eigen::Vector2d& v_ref,
//		const Eigen::Vector2d& p_err,
//		double lambda, double k_min, double k_max, double gamma, double phi,
//		Eigen::Vector2d& k_adapt);

	Eigen::Vector2d asmc_xy_(
		const Eigen::Vector2d& p_err,
		double lambda, double k_min, double k_max, double gamma, double phi,
		Eigen::Vector2d& k_adapt);


	// Setup for attitude level

	// nominal condition
    double hover_thrust_{0.738};
    // with tether
    //double hover_thrust_{0.78};
    
    double asmc_L1_{1.0};
    double asmc_gain_{1.0};
    double asmc_max_accel_{0.3};
    Eigen::Vector3d asmc_phi_{0.3, 0.3, 0.3};
    Eigen::Vector3d asmc_C_{1.0, 1.0, 1.0}; // s = e_v + C*e_p
    Eigen::Vector3d k_adapt_3d_{0.1, 0.1, 0.1};
    Eigen::Vector3d k_min_3d_{0.03, 0.03, 0.03};
    Eigen::Vector3d k_max_3d_{1.2, 1.2, 1.2};
    Eigen::Vector3d d_adapt_3d_{0.2, 0.2, 0.2};
    
    // Setup for setpoint initialization
    bool have_init_desired_{false};
    
    // setup for descending
    double z_ref_{0.0};
    bool z_ref_initialized_{false};
    
    // added for accel calculation (acc_ref)
	Eigen::Vector3d prev_v_des_{Eigen::Vector3d::Zero()};
	bool has_prev_v_des_{false};    



    // rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr command_long_client_;


};
