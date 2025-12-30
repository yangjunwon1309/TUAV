
#include "waypoint_velocity_tracker/pitag_land_vel.h"
#include <fstream>
#include <sstream>
#include <thread>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

//added
#include <random>


#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <geometry_msgs/msg/point.hpp>

bool calculateMeanStdDev(const std::deque<Eigen::Vector3d>& window, Eigen::Vector3d& mean, double& stddev_norm) {
    if (window.empty()) {
        return false;
    }

    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    for (const auto& pos : window) {
        sum += pos;
    }
    mean = sum / static_cast<double>(window.size());

    if (window.size() < 2) {
        stddev_norm = 0.0;
    } else {
        double variance_sum_sq_norm = 0.0;
        for (const auto& pos : window) {
            variance_sum_sq_norm += (pos - mean).squaredNorm();
        }
        double variance_norm = variance_sum_sq_norm / static_cast<double>(window.size() - 1);
        stddev_norm = std::sqrt(variance_norm);
    }

    return true;
}

void PiTagLandVel::addTagPosition(const Eigen::Vector3d& pos, const geometry_msgs::msg::Quaternion& orientation) {
    if (tag_position_window_.empty()) {
        tag_position_window_.push_back(Eigen::Vector3d(
                initial_pose_.pose.position.x,
                initial_pose_.pose.position.y,
                initial_pose_.pose.position.z));
    }
    Eigen::Vector3d mean;
    double stddev_norm;
    double threshold;

    if (tag_position_window_.size() == 1) {
        mean = tag_position_window_.front();
        threshold = outlier_threshold_sigma_ * initial_sigma_;
        RCLCPP_DEBUG(get_logger(), "Window size 1: Mean=(%.2f, %.2f, %.2f), Threshold=%.3f (Initial Sigma)",
                    mean.x(), mean.y(), mean.z(), threshold);

    } else {
        if (calculateMeanStdDev(tag_position_window_, mean, stddev_norm)) {
            threshold = outlier_threshold_sigma_ * stddev_norm;
             RCLCPP_DEBUG(get_logger(), "Window size %zu: Mean=(%.2f, %.2f, %.2f), StdDev Norm=%.3f, Threshold=%.3f",
                         tag_position_window_.size(), mean.x(), mean.y(), mean.z(), stddev_norm, threshold);
        } else {
             RCLCPP_ERROR(get_logger(), "Failed to calculate mean/stddev for outlier check.");
             return;
        }
    }

    double dist = (pos - mean).norm();
    const double min_threshold = 0.05;
    threshold = std::max(threshold, min_threshold);

    if (dist > threshold) {
        RCLCPP_WARN(get_logger(), "Rejected tag detection (Position Outlier): %.3f m from mean (Threshold: %.3f m)", dist, threshold);
        return; 
    }

    tag_position_window_.push_back(pos);
    Eigen::Quaterniond eigen_orientation(orientation.w, orientation.x, orientation.y, orientation.z);
    tag_orientation_window_.push_back(eigen_orientation.normalized()); // Store normalized quaternion

    if (tag_position_window_.size() > window_size_) {
        tag_position_window_.pop_front();
    }

    if (tag_orientation_window_.size() > window_size_) {
        tag_orientation_window_.pop_front();
    }
}

Eigen::Quaterniond PiTagLandVel::getSmoothedTagOrientation() const {
    if (tag_orientation_window_.empty()) {
         return Eigen::Quaterniond(latest_tag_pose_map_.orientation.w,
                                   latest_tag_pose_map_.orientation.x,
                                   latest_tag_pose_map_.orientation.y,
                                   latest_tag_pose_map_.orientation.z).normalized();
    }

    Eigen::Vector4d avg_coeffs = Eigen::Vector4d::Zero();
    for (const auto& q : tag_orientation_window_) {
        if (tag_orientation_window_.front().coeffs().dot(q.coeffs()) < 0.0) {
            avg_coeffs -= q.coeffs(); // Flip the sign if in opposite hemisphere
        } else {
            avg_coeffs += q.coeffs();
        }
    }

    avg_coeffs.normalize();
    if (avg_coeffs.norm() < 1e-6) {
        RCLCPP_WARN(get_logger(), "Quaternion averaging resulted in near-zero vector, returning first element.");
        return tag_orientation_window_.front();
    }
    return Eigen::Quaterniond(avg_coeffs(3), avg_coeffs(0), avg_coeffs(1), avg_coeffs(2)); // Eigen stores as (x,y,z,w), constructor is (w,x,y,z)
}


using namespace std::chrono_literals;

template <typename T>
T clamp(const T& v, const T& lo, const T& hi)
{
    return std::min(std::max(v, lo), hi);
}

PiTagLandVel::PiTagLandVel()
    : Node("apriltag_land_vel"), has_vel_(false), tf_buffer_(get_clock()), tf_listener_(tf_buffer_) 
{
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    declare_parameter("p_gain", 1.0);
    declare_parameter("max_vel", 0.5);
    declare_parameter("descent_vel", 0.1);
    // given pos_thresh
    // declare_parameter("pos_thresh", 0.1);
    // changed pos_thresh
    declare_parameter("pos_thresh", 0.05);
    declare_parameter("timeout_sec", 2.0);
    declare_parameter("takeoff_height", 2.0);
    declare_parameter("takeoff_thresh", 0.1);
    declare_parameter("search_step", 0.2);
    declare_parameter("landing_origin", std::vector<double>{0.0, 0.0, 0.0});
    declare_parameter("yaw_gain", 0.5);
    declare_parameter<std::string>("mission_csv", "mission.csv");
    

    get_parameter("p_gain", p_gain_);
    get_parameter("max_vel", max_vel_);
    get_parameter("descent_vel", descent_vel_);
    get_parameter("pos_thresh", pos_thresh_);
    get_parameter("timeout_sec", timeout_sec_);
    get_parameter("takeoff_height", takeoff_height_);
    get_parameter("takeoff_thresh", takeoff_thresh_);
    get_parameter("search_step", search_step_);
    get_parameter("yaw_gain", yaw_gain_);
    get_parameter("mission_csv", mission_csv_path_);

    // tag_sub_ = create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
    //     "/apriltag_detections",rclcpp::SensorDataQoS(), std::bind(&PiTagLandVel::tagCallback, this, std::placeholders::_1));
    tag_sub_ = create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
        "/fiducial_april_detections",rclcpp::SensorDataQoS(), std::bind(&PiTagLandVel::tagCallback, this, std::placeholders::_1));
        
    local_position_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/mavros/local_position/pose", rclcpp::SensorDataQoS(),
        std::bind(&PiTagLandVel::poseCallback, this, std::placeholders::_1));

    // subscribe current_vel
    vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/mavros/local_position/velocity_local", rclcpp::SensorDataQoS(),
      std::bind(&PiTagLandVel::vel_callback, this, std::placeholders::_1));

    //added to switch to landing mode
    mode_sub_ = create_subscription<std_msgs::msg::Int32>(
    "/decon_uav/mode", 10,
    std::bind(&PiTagLandVel::modeCallback, this, std::placeholders::_1));
        
            
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
    // update attitude publisher
    att_pub_ = create_publisher<mavros_msgs::msg::AttitudeTarget>("/mavros/setpoint_raw/attitude", rclcpp::SensorDataQoS()); 
    latest_tag_position_map_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/latest_tag_position_map", 10);
    robot_position_map_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/robot_position_map", 10);
    
    // sending dummy tag position
    pub_dummy_pos_ = create_publisher<geometry_msgs::msg::Point>("/dummy_target_pos", 10);

    timer_ = create_wall_timer(20ms, std::bind(&PiTagLandVel::timerCallback, this));
    arming_cli_ = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    rclcpp::QoS state_qos(10);
    state_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
        "/mavros/state", state_qos,
        std::bind(&PiTagLandVel::state_callback, this, std::placeholders::_1));
    current_index_ = 0;
    drone_pose_from_tag_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/pose_relative_to_tag", 10);
    drone_start_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/drone_start_pose", 10);
    load_waypoints();

    clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    // command_long_client_ = this->create_client<mavros_msgs::srv::CommandLong>("/mavros/cmd/command");

    // Params for attitude-level ASMC (NEW)
    declare_parameter("hover_thrust", hover_thrust_);
    declare_parameter("asmc_L1", asmc_L1_);
    declare_parameter("asmc_gain", asmc_gain_);
    declare_parameter("asmc_max_accel", asmc_max_accel_);
    get_parameter("hover_thrust", hover_thrust_);
    get_parameter("asmc_L1", asmc_L1_);
    get_parameter("asmc_gain", asmc_gain_);
    get_parameter("asmc_max_accel", asmc_max_accel_);        
    
    RCLCPP_INFO(this->get_logger(), "PiTagLandVel node initialized.");
    
    
	// subscirbe IMU
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/mavros/imu/data_raw",   // or /mavros/imu/data ?
        rclcpp::SensorDataQoS(),
        std::bind(&PiTagLandVel::imuCallback, this, std::placeholders::_1));	    
    
    // publisher for cos_tilt
	cos_tilt_pub_ = this->create_publisher<std_msgs::msg::Float64>(
    "debug/cos_tilt", rclcpp::QoS(10));    
	
	// publisher for ext_force
    ext_force_dir_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
        "debug/ext_force_dir", rclcpp::QoS(10));    
    
}
void PiTagLandVel::load_waypoints() {
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
      std::getline(ss, token, ','); wp.yaw_deg = std::stod(token);
      waypoints_.push_back(wp);
    }
  }

void PiTagLandVel::state_callback(const mavros_msgs::msg::State::SharedPtr msg) {
    current_state_ = *msg;
  
    if (!offboard_active_ && msg->mode == "OFFBOARD") {
      RCLCPP_INFO(this->get_logger(), "Detected OFFBOARD mode transition, arming...");

      mavros_msgs::msg::AttitudeTarget att;
      att.type_mask =
          mavros_msgs::msg::AttitudeTarget::IGNORE_ROLL_RATE |
          mavros_msgs::msg::AttitudeTarget::IGNORE_PITCH_RATE |
          mavros_msgs::msg::AttitudeTarget::IGNORE_YAW_RATE;
      att.orientation = tf2::toMsg(tf2::Quaternion(0,0,0,1));

      // att.thrust = static_cast<float>(hover_thrust_);
      att.thrust = 0;
      
      
      auto t0 = this->now();
      while ((this->now() - t0).seconds() < 2.0) {
        att.header.stamp = this->now();
        att_pub_->publish(att);
        rclcpp::sleep_for(20ms);      
      }
  
      auto arm_req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
      arm_req->value = true;
  
      arming_cli_->async_send_request(arm_req,
        [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture result) {
          if (result.get()->success) {
            RCLCPP_INFO(this->get_logger(), "Drone armed by code.");
            this->offboard_active_ = true;
            this->switchToState(State::Takeoff);
          } else {
            RCLCPP_ERROR(this->get_logger(), "Arming failed.");
          }
        });
    }

    if (offboard_active_ && msg->mode != "OFFBOARD") {
        RCLCPP_WARN(get_logger(), "Exited OFFBOARD mode (now in %s)", msg->mode.c_str());
        // offboard_active_ = false;
        switchToState(State::Finished);  
    }
  }

void PiTagLandVel::switchToState(State new_state)
{
    state_ = new_state;
    if (state_ == State::Search) search_origin_ = current_position_;
    const char* state_str = "";
    switch (state_) {
        case State::Arming:  state_str = "Arming"; break;
        case State::Takeoff:  state_str = "Takeoff"; break;
        case State::Search:   state_str = "Search"; break;
        case State::Approach: state_str = "Approach"; break;
        case State::Descend:  state_str = "Descend"; break;
        case State::Finished: state_str = "Finished"; break;
    }

    RCLCPP_INFO(get_logger(), "Switched to state: %s", state_str);
}

void PiTagLandVel::tryArmingViaMavros()
{
    if (!arming_cli_->wait_for_service(1s)) {
        RCLCPP_WARN(get_logger(), "Arming service not available (timeout).");
        return;
    }

    auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    req->value = true;

    auto future = arming_cli_->async_send_request(req);

    // rclcpp::sleep_for(100ms);
    if (future.wait_for(1s) == std::future_status::ready) {
        auto resp = future.get();
        if (resp->success) {
            RCLCPP_INFO(get_logger(), "Arming succeeded via MAVROS!");
        } else {
            RCLCPP_ERROR(get_logger(), "Arming failed via MAVROS.");
        }
    } else {
        RCLCPP_ERROR(get_logger(), "Arming service call timed out.");
    }
}

void PiTagLandVel::publishVelocitySetpoint(double vx, double vy, double vz, double yaw)
{
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = this->now();
    msg.twist.linear.x = vx;
    msg.twist.linear.y = vy;
    msg.twist.linear.z = vz;
    
    msg.twist.angular.z = yaw;
    cmd_vel_pub_->publish(msg);
}

void PiTagLandVel::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if (!initial_pose_saved_ && offboard_active_) {
        initial_pose_ = *msg;
        initial_pose_saved_ = true;
        RCLCPP_INFO(this->get_logger(), "Initial pose saved: (%.2f, %.2f, %.2f)",
          initial_pose_.pose.position.x,
          initial_pose_.pose.position.y,
          initial_pose_.pose.position.z);

        //save initial_yaw
        tf2::Quaternion q(
            current_orientation_.x(),
            current_orientation_.y(),
            current_orientation_.z(),
            current_orientation_.w()
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch, current_yaw;
        m.getRPY(roll, pitch, current_yaw);
        initial_yaw_ = current_yaw;

        // Publish the initial pose as a separate topic
        geometry_msgs::msg::PoseStamped start_pose;
        start_pose.header = msg->header;
        start_pose.pose = initial_pose_.pose;
        drone_start_pose_pub_->publish(start_pose);
      }
    
    current_position_ = Eigen::Vector3d(
        msg->pose.position.x,
        msg->pose.position.y,
        msg->pose.position.z);
    
    current_orientation_ = Eigen::Quaterniond(
        msg->pose.orientation.w,
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z);
        

    //for simulation
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = msg->header.stamp;
    tf.header.frame_id = "map";         // local frame
    tf.child_frame_id = "base_link";     // drone body frame

    tf.transform.translation.x = msg->pose.position.x;
    tf.transform.translation.y = msg->pose.position.y;
    tf.transform.translation.z = msg->pose.position.z;
    tf.transform.rotation = msg->pose.orientation;

    tf_broadcaster_->sendTransform(tf);
}

void PiTagLandVel::tagCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
{
    if (msg->detections.empty()) {
        tag_detected_ = false;
        return;
    }
    tag_detected_ = true;
    last_tag_time_ = clock_->now();  // consistently use clock_

    auto pose_with_cov = msg->detections[0].pose;
    try {

        geometry_msgs::msg::PoseStamped ps_in, ps_out;
        ps_in.header = msg->header;
        ps_in.pose = pose_with_cov.pose.pose;

        auto tf = tf_buffer_.lookupTransform("map", ps_in.header.frame_id, tf2::TimePointZero);
        tf2::doTransform(ps_in, ps_out, tf);

        latest_tag_pose_map_ = ps_out.pose;
        // RCLCPP_INFO(get_logger(), "Detected tag at (%.2f, %.2f, %.2f) in map frame",
        //             latest_tag_pose_.position.x, latest_tag_pose_.position.y, latest_tag_pose_.position.z);
        latest_tag_position_map_pub_->publish(ps_out);

        //publish AGV position in map frame: 1m below the tag
        latest_robot_pose_map_.position.x = ps_out.pose.position.x;
        latest_robot_pose_map_.position.y = ps_out.pose.position.y;
        latest_robot_pose_map_.position.z = ps_out.pose.position.z - 1.0;
        latest_robot_pose_map_.orientation = ps_out.pose.orientation;
        
        if (tagDetectedRecently()) {
            addTagPosition(Eigen::Vector3d(
                ps_out.pose.position.x,
                ps_out.pose.position.y,
                ps_out.pose.position.z),
                ps_out.pose.orientation);        
        }

        geometry_msgs::msg::PoseStamped robot_pose_map;
        robot_pose_map.header.stamp = ps_out.header.stamp;
        robot_pose_map.header.frame_id = "map";
        robot_pose_map.pose = latest_robot_pose_map_;

        robot_position_map_pub_->publish(robot_pose_map);
        
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(get_logger(), "TF transform failed: %s", ex.what());
        tag_detected_ = false;
    }
}

void PiTagLandVel::timerCallback()
{
    switch (state_) {
        case State::Arming: break;
        case State::Takeoff: handleTakeoff(); break;
        case State::Search: handleSearch(); break;
        case State::Approach: handleApproach(); break;
        case State::Descend: handleDescend(); break;
        case State::Finished: break;
    }
}

void PiTagLandVel::vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
  current_vel_ = *msg;
  has_vel_ = true;
}


Eigen::Vector3d PiTagLandVel::getSmoothedTagPosition() const {
    if (tag_position_window_.empty()) {

        return Eigen::Vector3d(
            latest_tag_pose_map_.position.x,
            latest_tag_pose_map_.position.y,
            latest_tag_pose_map_.position.z);
    }

    Eigen::Vector3d sum(0.0, 0.0, 0.0);
    for (const auto& pos : tag_position_window_) {
        sum += pos;
    }
    return sum / static_cast<double>(tag_position_window_.size());
}


// ================= get Dummy Tag position ====================== //
Eigen::Vector3d PiTagLandVel::getDummyTagPosition() {
    
    Eigen::Vector3d initial_position;
    initial_position << initial_pose_.pose.position.x,
                        initial_pose_.pose.position.y,
                        initial_pose_.pose.position.z + takeoff_height_;

    double dt = this->dt_; 
    if (std::isnan(dt) || dt <= 0.0) dt = 0.02;
    
    accumulated_time_ += dt; 
    double t = accumulated_time_;

    // change dummy_mode
    int dummy_mode_ = 0; 

// =========== check vel_limit (boundary ~~ AW)  ========== //    
    // change amplitude
    //double A = 2.0;
    double A = 1.0;
    
    // change rate
    //double w = 0.5;
    double w = 0.2;

// =========================================    
    
    // change angle
    //double alpha = M_PI_4; // for mode 0, 1
    double alpha = 0;
    
    
    double w_prime = 0.2;  // for mode 2 

    double deltax = 0.0;
    double deltay = 0.0;

    if (dummy_mode_ == 0) {                                     // Simple Harmonic (Sin)
        double osc = A * std::sin(w * t);
        deltax = osc * std::cos(alpha);
        deltay = osc * std::sin(alpha);

    } else if (dummy_mode_ == 1) {                              // Constant Velocity Oscillation (Triangle)
        double linear_osc = (2.0 * A / M_PI) * std::asin(std::sin(w * t));
        deltax = std::cos(alpha) * linear_osc;
        deltay = std::sin(alpha) * linear_osc;

    } else if (dummy_mode_ == 2) {                              // Rose/Spiral Curve
        double R = A * std::pow(std::sin(w_prime * t), 2); 
        deltax = R * std::cos(w * t);
        deltay = R * std::sin(w * t);
        
    } else if (dummy_mode_ == 3) {                  //  Drifting 
        double v_drift = 0.1;  // m/s 
        double drift = v_drift * t;

        // drift orientation (alpha)
        deltax = drift * std::cos(alpha);  
        deltay = drift * std::sin(alpha);  
    }
    
    
    

    Eigen::Vector3d dummy_position = initial_position;
    dummy_position.x() += deltax;
    dummy_position.y() += deltay;
    
// ====== noise setup ====== //
    // random noise, current std = 0
    static std::random_device rd;
    static std::mt19937 gen(rd());
    
    double noise_mean = 0.0;
    double noise_stddev = 0.0;

    if (noise_stddev > 0.0) {
        std::normal_distribution<double> dist(noise_mean, noise_stddev);
        
        dummy_position.x() += dist(gen);
        dummy_position.y() += dist(gen);
        dummy_position.z() += dist(gen);
    }
// =======  end ============= //    

    if (pub_dummy_pos_) { 
        geometry_msgs::msg::Point msg;
        msg.x = dummy_position.x();
        msg.y = dummy_position.y();
        msg.z = dummy_position.z();
        
        pub_dummy_pos_->publish(msg);
    }

    return dummy_position; 
}

// =========================== end =============================== //


void PiTagLandVel::modeCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  current_mode_ = msg->data;
  RCLCPP_INFO(get_logger(), "Mode updated via /decon_uav/mode: %d", current_mode_);
}


void PiTagLandVel::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    imu_accel_body_ = Eigen::Vector3d(
        msg->linear_acceleration.x,
        msg->linear_acceleration.y,
        msg->linear_acceleration.z
    );
    has_imu_accel_ = true;
}


Eigen::Vector2d PiTagLandVel::asmc_xy_(
    const Eigen::Vector2d& p_err,
    double lambda, double k_min, double k_max, double gamma, double phi,
    Eigen::Vector2d& k_adapt)
{
    double vx = std::clamp(this->p_gain_ * (-p_err.x()), -this->search_step_, this->search_step_);
    double vy = std::clamp(this->p_gain_ * (-p_err.y()), -this->search_step_, this->search_step_);

    Eigen::Vector2d v_ref(vx, vy);

    Eigen::Vector2d v_meas(
        this->current_vel_.twist.linear.x,
        this->current_vel_.twist.linear.y
    );
    if (!this->has_vel_) v_meas.setZero();

    Eigen::Vector2d v_err = v_meas - v_ref;
    Eigen::Vector2d s     = v_err + lambda * p_err;

    Eigen::Vector2d sat_s;
    for (int i = 0; i < 2; ++i) {
        double denom = std::abs(s[i]) + phi;
        sat_s[i] = (denom > 1e-6) ? (s[i] / denom) : 0.0;
    }

    k_adapt += gamma * s.cwiseAbs() * this->dt_;
    
    for (int i = 0; i < 2; ++i)    
        k_adapt[i] = std::clamp(k_adapt[i], k_min, k_max);

    Eigen::Vector2d v_cmd = v_ref - lambda * p_err - k_adapt.cwiseProduct(sat_s);
    return v_cmd;
}


Eigen::Vector3d PiTagLandVel::asmc3d_from_vdes(const Eigen::Vector3d& target_pos,
                                               const Eigen::Vector3d& v_des)
{
    Eigen::Vector3d p_now = current_position_;
    Eigen::Vector3d v_now(current_vel_.twist.linear.x,
                          current_vel_.twist.linear.y,
                          current_vel_.twist.linear.z);
    
    double dt = std::max(0.02, this->dt_);

    Eigen::Vector3d delta_v_req = v_des - ref_vel_smooth_;   // default value of ref_vel_smooth_ is zero
    
    // ensuring delta v physical constraints
    double max_delta_v = asmc_max_accel_ * dt;
    
    if (delta_v_req.norm() > max_delta_v) {
        delta_v_req = delta_v_req.normalized() * max_delta_v;
    }

    Eigen::Vector3d a_ref_smooth = delta_v_req / dt; 
    
    ref_vel_smooth_ += delta_v_req;

    Eigen::Vector3d e_p = target_pos - p_now;      // (target - current)
    Eigen::Vector3d e_v = ref_vel_smooth_ - v_now;
    
    // Option 1: light deadzone on x,y
    Eigen::Vector3d e_p_f(
        e_p.x() * std::abs(std::tanh(e_p.x())) * 0.5,
        e_p.y() * std::abs(std::tanh(e_p.y())) * 0.5,
        e_p.z());
    
    // Option 2: light deadzone on x,y,z
    //Eigen::Vector3d e_p_f(
    //    e_p.x() * std::abs(std::tanh(e_p.x())) * 0.5,
    //    e_p.y() * std::abs(std::tanh(e_p.y())) * 0.5,
    //    e_p.z()* std::abs(std::tanh(e_p.z())) * 1.0);    
    
    // Option 3: without deadzone
    //Eigen::Vector3d e_p_f(
    //    e_p.x(),
    //    e_p.y(),
    //    e_p.z());        
    
    Eigen::Vector3d s = e_v + asmc_C_.cwiseProduct(e_p_f);
    Eigen::Vector3d sat_s = (s.array() / (s.cwiseAbs().array() + asmc_phi_.array())).matrix();

    // k_adapt_3d_ += d_adapt_3d_.cwiseProduct(s.cwiseAbs()) * this->dt_;
    // k_adapt_3d_ = k_adapt_3d_.cwiseMax(k_min_3d_).cwiseMin(k_max_3d_);
    
    Eigen::Vector3d s_abs = s.cwiseAbs();
    Eigen::Vector3d sign_term;

    double asmc_eps_ = 0.2; 
    
    for(int i=0; i<3; i++) {
        if (s_abs(i) > asmc_eps_) {
            sign_term(i) = 1.0;  // Error is large -> Increase Gain
        } else {
            sign_term(i) = -1.0; // Error is small -> Decrease Gain
        }
    }

    // k_new = k_old + (d_adapt * |s| * sign_term) * dt
    k_adapt_3d_ += d_adapt_3d_.cwiseProduct(s_abs).cwiseProduct(sign_term) * dt;
    k_adapt_3d_ = k_adapt_3d_.cwiseMax(k_min_3d_).cwiseMin(k_max_3d_);

// ==============  without accel_ref ====================== //    
//    Eigen::Vector3d a_cmd = ( Kp.cwiseProduct(e_p_f)
//                            + Kd.cwiseProduct(e_v)
//                            + k_adapt_3d_.cwiseProduct(sat_s) );
                            
//    Eigen::Vector3d a_cmd = ( Kp.cwiseProduct(e_p_f)
//                            + Kd.cwiseProduct(e_v)
//                            );                            


// ================ trial with accel_ref ================== //

    Eigen::Vector3d Kp(p_gain_, p_gain_, p_gain_);

    Eigen::Vector3d Kd(
        1.0,
        1.0,
        1.0
    );

    Eigen::Vector3d a_fb = ( Kp.cwiseProduct(e_p_f)
                           + Kd.cwiseProduct(e_v)
                           + k_adapt_3d_.cwiseProduct(sat_s) );

    prev_v_des_ = v_des;
    has_prev_v_des_ = true;

    //    // a_cmd = a_ref_smooth + a_fb
    Eigen::Vector3d a_cmd = a_ref_smooth + a_fb;
// ================= end ===================================== //
                            
    a_cmd /= std::max(1e-3, asmc_L1_);
    a_cmd *= asmc_gain_;
    
    // xy clamp
    auto lim_xy = [this](double a){ return asmc_max_accel_ * std::tanh(a / std::max(1e-6, asmc_max_accel_)); };
    a_cmd.x() = lim_xy(a_cmd.x()); a_cmd.y() = lim_xy(a_cmd.y()); 
    
    // z clamp
	auto lim_z = [](double a){
		const double max_z = 0.3;  
		return max_z * std::tanh(a / std::max(1e-6, max_z));
	};    
    
    a_cmd.z() = lim_z(a_cmd.z());
    
    // ============== check log ======================
    static int dbg_cnt = 0;
    dbg_cnt++;
    if (dbg_cnt % 100 == 0) {
        RCLCPP_INFO(this->get_logger(),
                    "[ASMC3D] a_cmd = [%.3f, %.3f, %.3f]",
                    a_cmd.x(), a_cmd.y(), a_cmd.z());
    }
    // =============================================    
    
    return a_cmd;
}


void PiTagLandVel::publishAttitudeTargetFromAccel(const Eigen::Vector3d& a_cmd,
                                                  double desired_yaw,
                                                  double hover_thrust)
{
    constexpr double g = 9.81;
    Eigen::Vector3d total_accel = a_cmd + Eigen::Vector3d(0,0,g);
    tf2::Vector3 z_b(total_accel.x(), total_accel.y(), total_accel.z());
    z_b.normalize();
    tf2::Vector3 x_c(std::cos(desired_yaw), std::sin(desired_yaw), 0.0);
    tf2::Vector3 y_b = z_b.cross(x_c);
    if (y_b.length() < 1e-3) { x_c = tf2::Vector3(1,0,0); y_b = z_b.cross(x_c); }
    y_b.normalize();
    tf2::Vector3 x_b = y_b.cross(z_b);
    tf2::Matrix3x3 R;
    R[0][0]=x_b.x(); R[1][0]=x_b.y(); R[2][0]=x_b.z();
    R[0][1]=y_b.x(); R[1][1]=y_b.y(); R[2][1]=y_b.z();
    R[0][2]=z_b.x(); R[1][2]=z_b.y(); R[2][2]=z_b.z();
    tf2::Quaternion q_des; R.getRotation(q_des);
    
	// thrust_norm rewarding cos tilt   
    const double cos_tilt = std::max(0.25, static_cast<double>(z_b.z())); // avoid 0-division
    const double thrust_feed = (a_cmd.z() + g) / (g * cos_tilt);   
    double thrust_norm = hover_thrust * thrust_feed;
    thrust_norm = clamp(thrust_norm, 0.0, 1.0);    
    
	if (cos_tilt_pub_) {
		std_msgs::msg::Float64 msg;
		msg.data = cos_tilt;
		cos_tilt_pub_->publish(msg);
	}        
	
    if (ext_force_dir_pub_ && has_imu_accel_) {

        Eigen::Vector3d a_world = current_orientation_ * imu_accel_body_;
        
        
        // case for with g
        // Eigen::Vector3d g_vec(0.0, 0.0, g);
        // Eigen::Vector3d a_world_no_g = a_world - g_vec;

        Eigen::Vector3d z_b_world(z_b.x(), z_b.y(), z_b.z());
        
        double hover_safe = std::max(hover_thrust, 1e-3);

        double a_thrust_mag = (thrust_norm / hover_safe) * g; 
        Eigen::Vector3d a_thrust_world = a_thrust_mag * z_b_world;

		// case for with g
        //Eigen::Vector3d a_ext_world = a_world_no_g - a_thrust_world;
        
        Eigen::Vector3d a_ext_world = a_world - a_thrust_world;

        geometry_msgs::msg::Vector3 dir_msg;
        
        
        
// =============== normalized data ============================= //        
//        if (a_ext_world.norm() > 1e-3) {
//            Eigen::Vector3d dir_ext = a_ext_world.normalized();
//            dir_msg.x = dir_ext.x();
//            dir_msg.y = dir_ext.y();
//            dir_msg.z = dir_ext.z();
//        } else {
//            dir_msg.x = 0.0;
//            dir_msg.y = 0.0;
//            dir_msg.z = 0.0;
//        }
// ================= raw data  ============================ //        
		if (a_ext_world.norm() > 1e-3) {
			dir_msg.x = a_ext_world.x();
			dir_msg.y = a_ext_world.y();
			dir_msg.z = a_ext_world.z();
		} else {
			dir_msg.x = 0.0;
			dir_msg.y = 0.0;
			dir_msg.z = 0.0;
		}        
// ================= end ======================================//				
        ext_force_dir_pub_->publish(dir_msg);
    }	
	
    mavros_msgs::msg::AttitudeTarget att;
    att.header.stamp = now();
    
// =========== commander version ====================  //    
//    att.type_mask =
//        mavros_msgs::msg::AttitudeTarget::IGNORE_ROLL_RATE |
//        mavros_msgs::msg::AttitudeTarget::IGNORE_PITCH_RATE;
//    att.orientation = tf2::toMsg(q_des);
//    att.thrust = static_cast<float>(thrust_norm);
//    att.body_rate.z = 0.0f; 
//    att_pub_->publish(att);
// ============   

    att.type_mask =
        mavros_msgs::msg::AttitudeTarget::IGNORE_ROLL_RATE |
        mavros_msgs::msg::AttitudeTarget::IGNORE_PITCH_RATE |
        mavros_msgs::msg::AttitudeTarget::IGNORE_YAW_RATE;
    att.orientation = tf2::toMsg(q_des);
    att.thrust = static_cast<float>(thrust_norm); 
    att_pub_->publish(att);

}


Eigen::Vector3d desired_pos_{0,0,0}, desired_pos_prev_{0,0,0};
double desired_yaw_{0.0}, desired_yaw_prev_{0.0};
rclcpp::Time last_time_;

inline double wrapPi(double a){
    while (a >  M_PI) a -= 2*M_PI;
    while (a < -M_PI) a += 2*M_PI;
    return a;
}
inline double angLerp(double a, double b, double alpha){
    double d = wrapPi(b - a);
    return wrapPi(a + alpha * d);
}

// ============= request_disarm =========================


void PiTagLandVel::requestDisarm()
{
    if (!arming_cli_) {
        RCLCPP_WARN(get_logger(), "Arming client not initialized");
        return;
    }

    if (!arming_cli_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_WARN(get_logger(), "Arming service not available");
        return;
    }

    auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    req->value = false;  // false = disarm

    auto result_future = arming_cli_->async_send_request(req);
}

// ============= end ===============================


void PiTagLandVel::handleTakeoff()
{
    if (!takeoff_origin_set_) {
        takeoff_origin_ = current_position_;
        takeoff_origin_set_ = true;

        tf2::Quaternion q(
            current_orientation_.x(),
            current_orientation_.y(),
            current_orientation_.z(),
            current_orientation_.w()
        );

        RCLCPP_INFO(get_logger(), "Takeoff origin x=%.2f, y=%.2f, z=%.2f",takeoff_origin_.x(),takeoff_origin_.y(),takeoff_origin_.z());
    }

    Eigen::Vector3d target = takeoff_origin_;
    target.z() += takeoff_height_;

    Eigen::Vector3d error = target - current_position_;
    double max_vel_takeoff_xy = 0.1;
    double vx = clamp(p_gain_ * error.x(), -max_vel_takeoff_xy, max_vel_takeoff_xy);
    double vy = clamp(p_gain_ * error.y(), -max_vel_takeoff_xy, max_vel_takeoff_xy);

    double max_vel_takeoff_z = 0.3;
    double vz = clamp(p_gain_ * error.z(), -max_vel_takeoff_z, max_vel_takeoff_z);
    
    if (std::abs(error.z()) < takeoff_thresh_) {
        RCLCPP_INFO(get_logger(), "Takeoff complete → switching to Search");
        switchToState(State::Search);
        return;
    }
    
    Eigen::Vector3d v_des(vx, vy, vz);
    Eigen::Vector3d target_pos = target;
    Eigen::Vector3d a_cmd = asmc3d_from_vdes(target_pos, v_des);
    double desired_yaw = 0.0;
    publishAttitudeTargetFromAccel(a_cmd, desired_yaw, hover_thrust_);
    
    //RCLCPP_INFO(get_logger(), "Takeoff(att): vdes(%.2f,%.2f,%.2f)", v_des.x(), v_des.y(), v_des.z());    
    
}

void PiTagLandVel::handleSearch()
{
    if(current_index_ >= waypoints_.size()) {
        RCLCPP_INFO(get_logger(), "All waypoints processed, switching to Finished state.");
        switchToState(State::Approach);
        return;
    }

	auto now = this->get_clock()->now();
	double dt = last_time_.nanoseconds() > 0 ? (now - last_time_).seconds() : 1.0/50.0;
	last_time_ = now;

	// update dt
	dt_=dt;

    double target_x= initial_pose_.pose.position.x + waypoints_[current_index_].x;
    double target_y= initial_pose_.pose.position.y+waypoints_[current_index_].y;
    double target_z = initial_pose_.pose.position.z+ waypoints_[current_index_].z;
    double waypoint_rad= waypoints_[current_index_].yaw_deg * M_PI / 180.0; // Convert to radians

	tf2::Quaternion q_initial(
		initial_pose_.pose.orientation.x,
		initial_pose_.pose.orientation.y,
		initial_pose_.pose.orientation.z,
		initial_pose_.pose.orientation.w
	);

	tf2::Matrix3x3 m_initial(q_initial);
	double roll_initial, pitch_initial; // no use
	double initial_yaw_rad;
	m_initial.getRPY(roll_initial, pitch_initial, initial_yaw_rad);
    double target_yaw_rad = initial_yaw_rad + waypoint_rad;

    //  double target_yaw = 0.0;
     tf2::Quaternion q(
         current_orientation_.x(),
         current_orientation_.y(),
         current_orientation_.z(),
         current_orientation_.w()
     );

     tf2::Matrix3x3 m(q);
     double roll, pitch, current_yaw;
     m.getRPY(roll, pitch, current_yaw);

     double yaw_err = target_yaw_rad - current_yaw;
     while (yaw_err > M_PI) yaw_err -= 2 * M_PI;
     while (yaw_err < -M_PI) yaw_err += 2 * M_PI;

     if (positionReachedXY(Eigen::Vector3d(target_x, target_y, target_z)) && std::abs(yaw_err) < 0.1) {
         RCLCPP_INFO(get_logger(), "Reached waypoint %zu: (%.2f, %.2f, %.2f)", current_index_, target_x, target_y, target_z);
         current_index_++;

         if (current_index_ >= waypoints_.size()) {
             RCLCPP_INFO(get_logger(), "All waypoints processed, switching to Finished state.");
             switchToState(State::Approach);

             return;
         }
         return;
     }


    double dx = target_x - current_position_.x();
    double vx = clamp(p_gain_ * dx, -search_step_, search_step_);
    
    double dy = target_y - current_position_.y();
    double vy = clamp(p_gain_ * dy, -search_step_, search_step_);

    double dz = target_z - current_position_.z();
    double vz = clamp(p_gain_ * dz, -search_step_, search_step_);

    Eigen::Vector2d p_err(- dx, - dy);

    // option 1: use previous util asmc
    //Eigen Eigen::Vector2d v_cmd_xy = asmc_xy_(p_err, lambda_search_, kmin_search_, kmax_search_, gamma_search_, phi_search_, k_xy_search_);
    // double vx_cmd = clamp(v_cmd_xy.x(), -search_step_,  search_step_);
	// double vy_cmd = clamp(v_cmd_xy.y(), -search_step_,  search_step_);
    //Eigen::Vector3d v_des(vx_cmd, vy_cmd, vz);
    
    // option 2 : without util
    Eigen::Vector3d v_des(vx, vy, vz);

    Eigen::Vector3d target_pos(target_x, target_y, target_z);
    Eigen::Vector3d a_cmd = asmc3d_from_vdes(target_pos, v_des);

    //target_yaw_rad = 0.0;
    publishAttitudeTargetFromAccel(a_cmd, target_yaw_rad, hover_thrust_);

}

void PiTagLandVel::handleApproach()
{
    double max_approach_vel_=0.3;

	auto now = this->get_clock()->now();
	double dt = last_time_.nanoseconds() > 0 ? (now - last_time_).seconds() : 1.0/50.0;
	last_time_ = now;
    
    dt_ = dt;


// =========== Option for tagposition ============ //
	// Option 1 : setup for static state
    Eigen::Vector3d target_pos = getSmoothedTagPosition();
    
    // Option 2 : setup for changing state
    //Eigen::Vector3d target_pos = getDummyTagPosition();
    
    
// ========== end =================================//    
    
    Eigen::Quaterniond target_orientation = getSmoothedTagOrientation();

    double dx = target_pos.x() - current_position_.x();
    double dy = target_pos.y() - current_position_.y();
    
    // use takeoff_origin.z()
    // double target_z = takeoff_origin_.z() + takeoff_height_;
    // use initial_pose_.pose.position.z
    double target_z = initial_pose_.pose.position.z + takeoff_height_;
    
    double dz = target_z - current_position_.z();

    // double vx = clamp(p_gain_ * dx, -max_approach_vel_, max_approach_vel_);
    // double vy = clamp(p_gain_ * dy, -max_approach_vel_, max_approach_vel_);
    // double vz = clamp(p_gain_ * dz, -max_vel_, max_vel_);
    // // RCLCPP_INFO(get_logger(), "vel command sent (%.2f, %.2f)", vx, vy);
	// Eigen::Vector2d p_err(-dx, -dy);
	// Eigen::Vector2d v_cmd_xy = asmc_xy_(p_err,
	// 	lambda_approach_, kmin_approach_, kmax_approach_, gamma_approach_, phi_approach_,
	// 	k_xy_approach_
	// );
	// double vx_cmd = clamp(v_cmd_xy.x(), -search_step_,  search_step_);
	// double vy_cmd = clamp(v_cmd_xy.y(), -search_step_,  search_step_);

// ========== target_yaw =======================
    tf2::Quaternion tag_q(
        target_orientation.x(),
        target_orientation.y(),
        target_orientation.z(),
        target_orientation.w()
    );
    tf2::Matrix3x3 tag_m(tag_q);
    double tag_roll, tag_pitch, target_yaw;
    tag_m.getRPY(tag_roll, tag_pitch, target_yaw);
    
// ============ current_yaw ==================
    // Convert current orientation to yaw angle
    tf2::Quaternion q(
        current_orientation_.x(),
        current_orientation_.y(),
        current_orientation_.z(),
        current_orientation_.w()
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, current_yaw;
    m.getRPY(roll, pitch, current_yaw);    
        
    // yaw error 계산 및 정규화 - tag 90도 회전되어 detection됨
    double yaw_err = (target_yaw+M_PI/2) - current_yaw;
    while (yaw_err > M_PI) yaw_err -= 2 * M_PI;
    while (yaw_err < -M_PI) yaw_err += 2 * M_PI;

    // yaw 속도 명령 생성
    double yaw_rate = clamp(yaw_gain_ * yaw_err, -0.2, 0.2);     
// ========================================================    

    static rclcpp::Time reached_time;
    static bool reached = false;
    
    
    if (!have_init_desired_) {
        desired_pos_ = Eigen::Vector3d(
		    current_position_.x(),
		    current_position_.y(),
		    target_z
		);
        
        desired_pos_prev_ = desired_pos_;
        desired_yaw_ = current_yaw;
        have_init_desired_ = true;
    }

	double tau_pos = 0.2;
	double alpha_p = 1.0 - std::exp(-dt/tau_pos);
	Eigen::Vector3d target_wp(target_pos.x(), target_pos.y(), target_z);
	desired_pos_ = desired_pos_ + alpha_p*(target_wp - desired_pos_);

//============= deactivated in the current setup ==========================
	double tau_yaw = 0.4;
	double alpha_y = 1.0 - std::exp(-dt/tau_yaw);
	desired_yaw_ = angLerp(desired_yaw_, (target_yaw+M_PI/2), alpha_y);	

	Eigen::Vector3d v_des = (desired_pos_ - desired_pos_prev_) / std::max(dt, 1e-3);
	v_des.x() = clamp(v_des.x(), -search_step_, search_step_);
	v_des.y() = clamp(v_des.y(), -search_step_, search_step_);
	v_des.z() = clamp(v_des.z(), -0.1, 0.1);
	desired_pos_prev_ = desired_pos_;
	
	
// ============= Error log for dummy ==========================	
   // use above dx, dy

//    static int log_counter = 0;
//    log_counter++;
//
//    if (log_counter % 50 == 0) {
//        RCLCPP_INFO(get_logger(),
//                    "[TRACK] ex=%.3f ey=%.3f",
//                    dx, dy);
//    }

	// use above dx,dy,dz 
	// === sample index ===
	static std::size_t sample_idx = 0;
	static double sum_e_xy_sq = 0.0;   // Σ[(dx^2 + dy^2)]
	static double sum_abs_dz  = 0.0;   // Σ|dz|

	// time step
	if (std::isnan(dt) || dt <= 0.0) dt = 0.02;
	// accumulated_time_ += dt;
	double t = accumulated_time_;          // until current_time [s]

	sample_idx++;

	double e_xy_sq = dx*dx + dy*dy;
	double abs_dz  = std::fabs(dz);

	sum_e_xy_sq += e_xy_sq;
	sum_abs_dz  += abs_dz;

	// X -> RMSE, Z -> mean
	double X_rmse = std::sqrt(sum_e_xy_sq / static_cast<double>(sample_idx));
	double Z_mean =        (sum_abs_dz  / static_cast<double>(sample_idx));

	// log
//	if (sample_idx % 100 == 0) {
//		RCLCPP_INFO(
//		    get_logger(),
//		    "[TRACK] k=%zu t=%.3f dt=%.3f "
//		    "ex=%.3f ey=%.3f ez=%.3f  "
//		    "X_rmse=%.3f Z_mean=%.3f",
//		    sample_idx, t, dt,
//		    dx, dy, dz,
//		    X_rmse, Z_mean);
//	}
    
// ========================== end ==============================    
    
    Eigen::Vector3d a_cmd = asmc3d_from_vdes(desired_pos_, v_des);
    double desired_yaw = target_yaw + M_PI/2.0;
    publishAttitudeTargetFromAccel(a_cmd, desired_yaw, hover_thrust_);     

    if (std::abs(dx) < pos_thresh_ && std::abs(dy) < pos_thresh_ && std::abs(yaw_err) < 0.1 && !reached) {
        RCLCPP_INFO(get_logger(), "Approach complete → switching to Descend");

        reached = true;
        reached_time = this->now();

    }
    else if (reached && current_mode_ == 4 && (this->now() - reached_time).seconds() > 5.0) {
        RCLCPP_INFO(get_logger(), "Approach timed out, switching to Descend");
        switchToState(State::Descend);
    }
}

// ==========================  approach --> descending ========================

void PiTagLandVel::handleDescend()
{

// =============  use below target pos  (especially z value) =============================
//    if (!initial_pose_saved_ && offboard_active_) {
//        initial_pose_ = *msg;
//        initial_pose_saved_ = true;
//        RCLCPP_INFO(this->get_logger(), "Initial pose saved: (%.2f, %.2f, %.2f)",
//          initial_pose_.pose.position.x,
//          initial_pose_.pose.position.y,
//          initial_pose_.pose.position.z);
// ==================================================================

    Eigen::Vector3d target_pos = getSmoothedTagPosition();
    Eigen::Quaterniond target_orientation = getSmoothedTagOrientation();

    // linearly decreasing descending velocity z
    //double h_current = current_position_.z() - target_pos.z();
    
    // option 1 : general
    double h_current = current_position_.z() - initial_pose_.pose.position.z;
    
    // option 2 : fixed h_current
	//double h_current = current_position_.z() - initial_pose_.pose.position.z;
    
    
    double vz_fast = -0.3;
    
    // small
    //double vz_slow = -0.08;
    // increasing
    double vz_slow = -0.2;
    

    auto lin_map = [](double val, double in_min, double in_max, double out_min, double out_max) {
        double t = std::clamp((val - in_min) / (in_max - in_min), 0.0, 1.0);
        return out_min + t * (out_max - out_min);
    };
    
    
    
    // increasing min_height
    double vz_cmd = lin_map(h_current, 0.5, 1.5, vz_slow, vz_fast);

    Eigen::Vector3d error = target_pos - current_position_;
    
    // boundary
    //Eigen::Vector3d vel_des = Kp_descend_.cwiseProduct(error);
    
	Eigen::Vector3d vel_raw = Kp_descend_.cwiseProduct(error);

	Eigen::Vector3d vel_des = vel_raw.cwiseMax(-0.1).cwiseMin(0.1);    
    
    
	double d_xy = std::hypot(error.x(), error.y());
    Eigen::Vector2d v_p_xy(
        vel_des.x(),
        vel_des.y()
    );
    Eigen::Vector2d p_err(-error.x(), -error.y()); 
	Eigen::Vector2d v_asmc_xy = asmc_xy_(
		p_err,
		lambda_descend_, kmin_descend_, kmax_descend_, gamma_descend_, phi_descend_,
		k_xy_descend_
	);
	
// ========================    not use ==============================//	
	auto lin = [](double x, double a, double b){ return std::clamp((x-a)/(b-a), 0.0, 1.0); };
	
	double w_err = lin(d_xy, 0.05, 0.5); 
	double h      = current_position_.z();
	double w_alt = lin(h,    0.1,  1.0);
	double w     = std::clamp(0.6*w_err + 0.4*w_alt, 0.0, 1.0);
	
	Eigen::Vector2d vel_new = (1.0 - w) * v_p_xy + w * v_asmc_xy;
	vel_new.x() = clamp(vel_new.x(), -0.1, 0.1);
	vel_new.y() = clamp(vel_new.y(), -0.1, 0.1);
// ===================================================================//

    tf2::Quaternion tag_q(
        target_orientation.x(),
        target_orientation.y(),
        target_orientation.z(),
        target_orientation.w()
    );
    tf2::Matrix3x3 tag_m(tag_q);
    double tag_roll, tag_pitch, target_yaw;
    tag_m.getRPY(tag_roll, tag_pitch, target_yaw);

    tf2::Quaternion q(
        current_orientation_.x(),
        current_orientation_.y(),
        current_orientation_.z(),
        current_orientation_.w()
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, current_yaw;
    m.getRPY(roll, pitch, current_yaw);

    double yaw_err = (target_yaw+M_PI) - current_yaw;
    while (yaw_err > M_PI) yaw_err -= 2 * M_PI;
    while (yaw_err < -M_PI) yaw_err += 2 * M_PI;

    double yaw_rate = clamp(yaw_gain_ * yaw_err, -0.5, 0.5);

    // RCLCPP_INFO(get_logger(), "Landing: vx=%.2f vy=%.2f vz=%.2f yaw_rate=%.2f", 
    //             vel_cmd.x(), vel_cmd.y(), vel_cmd.z(), yaw_rate);

    // option 0 : use original blended velocity command
    //publishVelocitySetpoint(vel_new.x(), vel_new.y(), vz_cmd, yaw_rate);
    
    // option 1: use util_asmc
    //Eigen::Vector3d v_des(vel_new.x(), vel_new.y(), vz_cmd);
    
    // option 2: without util_asmc
    Eigen::Vector3d v_des(vel_des.x(), vel_des.y(), vz_cmd); 
    
    Eigen::Vector3d target_pos_vec = target_pos;
    
// ==============  setup for target_pos ================== //    
    // option 1 : add bias
    //double bias = 0.2;
    //target_pos_vec.z() = current_position_.z() - bias;
    
    // option 2 : use z profile

	if (!z_ref_initialized_) {
		z_ref_ = current_position_.z();
		z_ref_initialized_ = true;
	}
	
	double dt = this->dt_;
	if (std::isnan(dt) || dt <= 0.0) dt = 0.02;

	z_ref_ += vz_cmd * dt;

	double z0 = initial_pose_.pose.position.z;
	
	double h_ref = z_ref_ - z0;
	
	double h_min = -0.2;

	double h_max =  4.0;
	
	h_ref = std::clamp(h_ref, h_min, h_max);
	
	z_ref_ = z0 + h_ref;

	//  clamp
	//double z_min = initial_pose_.pose.position.z; // 초기 높이
	//z_ref_ = std::max(z_ref_, z_min);

	// target
	target_pos_vec.z() = z_ref_;    
    
    
    //target_pos_vec.z() = current_position_.z();
    
	//target_pos_vec.z() = initial_pose_.pose.position.z;
// ======================================================== //


    Eigen::Vector3d a_cmd = asmc3d_from_vdes(target_pos_vec, v_des);
    double desired_yaw = target_yaw + M_PI;
    publishAttitudeTargetFromAccel(a_cmd, desired_yaw, hover_thrust_);    
    
// ============== setup for touchdown ===============================


    double vz_now = current_vel_.twist.linear.z; 
    double vx_now = current_vel_.twist.linear.x;
    double vy_now = current_vel_.twist.linear.y;
    static int touchdown_count = 0;
    
    // h_current가 불안정해서 조건문 수정해야  될지도
    //bool low_alt  = (h_current < 0.03);          // check height
    //bool low_vz   = (std::fabs(vz_now) < 0.1);  // check v_z    


	// 조건 수정
    bool low_alt  = (h_current < 0.3);          // check height
    bool low_vz   = (std::fabs(vz_now) < 0.1);  // check v_z    
    bool low_vx   = (std::fabs(vx_now) < 0.05);  // check v_x
    bool low_vy   = (std::fabs(vy_now) < 0.05);  // check v_y


	// ===== DEBUG LOG  =====
	static int td_dbg_cnt = 0;
	td_dbg_cnt++;
	if (td_dbg_cnt % 30 == 0) { 
		RCLCPP_INFO(
		    get_logger(),
		    "[TD DEBUG] cur_z=%.3f tag_z=%.3f h_current=%.3f  vz_now=%.3f  low_alt=%d  low_vz=%d  count=%d",
        	current_position_.z(),
        	target_pos.z(),		    
		    h_current,
		    vz_now,
		    low_alt ? 1 : 0,
		    low_vz ? 1 : 0,
		    touchdown_count
		);
	}



//    if (low_alt && low_vz) {
//        touchdown_count++;
//    } else {
//        touchdown_count = 0;
//    }

    if (low_alt && low_vz && low_vx && low_vy) {
        touchdown_count++;
    } else {
        touchdown_count = 0;
    }



    // condition
    if (touchdown_count > 50) {
        RCLCPP_INFO(get_logger(), "Touchdown detected, sending zero thrust and disarming.");

        // 추력 0, 자세는 현재 값 유지
        Eigen::Vector3d a_zero = Eigen::Vector3d::Zero();
        double yaw_hold = current_yaw; 
        publishAttitudeTargetFromAccel(a_zero, yaw_hold, 0.0);  // thrust=0

        // disarm
        requestDisarm();  

        // change to finish
        switchToState(State::Finished);
        return;
    }



// ==============    end     ==========================================        
    
    
    
}

bool PiTagLandVel::tagDetectedRecently()
{

    rclcpp::Time now_time = clock_->now();
    return (tag_detected_ && (now_time - last_tag_time_).seconds() < timeout_sec_);
}

bool PiTagLandVel::positionReachedXY(const Eigen::Vector3d &target)
{
    Eigen::Vector2d delta = (current_position_ - target).head<2>();
    return delta.norm() < pos_thresh_;
}

Eigen::Vector3d PiTagLandVel::computeVelocityCommand(const Eigen::Vector3d &target)
{
    Eigen::Vector3d error = target - current_position_;
    Eigen::Vector3d vel = p_gain_ * error;
    for (int i = 0; i < 3; ++i)
        vel[i] = clamp(vel[i], -max_vel_, max_vel_);
    return vel;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PiTagLandVel>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
