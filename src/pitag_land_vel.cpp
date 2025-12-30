
#include "waypoint_velocity_tracker/pitag_land_vel.h"
#include <fstream>
#include <sstream>
#include <thread>

//added

void PiTagLandVel::addTagPosition(const Eigen::Vector3d& pos) {
    // Compute current average
    Eigen::Vector3d mean = getSmoothedTagPosition();

    // Reject if distance is too far from average
    if (!tag_position_window_.empty()) {
        double dist = (pos - mean).norm();
        if (dist > 0.5) {  // reject outlier (>0.5m from average)
            RCLCPP_WARN(get_logger(), "Rejected tag detection: %.2f m from mean", dist);
            return;
        }
    }

    // Add to window
    tag_position_window_.push_back(pos);
    if (tag_position_window_.size() > window_size_) {
        tag_position_window_.pop_front();
    }
}

Eigen::Vector3d PiTagLandVel::getSmoothedTagPosition() const {
    if (tag_position_window_.empty()) {
        return latest_tag_position_;  // fallback
    }

    Eigen::Vector3d sum(0.0, 0.0, 0.0);
    for (const auto& pos : tag_position_window_) {
        sum += pos;
    }
    return sum / static_cast<double>(tag_position_window_.size());
}

using namespace std::chrono_literals;

template <typename T>
T clamp(const T& v, const T& lo, const T& hi)
{
    return std::min(std::max(v, lo), hi);
}

PiTagLandVel::PiTagLandVel()
    : Node("apriltag_land_vel"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
{
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    declare_parameter("p_gain", 1.0);
    declare_parameter("max_vel", 0.5);
    declare_parameter("descent_vel", 0.1);
    declare_parameter("pos_thresh", 0.1);
    declare_parameter("timeout_sec", 2.0);
    declare_parameter("takeoff_height", 1.5);
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
            
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
    latest_tag_position_map_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/latest_tag_position_map", 10);

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
    
    RCLCPP_INFO(this->get_logger(), "PiTagLandVel node initialized.");
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

  
      for (int i = 0; i < 10; ++i) {
        geometry_msgs::msg::TwistStamped dummy;
        dummy.header.stamp = this->now();
        dummy.twist.linear.x = 0.0;
        dummy.twist.linear.y = 0.0;
        dummy.twist.linear.z = 0.0;
        cmd_vel_pub_->publish(dummy);
        // rclcpp::sleep_for(10ms); 
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

    // 선택적으로 결과 확인 (논블로킹 방식)
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
    tag_x_ = msg->detections[0].pose.pose.pose.position.x;
    tag_y_ = msg->detections[0].pose.pose.pose.position.y;

    auto pose_with_cov = msg->detections[0].pose;
    try {
        //transform camera frame -> map frame 
        geometry_msgs::msg::PoseStamped ps_in, ps_out;
        ps_in.header = msg->header;
        ps_in.pose = pose_with_cov.pose.pose;

        // "map" 프레임 기준으로 변환
        auto tf = tf_buffer_.lookupTransform("map", ps_in.header.frame_id, tf2::TimePointZero);
        tf2::doTransform(ps_in, ps_out, tf);

        // 변환된 좌표 저장
        latest_tag_position_ = Eigen::Vector3d(
            ps_out.pose.position.x,
            ps_out.pose.position.y,
            ps_out.pose.position.z);
        // RCLCPP_INFO(get_logger(), "Detected tag at (%.2f, %.2f, %.2f) in map frame",
        //             latest_tag_position_.x(), latest_tag_position_.y(), latest_tag_position_.z());
        latest_tag_position_map_pub_->publish(ps_out);

        // /////////////////////////////////////////////////////////////////////////
        // //publish relative position of drone to tag (assume tag is 0,0,0)

        // //get static transform: camera -> drone base_link
        // geometry_msgs::msg::TransformStamped camera_to_base_link=tf_buffer_.lookupTransform(
        //     "x500_mono_cam_0/mono_cam/base_link/imager", // camera frame
        //     "base_link",                                 // base_link
        //     tf2::TimePointZero);

        // //tag position in camera frame is detected
        // geometry_msgs::msg::PoseStamped tag_in_cam_pose;
        // tag_in_cam_pose.header = msg->detections[0].pose.header;
        // tag_in_cam_pose.pose = msg->detections[0].pose.pose.pose;

        // // Convert to transform for multiplication
        // geometry_msgs::msg::TransformStamped tag_to_cam;
        // tag_to_cam.header = tag_in_cam_pose.header;
        // tag_to_cam.child_frame_id = "camera";
        // tag_to_cam.transform.translation.x = tag_in_cam_pose.pose.position.x;
        // tag_to_cam.transform.translation.y = tag_in_cam_pose.pose.position.y;
        // tag_to_cam.transform.translation.z = tag_in_cam_pose.pose.position.z;
        // tag_to_cam.transform.rotation = tag_in_cam_pose.pose.orientation;

        // // Convert both to tf2 and multiply: tag->camera × camera->base_link
        // tf2::Transform tf_tag_cam, tf_cam_base;
        // tf2::fromMsg(tag_to_cam.transform, tf_tag_cam);
        // tf2::fromMsg(camera_to_base_link.transform, tf_cam_base);

        // tf2::Transform tf_tag_base = tf_tag_cam * tf_cam_base;

        // // Publish result as PoseStamped
        // geometry_msgs::msg::PoseStamped rel_pose;
        // rel_pose.header.stamp = this->now();
        // rel_pose.header.frame_id = "tag"; // origin at tag
        // // Extract translation
        // rel_pose.pose.position.x = tf_tag_base.getOrigin().x();
        // rel_pose.pose.position.y = tf_tag_base.getOrigin().y();
        // rel_pose.pose.position.z = tf_tag_base.getOrigin().z();

        // // Extract rotation
        // tf2::Quaternion q = tf_tag_base.getRotation();
        // rel_pose.pose.orientation = tf2::toMsg(q);

        // drone_pose_from_tag_pub_->publish(rel_pose);




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

void PiTagLandVel::handleTakeoff()
{
    if (!takeoff_origin_set_) {
        takeoff_origin_ = current_position_;
        takeoff_origin_set_ = true;
        RCLCPP_INFO(get_logger(), "Takeoff origin x=%.2f, y=%.2f, z=%.2f",takeoff_origin_.x(),takeoff_origin_.y(),takeoff_origin_.z());
    }

    Eigen::Vector3d target = takeoff_origin_;
    target.z() += takeoff_height_;

    // 현재 위치와의 오차
    Eigen::Vector3d error = target - current_position_;
    double max_vel_takeoff_xy = 0.1; // 최대 속도 설정
    double vx = clamp(p_gain_ * error.x(), -max_vel_takeoff_xy, max_vel_takeoff_xy);
    double vy = clamp(p_gain_ * error.y(), -max_vel_takeoff_xy, max_vel_takeoff_xy);
    double vz = clamp(p_gain_ * error.z(), -max_vel_takeoff_xy, max_vel_takeoff_xy);
    if(vz>0 && vz<0.5){
        vz=0.5;
    }

    //try to control yaw=0
    double target_yaw_rad =0;
    tf2::Quaternion q(
        current_orientation_.x(),
        current_orientation_.y(),
        current_orientation_.z(),
        current_orientation_.w()
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, current_yaw;
    m.getRPY(roll, pitch, current_yaw);
    // yaw error 계산 및 정규화
    double yaw_err = target_yaw_rad - current_yaw;
    while (yaw_err > M_PI) yaw_err -= 2 * M_PI;
    while (yaw_err < -M_PI) yaw_err += 2 * M_PI;
    // yaw 속도 명령 생성
    double yaw_rate = clamp(yaw_gain_ * yaw_err, -0.3, 0.3);
    
    // 이륙 완료 조건은 z축 오차만 고려
    if (std::abs(error.z()) < takeoff_thresh_) {
        RCLCPP_INFO(get_logger(), "Takeoff complete → switching to Search");
        switchToState(State::Search);
        return;
    }

    publishVelocitySetpoint(vx, vy, vz, yaw_rate);
    RCLCPP_INFO(get_logger(), "Takeoff: vx=%.2f, vy=%.2f, vz=%.2f, yaw_rate=%.2f",
                vx, vy, vz, yaw_rate);
}

void PiTagLandVel::handleSearch()
{
    // if (tagDetectedRecently()) {
    //     switchToState(State::Approach);
    //     return;
    // }
    // double target_x= search_origin_.x() + 5.0;
    // double target_y= search_origin_.y();
    // double target_z = takeoff_origin_.z() + takeoff_height_;
    if(current_index_ >= waypoints_.size()) {
        RCLCPP_INFO(get_logger(), "All waypoints processed, switching to Finished state.");
        switchToState(State::Approach);
        return;
    }

    double target_x= initial_pose_.pose.position.x + waypoints_[current_index_].x;
    double target_y= initial_pose_.pose.position.y+waypoints_[current_index_].y;
    double target_z = initial_pose_.pose.position.z+ waypoints_[current_index_].z;
    double waypoint_rad= waypoints_[current_index_].yaw_deg * M_PI / 180.0; // Convert to radians
    double target_yaw_rad = initial_pose_.pose.orientation.z +waypoint_rad;


    double dx = target_x - current_position_.x();
    double vx = clamp(p_gain_ * dx, -search_step_, search_step_);
    
    double dy = target_y - current_position_.y();
    double vy = clamp(p_gain_ * dy, -search_step_, search_step_);

    double dz = target_z - current_position_.z();
    double vz = clamp(p_gain_ * dz, -search_step_, search_step_);

     //yaw control
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
 
     // yaw error 계산 및 정규화
     double yaw_err = target_yaw_rad - current_yaw;
     while (yaw_err > M_PI) yaw_err -= 2 * M_PI;
     while (yaw_err < -M_PI) yaw_err += 2 * M_PI;

     //check if we are close enough to the target waypoint and yaw is aligned
     if (positionReachedXY(Eigen::Vector3d(target_x, target_y, target_z)) && std::abs(yaw_err) < 0.1) {
         RCLCPP_INFO(get_logger(), "Reached waypoint %zu: (%.2f, %.2f, %.2f)", current_index_, target_x, target_y, target_z);
         current_index_++;
        //  std::this_thread::sleep_for(800ms); // wait for 0.8 seconds before moving to next waypoint
         if (current_index_ >= waypoints_.size()) {
             RCLCPP_INFO(get_logger(), "All waypoints processed, switching to Finished state.");
             switchToState(State::Approach);
             return;
         }
         return;
     }
 
     // yaw 속도 명령 생성
     double yaw_rate = clamp(yaw_gain_ * yaw_err, -0.35, 0.35); 

    publishVelocitySetpoint(vx, vy, vz, yaw_rate);
}

void PiTagLandVel::handleApproach()
{
    //function to approach the tag - try to stop when tag is in center of camera frame
    double max_approach_vel_=0.3;
    
    //compute dx,dy,dz
    double dx = latest_tag_position_.x() - current_position_.x();
    double dy = latest_tag_position_.y() - current_position_.y();
    double target_z = takeoff_origin_.z() + takeoff_height_;
    double dz = target_z - current_position_.z();

    //compute velocity commands
    double vx = clamp(p_gain_ * dx, -max_approach_vel_, max_approach_vel_);
    double vy = clamp(p_gain_ * dy, -max_approach_vel_, max_approach_vel_);
    double vz = clamp(p_gain_ * dz, -max_vel_, max_vel_);
    // RCLCPP_INFO(get_logger(), "vel command sent (%.2f, %.2f)", vx, vy);
    
    //yaw control
    double target_yaw = 0.0;
    tf2::Quaternion q(
        current_orientation_.x(),
        current_orientation_.y(),
        current_orientation_.z(),
        current_orientation_.w()
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, current_yaw;
    m.getRPY(roll, pitch, current_yaw);

    // yaw error 계산 및 정규화
    double yaw_err = target_yaw - current_yaw;
    while (yaw_err > M_PI) yaw_err -= 2 * M_PI;
    while (yaw_err < -M_PI) yaw_err += 2 * M_PI;

    // yaw 속도 명령 생성
    double yaw_rate = clamp(yaw_gain_ * yaw_err, -0.2, 0.2); 

    static rclcpp::Time reached_time;
    static bool reached = false;
    publishVelocitySetpoint(vx, vy, vz, yaw_rate);
    if (std::abs(tag_x_) < pos_thresh_ && std::abs(tag_y_) < pos_thresh_ && !reached) {
        RCLCPP_INFO(get_logger(), "Approach complete → switching to Descend");
        RCLCPP_INFO(get_logger(), "Landing origin set to current position.");
        landing_origin_ = current_position_;
        // sleep(1); // wait for 2 seconds before switching to Descend state
        reached = true;
        reached_time = this->now();
        // switchToState(State::Descend);
    }
    else if (reached && (this->now() - reached_time).seconds() > 5.0) {
        RCLCPP_INFO(get_logger(), "Approach timed out, switching to Descend");
        landing_origin_ = current_position_;
        switchToState(State::Descend);
    }

}

/*
void PiTagLandVel::handleDescend()
{
    Eigen::Vector3d vel_cmd = computeVelocityCommand(landing_origin_);
    //yaw control
    double target_yaw = 0.0;
    tf2::Quaternion q(
        current_orientation_.x(),
        current_orientation_.y(),
        current_orientation_.z(),
        current_orientation_.w()
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, current_yaw;
    m.getRPY(roll, pitch, current_yaw);

    // yaw error 계산 및 정규화
    double yaw_err = target_yaw - current_yaw;
    while (yaw_err > M_PI) yaw_err -= 2 * M_PI;
    while (yaw_err < -M_PI) yaw_err += 2 * M_PI;

    // yaw 속도 명령 생성
    double yaw_rate = clamp(yaw_gain_ * yaw_err, -0.2, 0.2);
    if(current_position_.z()>1.2){
        publishVelocitySetpoint(vel_cmd.x(), vel_cmd.y(), -0.25, yaw_rate);
    }
    else{
        publishVelocitySetpoint(vel_cmd.x(), vel_cmd.y(), -std::abs(descent_vel_), yaw_rate);
    }
    
    
   
}
*/
void PiTagLandVel::handleDescend()
{
    // bool tag_detected = tagDetectedRecently();

    // If no tag detected, descend straight down slowly regardless of state
    // if (!tag_detected) {
    //     // RCLCPP_WARN(get_logger(), "Tag lost. Descending straight down slowly...");
    //     double vz = final_descend_started_ ? -0.3 : -0.2;
    //     publishVelocitySetpoint(0.0, 0.0, vz, 0.0);
    //     return;
    // }


    // Target position is fixed landing_origin_
    // Eigen::Vector3d target_pos = landing_origin_;
    Eigen::Vector3d target_pos;
    target_pos = getSmoothedTagPosition();  // smooth tracking while descending
    if (current_position_.z()-takeoff_origin_.z() < 0.5) 
    {

    // //try auto landing of px4
    // auto land_req = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
    // land_req->command = 21;  // MAV_CMD_NAV_LAND
    // land_req->confirmation = true;
    // land_req->param1 = 0.0;  // (optional landing parameters)
    // if (!command_long_client_->wait_for_service(1s)) {
    //     RCLCPP_ERROR(this->get_logger(), "LAND service not available!");
    //     return;
    // }
    // RCLCPP_INFO(this->get_logger(), "Sending LAND command to PX4...");    
    // command_long_client_->async_send_request(land_req,
    //     [](rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedFuture result) {
    //         if (result.get()->success) {
    //             RCLCPP_INFO(rclcpp::get_logger("LAND_CLIENT"), "Land command sent successfully.");
    //         } else {
    //             RCLCPP_WARN(rclcpp::get_logger("LAND_CLIENT"), "Failed to send land command.");
    //         }
    //     });

    } 

        
    // Calculate error vector
    Eigen::Vector3d error = target_pos - current_position_;


    // P control for velocity commands
    Eigen::Vector3d vel_cmd = Kp_.cwiseProduct(error);

    // Clamp horizontal velocities to max_vel_
    vel_cmd.x() = clamp(vel_cmd.x(), -0.1, 0.1);
    vel_cmd.y() = clamp(vel_cmd.y(), -0.1, 0.1);
    vel_cmd.z() = -0.4;

    // --- Yaw control ---
    double target_yaw = 0.0;

    tf2::Quaternion q(
        current_orientation_.x(),
        current_orientation_.y(),
        current_orientation_.z(),
        current_orientation_.w()
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, current_yaw;
    m.getRPY(roll, pitch, current_yaw);

    double yaw_err = target_yaw - current_yaw;
    while (yaw_err > M_PI) yaw_err -= 2 * M_PI;
    while (yaw_err < -M_PI) yaw_err += 2 * M_PI;

    double yaw_rate = clamp(yaw_gain_ * yaw_err, -0.3, 0.3);

    // // Log current command
    // RCLCPP_INFO(get_logger(), "Landing: vx=%.2f vy=%.2f vz=%.2f yaw_rate=%.2f", 
    //             vel_cmd.x(), vel_cmd.y(), vel_cmd.z(), yaw_rate);

    // Publish the velocity setpoint
    publishVelocitySetpoint(vel_cmd.x(), vel_cmd.y(), vel_cmd.z(), yaw_rate);
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