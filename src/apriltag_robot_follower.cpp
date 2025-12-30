
#include "waypoint_velocity_tracker/apriltag_robot_follower.h"

using namespace std::chrono_literals;
template <typename T>
T clamp(const T& v, const T& lo, const T& hi)
{
    return std::min(std::max(v, lo), hi);
}

AprilTagRobotFollow::AprilTagRobotFollow()
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
    declare_parameter("search_step", 0.4);
    declare_parameter("landing_origin", std::vector<double>{0.0, 0.0, 0.0});
    declare_parameter("yaw_gain", 0.5);
    declare_parameter("approach_thresh", 2.0);

    get_parameter("p_gain", p_gain_);
    get_parameter("max_vel", max_vel_);
    get_parameter("descent_vel", descent_vel_);
    get_parameter("pos_thresh", pos_thresh_);
    get_parameter("timeout_sec", timeout_sec_);
    get_parameter("takeoff_height", takeoff_height_);
    get_parameter("takeoff_thresh", takeoff_thresh_);
    get_parameter("search_step", search_step_);
    get_parameter("yaw_gain", yaw_gain_);
    get_parameter("approach_thresh", approach_thresh_);

    tag_sub_ = create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
        "/apriltag_detections",rclcpp::SensorDataQoS(), std::bind(&AprilTagRobotFollow::tagCallback, this, std::placeholders::_1));

    local_position_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/mavros/local_position/pose", rclcpp::SensorDataQoS(),
        std::bind(&AprilTagRobotFollow::poseCallback, this, std::placeholders::_1));
            
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);

    timer_ = create_wall_timer(20ms, std::bind(&AprilTagRobotFollow::timerCallback, this));
    arming_cli_ = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    rclcpp::QoS state_qos(10);
    state_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
        "/mavros/state", state_qos,
        std::bind(&AprilTagRobotFollow::state_callback, this, std::placeholders::_1));

    clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);

    RCLCPP_INFO(this->get_logger(), "AprilTagRobotFollow node initialized.");
}

void AprilTagRobotFollow::state_callback(const mavros_msgs::msg::State::SharedPtr msg) {
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
        rclcpp::sleep_for(10ms); 
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
  }

void AprilTagRobotFollow::switchToState(State new_state)
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

void AprilTagRobotFollow::tryArmingViaMavros()
{
    if (!arming_cli_->wait_for_service(1s)) {
        RCLCPP_WARN(get_logger(), "Arming service not available (timeout).");
        return;
    }

    auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    req->value = true;

    auto future = arming_cli_->async_send_request(req);

    // 선택적으로 결과 확인 (논블로킹 방식)
    rclcpp::sleep_for(100ms);
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

void AprilTagRobotFollow::publishVelocitySetpoint(double vx, double vy, double vz, double yaw)
{
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = this->now();
    msg.twist.linear.x = vx;
    msg.twist.linear.y = vy;
    msg.twist.linear.z = vz;
    
    msg.twist.angular.z = yaw;
    cmd_vel_pub_->publish(msg);
}

void AprilTagRobotFollow::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
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

void AprilTagRobotFollow::tagCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
{
    if (msg->detections.empty()) {
        tag_detected_ = false;
        return;
    }
    tag_detected_ = true;
    // last_tag_time_ = now();
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

        latest_tag_orientation_map = Eigen::Vector4d(
            ps_out.pose.orientation.x,
            ps_out.pose.orientation.y,
            ps_out.pose.orientation.z,
            ps_out.pose.orientation.w);
        latest_tag_orientation_quat_map_ = Eigen::Quaterniond(
            ps_out.pose.orientation.w,
            ps_out.pose.orientation.x,
            ps_out.pose.orientation.y,
            ps_out.pose.orientation.z);


    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(get_logger(), "TF transform failed: %s", ex.what());
        tag_detected_ = false;
    }
}

void AprilTagRobotFollow::timerCallback()
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

void AprilTagRobotFollow::handleTakeoff()
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

    double vx = clamp(p_gain_ * error.x(), -max_vel_, max_vel_);
    double vy = clamp(p_gain_ * error.y(), -max_vel_, max_vel_);
    double vz = clamp(p_gain_ * error.z(), -max_vel_, max_vel_);
    if(vz>0 && vz<0.35){
        vz=0.35;
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

void AprilTagRobotFollow::handleSearch()
{
    if (tagDetectedRecently()) {
        switchToState(State::Approach);
        return;
    }
    
    double vz = 0.5;

    publishVelocitySetpoint(0, 0, vz, 0);
}

void AprilTagRobotFollow::handleApproach()
{
    if(!tagDetectedRecently()) {
        RCLCPP_WARN(get_logger(), "No tag detected, switching to Search state.");
        switchToState(State::Search);
        return;
    }
    // If tag is detected, we will approach it
    
    //function to approach the tag - try to stop when tag is in center of camera frame
    double max_approach_vel_=0.3;
    
    //compute dx,dy,dz
    double dx = latest_tag_position_.x() - current_position_.x();
    double dy = latest_tag_position_.y() - current_position_.y();
    double target_z = takeoff_origin_.z() + takeoff_height_;
    double dz = target_z - current_position_.z();

    Eigen::Vector2d delta(dx, dy);
    if(delta.norm() > approach_thresh_){
        dz = 0.0; // stop descending
        RCLCPP_INFO(get_logger(), "Tag is %.2f m away, stopping descend.", delta.norm());
    } 

    //compute velocity commands
    double vx = clamp(p_gain_ * dx, -max_approach_vel_, max_approach_vel_);
    double vy = clamp(p_gain_ * dy, -max_approach_vel_, max_approach_vel_);
    double vz = clamp(p_gain_ * dz, -max_vel_, max_vel_);
    // RCLCPP_INFO(get_logger(), "vel command sent (%.2f, %.2f)", vx, vy);
    
    
    double tag_yaw = std::atan2(dy, dx); // Use atan2 to compute the yaw angle based on dx and dy
    // RCLCPP_INFO(get_logger(), " Tag yaw: %.2f", tag_yaw);

    
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
    double yaw_err = tag_yaw - current_yaw;
    while (yaw_err > M_PI) yaw_err -= 2 * M_PI;
    while (yaw_err < -M_PI) yaw_err += 2 * M_PI;

    // yaw 속도 명령 생성
    double yaw_rate = clamp(yaw_gain_ * yaw_err, -0.3, 0.3); 

    if (std::abs(tag_x_) < pos_thresh_ && std::abs(tag_y_) < pos_thresh_) {
        RCLCPP_INFO(get_logger(), "Approach complete make yaw=0.0");
        //yaw control
        yaw_err = - current_yaw;
        yaw_rate = clamp(yaw_gain_ * yaw_err, -0.3, 0.3);
        
    }
    publishVelocitySetpoint(vx, vy, vz, yaw_rate);


}

void AprilTagRobotFollow::handleDescend()
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

bool AprilTagRobotFollow::tagDetectedRecently()
{
    if(!tag_detected_) {
        rclcpp::Time now_time = clock_->now();
        RCLCPP_INFO(get_logger(), "Tag lost for: %f", (now_time - last_tag_time_).seconds());
        if ((now_time - last_tag_time_).seconds() > timeout_sec_) {
            RCLCPP_WARN(get_logger(), "Tag not detected for too long, switching to Search state.");
            return false;
        }

    }
    
    return true;
}


bool AprilTagRobotFollow::positionReachedXY(const Eigen::Vector3d &target)
{
    Eigen::Vector2d delta = (current_position_ - target).head<2>();
    return delta.norm() < pos_thresh_;
}

Eigen::Vector3d AprilTagRobotFollow::computeVelocityCommand(const Eigen::Vector3d &target)
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
    auto node = std::make_shared<AprilTagRobotFollow>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}