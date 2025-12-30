#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/point.hpp>

#include <fstream>
#include <sstream>
#include <vector>
#include <string>
using namespace std::chrono_literals;

struct Waypoint {
  double x, y, z;
};

class WaypointVisualizer : public rclcpp::Node {
public:
  WaypointVisualizer() : Node("waypoint_visualizer") {
    declare_parameter<std::string>("mission_csv", "mission.csv");
    get_parameter("mission_csv", mission_csv_path_);

    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("waypoint_markers", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/mavros/local_position/pose",
        rclcpp::SensorDataQoS(), 
        std::bind(&WaypointVisualizer::pose_callback, this, std::placeholders::_1));

    load_waypoints();
    marker_timer_ = create_wall_timer(1s, std::bind(&WaypointVisualizer::publish_markers, this));
  }

private:
  std::string mission_csv_path_;
  std::vector<Waypoint> waypoints_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr marker_timer_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  void load_waypoints() {
    std::ifstream file(mission_csv_path_);
    if (!file.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open CSV: %s", mission_csv_path_.c_str());
      return;
    }

    std::string line;
    std::getline(file, line); // Skip header
    while (std::getline(file, line)) {
      std::stringstream ss(line);
      std::string token;
      Waypoint wp;
      std::getline(ss, token, ','); wp.x = std::stod(token);
      std::getline(ss, token, ','); wp.y = std::stod(token);
      std::getline(ss, token, ','); wp.z = std::stod(token);
      waypoints_.push_back(wp);
    }

    RCLCPP_INFO(get_logger(), "Loaded %ld waypoints", waypoints_.size());
  }

  void publish_markers() {
    visualization_msgs::msg::MarkerArray marker_array;
    for (size_t i = 0; i < waypoints_.size(); ++i) {
      const auto &wp = waypoints_[i];

      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = this->now();
      marker.ns = "waypoints";
      marker.id = i;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = wp.x;
      marker.pose.position.y = wp.y;
      marker.pose.position.z = wp.z;
      marker.scale.x = 0.3;
      marker.scale.y = 0.3;
      marker.scale.z = 0.3;
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0f;
      marker.lifetime = rclcpp::Duration::from_seconds(0.0);

      marker_array.markers.push_back(marker);
    }

    marker_pub_->publish(marker_array);
  }

  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = msg->header.stamp;
    tf_msg.header.frame_id = msg->header.frame_id;  // usually "map"
    tf_msg.child_frame_id = "base_link";
    tf_msg.transform.translation.x = msg->pose.position.x;
    tf_msg.transform.translation.y = msg->pose.position.y;
    tf_msg.transform.translation.z = msg->pose.position.z;
    tf_msg.transform.rotation = msg->pose.orientation;

    tf_broadcaster_->sendTransform(tf_msg);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WaypointVisualizer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
