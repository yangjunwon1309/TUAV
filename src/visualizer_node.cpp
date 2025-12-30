#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "apriltag_msgs/msg/april_tag_detection_array.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"


using std::placeholders::_1;

class VisualizerNode : public rclcpp::Node

{
public:
    VisualizerNode() : Node("visualizer_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
        local_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose",
            rclcpp::SensorDataQoS(),
            std::bind(&VisualizerNode::localPoseCallback, this, _1));
        

        // tag_sub_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
        //     "/apriltag_detections", 10, std::bind(&VisualizerNode::tagCallback, this, _1));
        transformed_tag_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/tag_pose", 10,
            std::bind(&VisualizerNode::transformedTagCallback, this, _1));
        

        origin_timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&VisualizerNode::publishOriginMarker, this));
        
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        

    }

private:
    void localPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    // Publish visualization marker (optional)
    auto marker = createMarker(msg->pose.position, "map", 0, {0.0, 1.0, 0.0});
    marker_pub_->publish(marker);

    // Build the transform
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

    // void tagCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
    // {
    //     for (size_t i = 0; i < msg->detections.size(); ++i)
    //     {
    //         const auto &detection = msg->detections[i];
    //         geometry_msgs::msg::PoseStamped input_pose;
    //         input_pose.header = detection.pose.header;
    //         input_pose.header.frame_id = detection.pose.header.frame_id.empty() ?
    //                           "camera_color_optical_frame" :
    //                           detection.pose.header.frame_id;

    //         input_pose.pose = detection.pose.pose.pose;

    //         try
    //         {
    //             geometry_msgs::msg::TransformStamped transform =
    //                 tf_buffer_.lookupTransform("map", input_pose.header.frame_id, tf2::TimePointZero);

    //             geometry_msgs::msg::PoseStamped transformed_pose;
    //             tf2::doTransform(input_pose, transformed_pose, transform);

    //             auto marker = createMarker(transformed_pose.pose.position, "map", 100 + i, {1.0, 0.0, 0.0});
    //             marker_pub_->publish(marker);
    //         }
    //         catch (const tf2::TransformException &ex)
    //         {
    //             RCLCPP_WARN(this->get_logger(), "Could not transform tag pose: %s", ex.what());
    //         }
    //     }
    // }
    void transformedTagCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        auto marker = createMarker(msg->pose.position, msg->header.frame_id, 200, {1.0, 1.0, 0.0});
        marker_pub_->publish(marker);
    }
    

    void publishOriginMarker()
    {
        geometry_msgs::msg::Point origin;
        origin.x = 0.0;
        origin.y = 0.0;
        origin.z = 0.0;
        auto marker = createMarker(origin, "map", 999, {0.0, 0.0, 1.0});
        marker_pub_->publish(marker);
    }

    visualization_msgs::msg::Marker createMarker(
        const geometry_msgs::msg::Point &position,
        const std::string &frame_id,
        int id,
        const std::array<float, 3> &rgb)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = now();
        marker.ns = "visualizer";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position = position;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.r = rgb[0];
        marker.color.g = rgb[1];
        marker.color.b = rgb[2];
        marker.color.a = 1.0;
        marker.lifetime = rclcpp::Duration::from_seconds(2.0);
        return marker;
    }

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pose_sub_;
    // rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr tag_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr transformed_tag_sub_;

    rclcpp::TimerBase::SharedPtr origin_timer_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;



};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisualizerNode>());
    rclcpp::shutdown();
    return 0;
}
