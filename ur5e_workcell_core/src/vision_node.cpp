#include <rclcpp/rclcpp.hpp>
#include <fake_ar_publisher/msg/ar_marker.hpp>
// service interface to get the pose of the localized part
#include <ur5e_workcell_core/srv/localize_part.hpp>
// TF cordinate transforms
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class Localizer : public rclcpp::Node
{
public:     // The transform listener must be constructed using the buffer
    Localizer() : Node("vision_node"), buffer_(this->get_clock()), listener_(buffer_), last_msg_{nullptr}
    {
        ar_sub_ = this->create_subscription<fake_ar_publisher::msg::ARMarker>(
            "ar_pose_marker",
            rclcpp::QoS(1),
            std::bind(&Localizer::visionCallback, this, std::placeholders::_1));
        server_ = this->create_service<ur5e_workcell_core::srv::LocalizePart>(
            "localize_part",
            std::bind(&Localizer::localizePart, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    void visionCallback(fake_ar_publisher::msg::ARMarker::SharedPtr msg)
    {
        last_msg_ = msg;
/*         RCLCPP_INFO(get_logger(), "Received pose: x=%f, y=%f, z=%f",
                    msg->pose.pose.position.x,
                    msg->pose.pose.position.y,
                    msg->pose.pose.position.z); */
    }
    void localizePart(ur5e_workcell_core::srv::LocalizePart::Request::SharedPtr req,
                      ur5e_workcell_core::srv::LocalizePart::Response::SharedPtr res)
    {
        fake_ar_publisher::msg::ARMarker::SharedPtr p = last_msg_;
        if (!p)
        {
            RCLCPP_ERROR(this->get_logger(), "no data");
            res->success = false;
            return;
        }
        // process the incoming request from client
        geometry_msgs::msg::PoseStamped target_pose_from_camera;
        target_pose_from_camera.header = p->header;
        target_pose_from_camera.pose = p->pose.pose;
        // transform to another coordinate frame as per the  client request "base_frame"
        geometry_msgs::msg::PoseStamped target_pose_from_request = buffer_.transform(
            target_pose_from_camera,
            req->base_frame);
        res->pose = target_pose_from_request.pose;
        res->success = true;
    }

    rclcpp::Subscription<fake_ar_publisher::msg::ARMarker>::SharedPtr ar_sub_;
    rclcpp::Service<ur5e_workcell_core::srv::LocalizePart>::SharedPtr server_;

    fake_ar_publisher::msg::ARMarker::SharedPtr last_msg_;
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Localizer>();
    RCLCPP_INFO(node->get_logger(), "Vision node starting");
    rclcpp::spin(node);
}