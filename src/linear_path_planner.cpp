#include "linear_path_planner/linear_path_planner.hpp"

namespace linear_path_planner
{
    LinearPathPlanner::LinearPathPlanner(const rclcpp::NodeOptions&option) : Node("LinearPathPlanner", option)
    {
        target_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/target",
            0,
            std::bind(&LinearPathPlanner::target_callback, this, _1)
        );

        current_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/current",
            0,
            std::bind(&LinearPathPlanner::current_callback, this, _1)
        );

        path_pub = this->create_publisher<nav_msgs::msg::Path>("/path", 0);

        current_pose_ = nullptr;

        this->declare_parameter("step_size", 0.1);
        this->get_parameter("step_size", step_size_param);
    }

    void LinearPathPlanner::target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if(current_pose_ != nullptr)
        {
            auto new_path = nav_msgs::msg::Path();
            new_path.header.frame_id = "map";

            auto target_posture = getEuler(msg->pose.orientation);
            auto current_posture = getEuler(current_pose_->pose.orientation);

            auto dx = msg->pose.position.x - current_pose_->pose.position.x;
            auto dy = msg->pose.position.y - current_pose_->pose.position.y;
            auto d_rotate = target_posture.getZ() - current_posture.getZ();

            auto p2p = std::sqrt(dx*dx + dy*dy);
            auto step_num = p2p / step_size_param;

            for(int i = 0; i < step_num; i++)
            {
                auto t = static_cast<double>(i) / step_num;
                auto p = geometry_msgs::msg::PoseStamped();
                p.pose.position.x = current_pose_->pose.position.x + t * dx;
                p.pose.position.y = current_pose_->pose.position.y + t * dy;
                tf2::Quaternion q;
                q.setRPY(0.0, 0.0, current_posture.getZ() + t * d_rotate);

                p.pose.orientation.w = q.w();
                p.pose.orientation.x = q.x();
                p.pose.orientation.y = q.y();
                p.pose.orientation.z = q.z();
                
                new_path.poses.push_back(p);
            }

            path_pub->publish(new_path);
        }
    }

    void LinearPathPlanner::current_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_pose_ = msg;
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(linear_path_planner::LinearPathPlanner)