#include "linear_path_planner/linear_path_planner.hpp"

namespace linear_path_planner
{
    LinearPathPlanner::LinearPathPlanner(const rclcpp::NodeOptions&option) : Node("LinearPathPlanner", option)
    {
        target_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/target",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&LinearPathPlanner::target_callback, this, _1)
        );

        current_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/current",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&LinearPathPlanner::current_callback, this, _1)
        );

        timer_ = this->create_wall_timer(1ms, std::bind(&LinearPathPlanner::timer_callback, this));

        path_pub = this->create_publisher<nav_msgs::msg::Path>("/path", rclcpp::SystemDefaultsQoS());

        current_pose_ = nullptr;
        target_pose_ = nullptr;

        this->declare_parameter("step_size", 0.1);
        this->get_parameter("step_size", step_size_param);
    }

    void LinearPathPlanner::timer_callback()
    {
        if(current_pose_ != nullptr && target_pose_ != nullptr)
        {
            auto new_path = nav_msgs::msg::Path();
            new_path.header.frame_id = "map";

            const auto target_posture = getEuler(target_pose_->pose.orientation);
            const auto current_posture = getEuler(current_pose_->pose.orientation);

            const auto dx = target_pose_->pose.position.x - current_pose_->pose.position.x;
            const auto dy = target_pose_->pose.position.y - current_pose_->pose.position.y;
            const auto d_rotate = target_posture.getZ() - current_posture.getZ();

            // RCLCPP_INFO(this->get_logger(), "t:%lf, c:%lf", target_posture.getZ(), current_posture.getZ());

            const auto p2p = std::sqrt(dx*dx + dy*dy);
            const auto step_num = p2p / step_size_param;
            auto rad_inc = 0.0;

            if(p2p < step_size_param)
            {
                new_path.poses.push_back(*target_pose_);
            }
            else
            {
                for(int i = 1; i < step_num; i++)
                {
                    auto t = static_cast<double>(i) / step_num;
                    auto p = geometry_msgs::msg::PoseStamped();
                    p.pose.position.x = current_pose_->pose.position.x + t * dx;
                    p.pose.position.y = current_pose_->pose.position.y + t * dy;
                    tf2::Quaternion q;
                    rad_inc += d_rotate / step_num;
                    q.setRPY(0.0, 0.0, current_posture.getZ() + rad_inc);

                    p.pose.orientation.w = q.w();
                    p.pose.orientation.x = q.x();
                    p.pose.orientation.y = q.y();
                    p.pose.orientation.z = q.z();
                    
                    new_path.poses.push_back(p);
                }
            }
            new_path.poses.push_back(*current_pose_);
            new_path.poses.push_back(*target_pose_);

            current_pose_ = nullptr;

            path_pub->publish(new_path);
        }
    }

    void LinearPathPlanner::target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        target_pose_ = msg;
    }

    void LinearPathPlanner::current_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_pose_ = msg;
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(linear_path_planner::LinearPathPlanner)
