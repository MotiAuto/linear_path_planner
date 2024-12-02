#ifndef LINEAR_PATH_PLANNER_HPP_
#define LINEAR_PATH_PLANNER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

using std::placeholders::_1;

namespace linear_path_planner
{
    class LinearPathPlanner : public rclcpp::Node
    {
        public:
        explicit LinearPathPlanner(const rclcpp::NodeOptions&option = rclcpp::NodeOptions());

        void current_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

        private:
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;

        geometry_msgs::msg::PoseStamped::SharedPtr current_pose_;
        double step_size_param;

        tf2::Vector3 getEuler(const geometry_msgs::msg::Quaternion q)
        {
            tf2::Vector3 v;
            v.setW(q.w);
            v.setX(q.x);
            v.setY(q.y);
            v.setZ(q.z);

            return v;
        }
    };
}

#endif