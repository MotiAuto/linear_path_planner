#ifndef LINEAR_PATH_PLANNER_HPP_
#define LINEAR_PATH_PLANNER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace linear_path_planner
{
    class LinearPathPlanner : public rclcpp::Node
    {
        public:
        explicit LinearPathPlanner(const rclcpp::NodeOptions&option = rclcpp::NodeOptions());

        void current_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void timer_callback();

        private:
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
        rclcpp::TimerBase::SharedPtr timer_;

        geometry_msgs::msg::PoseStamped::SharedPtr current_pose_;
        geometry_msgs::msg::PoseStamped::SharedPtr target_pose_;
        double step_size_param;

        tf2::Vector3 getEuler(const geometry_msgs::msg::Quaternion& q)
        {
            tf2::Quaternion tf2_quat;
            tf2_quat.setW(q.w);
            tf2_quat.setX(q.x);
            tf2_quat.setY(q.y);
            tf2_quat.setZ(q.z);

            tf2::Matrix3x3 mat(tf2_quat);

            double roll, pitch, yaw;
            mat.getRPY(roll, pitch, yaw);

            return tf2::Vector3(roll, pitch, yaw);
        }
    };
}

#endif