#pragma once

#include "user_command_interface.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32.hpp>

namespace interface {

/**
 * Receives velocity and state-transition commands over ROS2 topics.
 *
 * /M20/cmd_vel   (geometry_msgs/Twist)  — velocity while in RL control mode
 *   linear.x  → forward_vel_scale   (m/s)
 *   linear.y  → side_vel_scale      (m/s)
 *   angular.z → turnning_vel_scale  (rad/s)
 *
 * /M20/target_mode  (std_msgs/Int32)  — state machine transitions (replaces z/c keys)
 *   1 → StandUp   (equivalent to pressing 'z')
 *   6 → RLControl (equivalent to pressing 'c')
 *   0 → Idle
 *
 * Safety: if no /M20/cmd_vel is received for > 0.5 s, velocity scales are zeroed.
 */
class ROS2CmdVelInterface : public UserCommandInterface {
public:
    ROS2CmdVelInterface(RobotName robot_name, rclcpp::Node::SharedPtr node)
        : UserCommandInterface(robot_name),
          node_(node),
          last_cmd_time_(node->now())
    {}

    void Start() override {
        cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
            "/M20/cmd_vel", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                usr_cmd_->forward_vel_scale  = static_cast<float>(msg->linear.x);
                usr_cmd_->side_vel_scale     = static_cast<float>(msg->linear.y);
                usr_cmd_->turnning_vel_scale = static_cast<float>(msg->angular.z);
                last_cmd_time_ = node_->now();
            });

        mode_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
            "/M20/target_mode", 10,
            [this](const std_msgs::msg::Int32::SharedPtr msg) {
                usr_cmd_->target_mode = static_cast<uint8_t>(msg->data);
                RCLCPP_INFO(node_->get_logger(),
                            "target_mode set to %d", msg->data);
            });

        RCLCPP_INFO(node_->get_logger(),
                    "ROS2CmdVelInterface ready\n"
                    "  /M20/cmd_vel        — velocity commands\n"
                    "  /M20/target_mode    — 1=StandUp, 6=RLControl");
    }

    void Stop() override {
        cmd_sub_.reset();
        mode_sub_.reset();
    }

    UserCommand* GetUserCommand() override {
        double age = (node_->now() - last_cmd_time_).seconds();
        if (age > 0.5) {
            usr_cmd_->forward_vel_scale  = 0.0f;
            usr_cmd_->side_vel_scale     = 0.0f;
            usr_cmd_->turnning_vel_scale = 0.0f;
        }
        return usr_cmd_;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr       mode_sub_;
    rclcpp::Time last_cmd_time_;
};

} // namespace interface
