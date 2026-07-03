#pragma once

#include "user_command_interface.h"
#include "custom_types.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <thread>
#include <atomic>
#include <mutex>

using namespace interface;
using namespace types;

/**
 * Reads /cmd_vel (TwistStamped or Twist) from the CMU autonomy stack's
 * local_planner and feeds it into the M20 rl_deploy state machine as
 * forward_vel_scale / side_vel_scale / turnning_vel_scale.
 *
 * Mode keys are still handled here so the operator can stand the robot
 * up before handing off to the autonomy stack:
 *   R → joint damping
 *   Z → stand up
 *   C → RL control mode (autonomy takes over via /cmd_vel)
 */
class Ros2CmdVelInterface : public UserCommandInterface
{
public:
    // Scale factors: cmd_vel linear.x is in m/s, scale to [0,1]
    float max_forward_  = 1.0f;
    float max_side_     = 1.0f;
    float max_yaw_      = 2.0f;
    float max_cmd_scale_ = 0.7f;
    double control_settle_sec_ = 2.0;
    double raw_override_sec_ = 0.5;

    explicit Ros2CmdVelInterface(RobotName robot_name)
        : UserCommandInterface(robot_name)
    {
        std::memset(usr_cmd_, 0, sizeof(UserCommand));
    }

    ~Ros2CmdVelInterface() { Stop(); }

    void Start() override
    {
        if (running_) return;
        running_ = true;
        spin_thread_ = std::thread([this]() {
            node_ = rclcpp::Node::make_shared("m20_cmdvel_interface");
            debug_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>(
                "/m20_cmdvel_debug", 10);

            // Accept both TwistStamped (CMU local_planner) and plain Twist
            sub_stamped_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
                "/cmd_vel", 10,
                [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
                    std::lock_guard<std::mutex> lk(mtx_);
                    if (node_->now().seconds() < raw_override_until_) return;
                    const bool accepting_cmd = command_enabled_();
                    usr_cmd_->forward_vel_scale = accepting_cmd
                        ? clip(msg->twist.linear.x / max_forward_, -max_cmd_scale_, max_cmd_scale_) : 0.f;
                    usr_cmd_->side_vel_scale = accepting_cmd
                        ? clip(msg->twist.linear.y / max_side_, -max_cmd_scale_, max_cmd_scale_) : 0.f;
                    usr_cmd_->turnning_vel_scale = accepting_cmd
                        ? clip(msg->twist.angular.z / max_yaw_, -max_cmd_scale_, max_cmd_scale_) : 0.f;
                    usr_cmd_->time_stamp = node_->now().seconds();
                    publish_debug_(0.f);
                });

            sub_twist_ = node_->create_subscription<geometry_msgs::msg::Twist>(
                "/cmd_vel_raw", 10,
                [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                    std::lock_guard<std::mutex> lk(mtx_);
                    raw_override_until_ = node_->now().seconds() + raw_override_sec_;
                    const bool accepting_cmd = command_enabled_();
                    usr_cmd_->forward_vel_scale = accepting_cmd
                        ? clip(msg->linear.x / max_forward_, -max_cmd_scale_, max_cmd_scale_) : 0.f;
                    usr_cmd_->side_vel_scale = accepting_cmd
                        ? clip(msg->linear.y / max_side_, -max_cmd_scale_, max_cmd_scale_) : 0.f;
                    usr_cmd_->turnning_vel_scale = accepting_cmd
                        ? clip(msg->angular.z / max_yaw_, -max_cmd_scale_, max_cmd_scale_) : 0.f;
                    usr_cmd_->time_stamp = node_->now().seconds();
                    publish_debug_(1.f);
                });

            // Mode commands: publish to this topic to change state
            // e.g. `ros2 topic pub /m20_mode std_msgs/msg/String "data: stand"`
            sub_mode_ = node_->create_subscription<std_msgs::msg::String>(
                "/m20_mode", 10,
                [this](const std_msgs::msg::String::SharedPtr msg) {
                    if (msg->data == "damping") {
                        usr_cmd_->target_mode = uint8_t(RobotMotionState::JointDamping);
                        RCLCPP_INFO(node_->get_logger(), "[MODE] Joint Damping");
                    } else if (msg->data == "stand") {
                        if (msfb_ && msfb_->GetCurrentState() != RobotMotionState::WaitingForStand) {
                            RCLCPP_WARN(node_->get_logger(), "[MODE] Ignoring stand: current state is not WaitingForStand");
                            return;
                        }
                        usr_cmd_->target_mode = uint8_t(RobotMotionState::StandingUp);
                        RCLCPP_INFO(node_->get_logger(), "[MODE] Standing Up");
                    } else if (msg->data == "control") {
                        if (msfb_ && msfb_->GetCurrentState() != RobotMotionState::StandingUp) {
                            RCLCPP_WARN(node_->get_logger(), "[MODE] Ignoring control: current state is not StandingUp");
                            return;
                        }
                        usr_cmd_->forward_vel_scale = 0.f;
                        usr_cmd_->side_vel_scale = 0.f;
                        usr_cmd_->turnning_vel_scale = 0.f;
                        control_enable_time_ = node_->now().seconds() + control_settle_sec_;
                        usr_cmd_->target_mode = uint8_t(RobotMotionState::RLControlMode);
                        RCLCPP_INFO(node_->get_logger(),
                            "[MODE] RL Control (autonomy active after %.1fs settle)",
                            control_settle_sec_);
                    }
                });

            RCLCPP_INFO(node_->get_logger(),
                "Ros2CmdVelInterface ready. Topics: /cmd_vel, /m20_mode");

            rclcpp::spin(node_);
        });
    }

    void Stop() override
    {
        running_ = false;
        if (node_) rclcpp::shutdown();
        if (spin_thread_.joinable()) spin_thread_.join();
        usr_cmd_->forward_vel_scale = usr_cmd_->side_vel_scale = usr_cmd_->turnning_vel_scale = 0.f;
    }

    UserCommand* GetUserCommand() override { return usr_cmd_; }

private:
    std::atomic<bool> running_{false};
    std::thread spin_thread_;
    std::mutex mtx_;
    double control_enable_time_{0.0};

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_stamped_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr         sub_twist_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr             sub_mode_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr     debug_pub_;
    double raw_override_until_{0.0};

    static float clip(float v, float lo, float hi)
    { return v < lo ? lo : (v > hi ? hi : v); }

    bool command_enabled_() const
    {
        return node_ &&
               node_->now().seconds() >= control_enable_time_ &&
               msfb_ &&
               msfb_->GetCurrentState() == RobotMotionState::RLControlMode;
    }

    void publish_debug_(float source)
    {
        if (!debug_pub_) return;
        std_msgs::msg::Float32MultiArray msg;
        msg.data = {
            source,
            usr_cmd_->forward_vel_scale,
            usr_cmd_->side_vel_scale,
            usr_cmd_->turnning_vel_scale,
            static_cast<float>(usr_cmd_->target_mode),
            static_cast<float>(usr_cmd_->safe_control_mode),
            msfb_ ? static_cast<float>(msfb_->GetCurrentState()) : -1.f,
            msfb_ ? static_cast<float>(msfb_->GetCurrentGait()) : -1.f,
        };
        debug_pub_->publish(msg);
    }
};
