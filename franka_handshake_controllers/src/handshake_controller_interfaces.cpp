#include <franka_handshake_controllers/handshake_controller.hpp>
#include <franka_handshake_controllers/robot_utils.hpp>
#include <std_msgs/msg/float64.hpp>
#include <cassert>
#include <cmath>
#include <exception>
#include <string>
#include <thread>
#include <Eigen/Eigen>

namespace franka_handshake_controllers
{

    using namespace std::chrono_literals;

    void HandShakeController::freq_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        handshake_tuning_ = msg->data;
        RCLCPP_INFO(get_node()->get_logger(), "Handshake tuning frequency updated to: %f Hz", handshake_tuning_);
    }

    rclcpp_action::GoalResponse HandShakeController::handle_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const franka_handshake_msgs::action::Handshake::Goal> /*goal*/)
    {
        RCLCPP_INFO(get_node()->get_logger(), "Received handshake goal");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse HandShakeController::handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<franka_handshake_msgs::action::Handshake>> /*goal_handle*/)
    {
        RCLCPP_INFO(get_node()->get_logger(), "Handshake goal canceled");
        handshake_active_ = false;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void HandShakeController::handle_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<franka_handshake_msgs::action::Handshake>> goal_handle)
    {
        RCLCPP_INFO(get_node()->get_logger(), "Handshake goal accepted");
        handshake_active_ = true;
        active_goal_handle_ = goal_handle;
        this->handshake_amplitude_ = goal_handle->get_goal()->amplitude;
        this->handshake_base_frequency_ = goal_handle->get_goal()->frequency;
        this->handshake_n_oscillations_ = goal_handle->get_goal()->n_oscillations;
        this->handshake_synchrony_factor_ = goal_handle->get_goal()->synchrony_factor;

        // --- Derived scalar timing used by envelope (same as before) ---
        double omega_base_ = 2.0 * M_PI * (handshake_base_frequency_ + handshake_tuning_);
        if (omega_base_ <= 0.0)
        {
            RCLCPP_WARN(get_node()->get_logger(), "Omega is non-positive, skipping handshake trajectory.");
            return;
        }

        for (int j = 0; j < num_joints; ++j)
        {
            phi_[j] = M_PI / 2.0;
            omega_[j] = omega_base_;
            C_[j] = 0.5 * (Q1_[j] + Q2_[j]);
            A_[j] = 0.5 * (Q2_[j] - Q1_[j]);
            e_t_filt_[j] = 0.0;
            last_q_goal_[j] = C_[j];
        }
    }

    void HandShakeController::handle_action_server_progress(double elapsed_time)
    {
        if (handshake_active_)
        {
            if (handshake_start_time_ == 0.0)
            {
                handshake_start_time_ = elapsed_time;
            }
            double elapsed = elapsed_time - handshake_start_time_;

            double duration = (double)handshake_n_oscillations_ / handshake_base_frequency_ / 2.0;
            if (duration <= 0.0)
            {
                RCLCPP_WARN(get_node()->get_logger(), "Handshake duration is non-positive, aborting.");
                handshake_active_ = false;
                handshake_start_time_ = 0.0;
                active_goal_handle_.reset();
                return;
            }
            double progress = std::clamp(elapsed / duration, 0.0, 1.0);

            auto feedback = std::make_shared<Handshake::Feedback>();
            feedback->progress = progress;
            active_goal_handle_->publish_feedback(feedback);

            if (elapsed >= duration)
            {
                auto result = std::make_shared<Handshake::Result>();
                result->success = true;
                result->message = "Handshake complete";
                active_goal_handle_->succeed(result);
                handshake_active_ = false;
                handshake_start_time_ = 0.0;
                active_goal_handle_.reset();
            }
        }
    }

    void HandShakeController::publish_commanded_pose(double timestamp, const Vector7d &q_goal)
    {
        std_msgs::msg::Float64MultiArray msg;
        msg.data.resize(8);
        msg.data[0] = timestamp;
        for (int i = 1; i < 8; ++i)
        {
            msg.data[i] = q_goal(i);
        }
        this->commanded_pose_pub_->publish(msg);
    }

    void HandShakeController::publish_actual_pose(double timestamp, const Vector7d &q_actual)
    {
        std_msgs::msg::Float64MultiArray msg;
        msg.data.resize(8);
        msg.data[0] = timestamp;
        for (int i = 1; i < 8; ++i)
        {
            msg.data[i] = q_actual(i);
        }
        this->actual_pose_pub_->publish(msg);
    }

    rclcpp_action::GoalResponse HandShakeController::handle_gains_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const SetGains::Goal> goal)
    {
        if (goal->k_gains.size() != 7 || goal->d_gains.size() != 7)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "SetGains: wrong vector size");
            return rclcpp_action::GoalResponse::REJECT;
        }
        if (this->handshake_active_)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Cannot change gains while handshake is active. Wait for handshake to complete.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse HandShakeController::handle_gains_cancel(
        const std::shared_ptr<GoalHandleSetGains>)
    {
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void HandShakeController::handle_gains_accepted(
        const std::shared_ptr<GoalHandleSetGains> goal_handle)
    {
        std::thread{[this, goal_handle]()
                    {
                        const auto goal = goal_handle->get_goal();
                        auto result = std::make_shared<SetGains::Result>();

                        {
                            Eigen::Map<const Eigen::Matrix<double, 7, 1>> k(goal->k_gains.data());
                            Eigen::Map<const Eigen::Matrix<double, 7, 1>> d(goal->d_gains.data());
                            k_target_ = k;
                            d_target_ = d;
                            blend_t_ = 0.0;
                            blending_ = true;
                        }

                        rclcpp::Rate rate(50);
                        while (rclcpp::ok() && blending_)
                        {
                            SetGains::Feedback fb;
                            fb.progress = std::min(1.0, blend_t_ / blend_T_);
                            goal_handle->publish_feedback(std::make_shared<SetGains::Feedback>(fb));
                            rate.sleep();
                        }

                        result->success = true;
                        result->message = "Gains updated";
                        goal_handle->succeed(result);
                    }}
            .detach();
    }
}
