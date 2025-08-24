// Copyright (c) 2023 Franka Robotics GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <string>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "franka_handshake_msgs/action/handshake.hpp"
#include "franka_handshake_msgs/action/set_gains.hpp"
#include <rclcpp_action/rclcpp_action.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_handshake_controllers
{

    /**
     * The joint impedance example controller moves joint 4 and 5 in a very compliant periodic movement.
     */
    class HandShakeController : public controller_interface::ControllerInterface
    {
    public:
        using Vector7d = Eigen::Matrix<double, 7, 1>;
        using SetGains = franka_handshake_msgs::action::SetGains;
        using GoalHandleSetGains = rclcpp_action::ServerGoalHandle<SetGains>;
        [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
            const override;
        [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
            const override;
        controller_interface::return_type update(const rclcpp::Time &time,
                                                 const rclcpp::Duration &period) override;
        CallbackReturn on_init() override;
        CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

        using Handshake = franka_handshake_msgs::action::Handshake;
        using GoalHandleHandshake = rclcpp_action::ServerGoalHandle<Handshake>;

        rclcpp_action::Server<Handshake>::SharedPtr handshake_action_server_;

        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const franka_handshake_msgs::action::Handshake::Goal> goal);

        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<franka_handshake_msgs::action::Handshake>> goal_handle);

        void handle_accepted(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<franka_handshake_msgs::action::Handshake>> goal_handle);

        rclcpp_action::GoalResponse handle_gains_goal(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const SetGains::Goal> goal);

        rclcpp_action::CancelResponse handle_gains_cancel(
            const std::shared_ptr<GoalHandleSetGains> goal_handle);

        void handle_gains_accepted(
            const std::shared_ptr<GoalHandleSetGains> goal_handle);

        // Store handshake parameters
        double handshake_amplitude_;
        double handshake_base_frequency_;
        double handshake_n_oscillations_;
        double handshake_synchrony_factor_; // New parameter for synchrony factor
        bool handshake_active_;
        double handshake_start_time_{0.0};
        std::shared_ptr<GoalHandleHandshake> active_goal_handle_;

    protected:
        double handshake_tuning_{0.0}; // handshake frequency (Hz)
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr freq_sub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr commanded_pose_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr actual_pose_pub_;
        void freq_callback(const std_msgs::msg::Float64::SharedPtr msg);

        void handle_action_server_progress(double elapsed_time);

        void publish_commanded_pose(double timestamp, const Vector7d &q_goal);
        void publish_actual_pose(double timestamp, const Vector7d &q_actual);

    private:
        std::string arm_id_;
        std::string robot_description_;
        const int num_joints = 7;
        Vector7d q_;
        Vector7d initial_q_;
        Vector7d dq_;
        Vector7d dq_filtered_;
        Vector7d k_gains_;
        Vector7d d_gains_;
        double controller_elapsed_time_{0.0};
        void updateJointStates();
        double min_jerk(double tau);
        double compute_envelope(double t_now, double T_half, double T_total);

        Vector7d dQ1_, dQ2_;

        std::array<double, 7> phi_;
        std::array<double, 7> omega_;
        std::array<double, 7> A_;
        std::array<double, 7> C_;
        std::array<double, 7> e_t_filt_;
        std::array<double, 7> last_q_goal_;

        // Gains (these are per-joint effective gains; keep them small enough for safety)
        const double sync_gain = this->handshake_synchrony_factor_; // if you have this member. If not, set here, e.g. 1.0 @adnan-saood edit here

        // NOTE: you can expose these to parameters;
        const double Kp = 4.0;     // phase adaptation gain (tune this)
        const double Komega = 0.8; // frequency adaptation gain (tune this)
        const double Ka = 1.0;     // amplitude adaptation gain (gradient step)
        const double Kc = 1.5;     // bias adaptation gain

        // Safety & filtering constants
        const double lambda_omega = 0.5;           // leak/damping toward nominal omega
        const double omega_min = 2.0 * M_PI * 0.2; // lower bound (rad/s)
        const double omega_max = 2.0 * M_PI * 1.5; // upper bound (rad/s)
        const double A_min = -2.0;                 // amplitude lower bound (avoid zero)
        const double A_max = 2.0;                  // amplitude upper bound
        const double C_min = -10.0;                // bias bounds (set to safe joint limits)
        const double C_max = 10.0;
        const double eps_b = 1e-6; // small regularizer for tangent normalization

        // low-pass filter constant for tangential error (choose consistent with controller rate)
        const double tau_e = 0.08; // seconds

        // nominal omega for damping (use base omega for now)
        double omega_base_ = 0.0;
        const double omega_nominal = omega_base_;

        Vector7d Q1_, Q2_;

        // Dynamic loading of impedance controller gains.
        rclcpp_action::Server<SetGains>::SharedPtr set_gains_server_;
        Vector7d k_curr_, d_curr_;
        Vector7d k_target_, d_target_;

        double blend_T_ = 0.5;
        double blend_t_ = 0.0;
        bool blending_ = false;

        // std::mutex gain_mtx_;
    };

} // namespace franka_handshake_controllers
