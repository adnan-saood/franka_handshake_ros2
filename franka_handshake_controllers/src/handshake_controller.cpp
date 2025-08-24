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
  using Handshake = franka_handshake_msgs::action::Handshake;
  using GoalHandleHandshake = rclcpp_action::ServerGoalHandle<Handshake>;

  void HandShakeController::freq_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    handshake_tuning_ = msg->data;
    RCLCPP_INFO(get_node()->get_logger(), "Handshake tuning frequency updated to: %f Hz", handshake_tuning_);
  }

  controller_interface::InterfaceConfiguration
  HandShakeController::command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (int i = 1; i <= num_joints; ++i)
    {
      config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
    }
    return config;
  }

  controller_interface::InterfaceConfiguration
  HandShakeController::state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (int i = 1; i <= num_joints; ++i)
    {
      config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
      config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
    }
    return config;
  }

  controller_interface::return_type HandShakeController::update(
      const rclcpp::Time & /*time*/,
      const rclcpp::Duration &period)
  {
    // --- keep try/catch for safety ---
    try
    {
      updateJointStates();
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Exception in updateJointStates: %s", e.what());
      return controller_interface::return_type::ERROR;
    }

    Vector7d q_goal = initial_q_;
    controller_elapsed_time_ = controller_elapsed_time_ + period.seconds();

    handle_action_server_progress(controller_elapsed_time_);

    if (this->handshake_active_)
    {
      // --- Setup Q1/Q2 as before (joint offsets from initial_q_) ---
      Vector7d QS = initial_q_;
      Q1_ = initial_q_;
      Q2_ = initial_q_;

      Q1_ += dQ1_; // lower point offsets
      Q2_ += dQ2_; // upper point offsets

      double T_half = M_PI / omega_base_;
      int k_total = this->handshake_n_oscillations_;
      double T_total = static_cast<double>(k_total) * T_half;

      // Elapsed time since handshake start, clamped
      double t_now = controller_elapsed_time_ - handshake_start_time_;
      t_now = std::clamp(t_now, 0.0, T_total);

      // minimum-jerk ramp (same as before)
      auto min_jerk = [](double tau)
      {
        tau = std::clamp(tau, 0.0, 1.0);
        return 10.0 * tau * tau * tau - 15.0 * tau * tau * tau * tau + 6.0 * tau * tau * tau * tau * tau;
      };

      // Envelope E(t) (same as before)
      double E = 1.0;
      if (t_now < T_half)
      {
        double tau = t_now / T_half;
        E = min_jerk(tau);
      }
      else if (t_now > (T_total - T_half))
      {
        double tau = (T_total - t_now) / T_half;
        E = min_jerk(tau);
      }

      for (int j = 0; j < num_joints; ++j)
        RCLCPP_INFO(get_node()->get_logger(),
                    "Adaptive state %d initialized as: %f, %f, %f, %f, %f, %f, %f",
                    j, phi_[j], omega_[j], A_[j], C_[j], e_t_filt_[j], last_q_goal_[j]);

      double dt = period.seconds();
      const double alpha_e = dt / (tau_e + dt);

      // --- Per-joint adaptation update (vectorized in explicit loop for clarity) ---
      for (int j = 0; j < num_joints; ++j)
      {
        // measured joint (actual) and previous commanded
        double q_meas = q_(j);

        // compute per-joint current commanded q_d from current adaptive params:
        // q_d = C - (E * A) * cos(phi)
        double q_d_j = C_[j] - (E * A_[j]) * std::cos(phi_[j]);

        // error between actual and commanded (consistent with earlier math: e_q = q_actual - q_d)
        double e_q = q_meas - q_d_j;

        // derivative of q_d wrt phi: dq_d/dphi = E * A * sin(phi)
        double dq_d_dphi = E * A_[j] * std::sin(phi_[j]);

        // --- REFINEMENT: smooth tangent normalization (replace sign() with continuous proxy) ---
        // b = dq_d_dphi / (|dq_d_dphi| + eps)
        double b = dq_d_dphi / (std::abs(dq_d_dphi) + eps_b);

        // tangential scalar error
        double e_t = b * e_q;

        // --- REFINEMENT: low-pass filter the tangential error to avoid jitter ---
        // e_t_filt_[j] = (1.0 - alpha_e) * e_t_filt_[j] + alpha_e * e_t;
        e_t_filt_[j] = e_t;

        // --- ADAPTATION LAWS (use filtered e_t) ---
        // effective sync-scaled gain (envelope already applied through E; multiply by sync if available)
        double current_sync_gain = sync_gain * E;

        // phase and frequency update (PLL-like with damping on omega)
        double phi_dot = omega_[j] + current_sync_gain * Kp * e_t_filt_[j];
        double omega_dot = current_sync_gain * Komega * e_t_filt_[j] - lambda_omega * (omega_[j] - omega_nominal);

        // amplitude and center updates (gradient-descent style on squared position error)
        double A_dot = -current_sync_gain * Ka * e_q * std::cos(phi_[j]);
        double C_dot = current_sync_gain * Kc * e_q;

        // integrate using dt (robust to variable update rate)
        phi_[j] += phi_dot * dt;
        omega_[j] += omega_dot * dt;
        A_[j] += A_dot * dt;
        C_[j] += C_dot * dt;

        // --- Saturate / bound safe ranges ---
        omega_[j] = std::clamp(omega_[j], omega_min, omega_max);
        A_[j] = std::clamp(A_[j], A_min, A_max);
        C_[j] = std::clamp(C_[j], C_min, C_max);

        // keep phi numerically bounded (wrap if desired)
        if (phi_[j] > 2.0 * M_PI)
          phi_[j] = std::fmod(phi_[j], 2.0 * M_PI);
        if (phi_[j] < 0.0)
          phi_[j] = std::fmod(phi_[j], 2.0 * M_PI);

        // save the joint command into q_goal vector in the same joint order
        q_goal(j) = q_d_j;

        // (optional) store last q_goal for diagnostics
        last_q_goal_[j] = q_d_j;
      } // end per-joint loop
    } // end if handshake_active_

    // Publish and run impedance controller
    publish_commanded_pose(this->controller_elapsed_time_, q_goal);
    publish_actual_pose(this->controller_elapsed_time_, q_);

    // velocity filtering & PD-based torque calculation (unchanged)
    const double kAlpha = 0.99;
    dq_filtered_ = (1 - kAlpha) * dq_filtered_ + kAlpha * dq_;
    Vector7d tau_d_calculated =
        k_gains_.cwiseProduct(q_goal - q_) + d_gains_.cwiseProduct(-dq_filtered_);
    for (int i = 0; i < num_joints; ++i)
    {
      command_interfaces_[i].set_value(tau_d_calculated(i));
    }

    return controller_interface::return_type::OK;
  }

  CallbackReturn HandShakeController::on_init()
  {
    try
    {
      auto_declare<std::string>("arm_id", "");
      auto_declare<std::vector<double>>("k_gains", {});
      auto_declare<std::vector<double>>("d_gains", {});
    }
    catch (const std::exception &e)
    {
      fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
      return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn HandShakeController::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    try
    {
      arm_id_ = get_node()->get_parameter("arm_id").as_string();
      auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
      auto d_gains = get_node()->get_parameter("d_gains").as_double_array();
      if (k_gains.empty())
      {
        RCLCPP_FATAL(get_node()->get_logger(), "k_gains parameter not set");
        return CallbackReturn::FAILURE;
      }
      if (k_gains.size() != static_cast<uint>(num_joints))
      {
        RCLCPP_FATAL(get_node()->get_logger(), "k_gains should be of size %d but is of size %ld",
                     num_joints, k_gains.size());
        return CallbackReturn::FAILURE;
      }
      if (d_gains.empty())
      {
        RCLCPP_FATAL(get_node()->get_logger(), "d_gains parameter not set");
        return CallbackReturn::FAILURE;
      }
      if (d_gains.size() != static_cast<uint>(num_joints))
      {
        RCLCPP_FATAL(get_node()->get_logger(), "d_gains should be of size %d but is of size %ld",
                     num_joints, d_gains.size());
        return CallbackReturn::FAILURE;
      }
      for (int i = 0; i < num_joints; ++i)
      {
        d_gains_(i) = d_gains.at(i);
        k_gains_(i) = k_gains.at(i);
      }
      dq_filtered_.setZero();

      auto parameters_client =
          std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "robot_state_publisher");
      parameters_client->wait_for_service();

      auto future = parameters_client->get_parameters({"robot_description"});
      auto result = future.get();
      if (!result.empty())
      {
        robot_description_ = result[0].value_to_string();
      }
      else
      {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
      }

      arm_id_ = "fr3";

      freq_sub_ = get_node()->create_subscription<std_msgs::msg::Float64>(
          "franka_handshake_tuning_freq", 10,
          std::bind(&HandShakeController::freq_callback, this, std::placeholders::_1));

      this->commanded_pose_pub_ =
          get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("/commanded_pose", 10);

      this->actual_pose_pub_ =
          get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("/actual_pose", 10);

      handshake_action_server_ = rclcpp_action::create_server<Handshake>(
          get_node()->get_node_base_interface(),
          get_node()->get_node_clock_interface(),
          get_node()->get_node_logging_interface(),
          get_node()->get_node_waitables_interface(),
          "handshake",
          std::bind(&HandShakeController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
          std::bind(&HandShakeController::handle_cancel, this, std::placeholders::_1),
          std::bind(&HandShakeController::handle_accepted, this, std::placeholders::_1));
    }
    catch (const std::exception &e)
    {
      RCLCPP_FATAL(get_node()->get_logger(), "Exception in on_configure: %s", e.what());
      return CallbackReturn::FAILURE;
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn HandShakeController::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    try
    {
      updateJointStates();
      dq_filtered_.setZero();
      initial_q_ = q_;
      controller_elapsed_time_ = 0.0;

      auto dQ1 = get_node()->get_parameter("dQ1").as_double_array();
      auto dQ2 = get_node()->get_parameter("dQ2").as_double_array();

      if (dQ1.size() != 7 || dQ2.size() != 7)
      {
        RCLCPP_FATAL(get_node()->get_logger(), "dQ1 and dQ2 must be size 7");
        return CallbackReturn::FAILURE;
      }

      for (int i = 0; i < 7; i++)
      {
        dQ1_(i) = dQ1.at(i);
        dQ2_(i) = dQ2.at(i);
      }
    }
    catch (const std::exception &e)
    {
      RCLCPP_FATAL(get_node()->get_logger(), "Exception in on_activate: %s", e.what());
      return CallbackReturn::FAILURE;
    }
    return CallbackReturn::SUCCESS;
  }

  void HandShakeController::updateJointStates()
  {
    for (auto i = 0; i < num_joints; ++i)
    {
      const auto &position_interface = state_interfaces_.at(2 * i);
      const auto &velocity_interface = state_interfaces_.at(2 * i + 1);

      assert(position_interface.get_interface_name() == "position");
      assert(velocity_interface.get_interface_name() == "velocity");

      q_(i) = position_interface.get_value();
      dq_(i) = velocity_interface.get_value();
    }
  }

  using namespace std::chrono_literals;

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

} // namespace franka_handshake_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_handshake_controllers::HandShakeController,
                       controller_interface::ControllerInterface)
