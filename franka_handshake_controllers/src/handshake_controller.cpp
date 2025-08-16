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
      Vector7d Q1 = initial_q_;
      Vector7d Q2 = initial_q_;

      Q1 += dQ1_; // lower point offsets
      Q2 += dQ2_; // upper point offsets

      // --- Derived scalar timing used by envelope (same as before) ---
      double omega_base = 2.0 * M_PI * (handshake_base_frequency_ + handshake_tuning_);
      if (omega_base <= 0.0)
      {
        RCLCPP_WARN(get_node()->get_logger(), "Omega is non-positive, skipping handshake trajectory.");
        publish_commanded_pose(q_goal);
        publish_actual_pose(q_);
        return controller_interface::return_type::OK;
      }
      double T_half = M_PI / omega_base;
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

      // --- Per-joint adaptive oscillator state (persistent between update calls) ---
      // NOTE: In a production controller you should move these to class members (initialized in on_configure/on_activate).
      static bool _adapt_state_initialized = false;
      static std::array<double, 7> phi;         // phase per joint (rad)
      static std::array<double, 7> omega;       // angular frequency per joint (rad/s)
      static std::array<double, 7> A;           // amplitude per joint (rad)
      static std::array<double, 7> C;           // bias/center per joint (rad)
      static std::array<double, 7> e_t_filt;    // filtered tangential error per joint
      static std::array<double, 7> last_q_goal; // previous commanded goal (for potential diagnostics)

      // Initialize on first handshake update
      if (!_adapt_state_initialized)
      {
        for (int j = 0; j < num_joints; ++j)
        {
          // initialize phi to start at midpoint (alpha = 0.5)
          phi[j] = M_PI / 2.0;

          // initialize omega to base nominal (could be per-joint if desired)
          omega[j] = omega_base;

          // Derive initial amplitude and center from Q1/Q2 so we match interpolation:
          // q_goal = (1-alpha) Q1 + alpha Q2, with alpha = 0.5(1 - cos phi)
          // => equivalently q_goal = C - A*cos(phi) with C = 0.5*(Q1+Q2), A = 0.5*(Q2-Q1)
          C[j] = 0.5 * (Q1[j] + Q2[j]); // midpoint
          A[j] = 0.5 * (Q2[j] - Q1[j]); // half-range (signed)
          e_t_filt[j] = 0.0;
          last_q_goal[j] = C[j] - (E * A[j]) * std::cos(phi[j]);
          RCLCPP_INFO(get_node()->get_logger(),
                      "Adaptive state %d initialized as: %f, %f, %f, %f, %f, %f, %f",
                      j, phi[j], omega[j], A[j], C[j], e_t_filt[j], last_q_goal[j]);
        }
        _adapt_state_initialized = true;

      }

      // --- Tuning parameters (move to class members if you prefer) ---
      // Gains (these are per-joint effective gains; keep them small enough for safety)
      const double sync_gain = 0.0; // if you have this member. If not, set here, e.g. 1.0 @adnan-saood edit here
      // If handshake_sync_ isn't a member, fallback to 1.0:
      // const double sync_gain = 1.0;

      // NOTE: you can expose these to parameters; here we pick names that match the Python sim.
      const double Kp = 4.0;     // phase adaptation gain (tune this)
      const double Komega = 0.8; // frequency adaptation gain (tune this)
      const double Ka = 1.0;     // amplitude adaptation gain (gradient step)
      const double Kc = 1.5;     // bias adaptation gain

      // Safety & filtering constants
      const double lambda_omega = 0.5;           // leak/damping toward nominal omega
      const double omega_min = 2.0 * M_PI * 0.2; // lower bound (rad/s)
      const double omega_max = 2.0 * M_PI * 1.5; // upper bound (rad/s)
      const double A_min = 1e-3;                 // amplitude lower bound (avoid zero)
      const double A_max = 2.0;                  // amplitude upper bound
      const double C_min = -10.0;                // bias bounds (set to safe joint limits)
      const double C_max = 10.0;
      const double eps_b = 1e-6; // small regularizer for tangent normalization

      // low-pass filter constant for tangential error (choose consistent with controller rate)
      const double tau_e = 0.08; // seconds
      double dt = period.seconds();
      const double alpha_e = dt / (tau_e + dt);

      // nominal omega for damping (use base omega for now)
      const double omega_nominal = omega_base;

      // --- Per-joint adaptation update (vectorized in explicit loop for clarity) ---
      for (int j = 0; j < num_joints; ++j)
      {
        // measured joint (actual) and previous commanded
        double q_meas = q_(j);

        // compute per-joint current commanded q_d from current adaptive params:
        // q_d = C - (E * A) * cos(phi)
        double q_d_j = C[j] - (E * A[j]) * std::cos(phi[j]);

        // error between actual and commanded (consistent with earlier math: e_q = q_actual - q_d)
        double e_q = q_meas - q_d_j;

        // derivative of q_d wrt phi: dq_d/dphi = E * A * sin(phi)
        double dq_d_dphi = E * A[j] * std::sin(phi[j]);

        // --- REFINEMENT: smooth tangent normalization (replace sign() with continuous proxy) ---
        // b = dq_d_dphi / (|dq_d_dphi| + eps)
        double b = dq_d_dphi / (std::abs(dq_d_dphi) + eps_b);

        // tangential scalar error
        double e_t = b * e_q;

        // --- REFINEMENT: low-pass filter the tangential error to avoid jitter ---
        // e_t_filt[j] = (1.0 - alpha_e) * e_t_filt[j] + alpha_e * e_t;
        e_t_filt[j] =  e_t;

        // --- ADAPTATION LAWS (use filtered e_t) ---
        // effective sync-scaled gain (envelope already applied through E; multiply by sync if available)
        double current_sync_gain = sync_gain * E;

        // phase and frequency update (PLL-like with damping on omega)
        double phi_dot = omega[j] + current_sync_gain * Kp * e_t_filt[j];
        double omega_dot = current_sync_gain * Komega * e_t_filt[j] - lambda_omega * (omega[j] - omega_nominal);

        // amplitude and center updates (gradient-descent style on squared position error)
        double A_dot = -current_sync_gain * Ka * e_q * std::cos(phi[j]);
        double C_dot = current_sync_gain * Kc * e_q;

        // integrate using dt (robust to variable update rate)
        phi[j] += phi_dot * dt;
        omega[j] += omega_dot * dt;
        A[j] += A_dot * dt;
        C[j] += C_dot * dt;

        // --- Saturate / bound safe ranges ---
        omega[j] = std::clamp(omega[j], omega_min, omega_max);
        A[j] = std::clamp(A[j], A_min, A_max);
        C[j] = std::clamp(C[j], C_min, C_max);

        // keep phi numerically bounded (wrap if desired)
        if (phi[j] > 2.0 * M_PI)
          phi[j] = std::fmod(phi[j], 2.0 * M_PI);
        if (phi[j] < 0.0)
          phi[j] = std::fmod(phi[j], 2.0 * M_PI);

        // save the joint command into q_goal vector in the same joint order
        q_goal(j) = q_d_j;

        // (optional) store last q_goal for diagnostics
        last_q_goal[j] = q_d_j;
      } // end per-joint loop
    } // end if handshake_active_

    // Publish and run your impedance controller as before
    publish_commanded_pose(q_goal);
    publish_actual_pose(q_);

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

  void HandShakeController::publish_commanded_pose(const Vector7d &q_goal)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(7);
    for (int i = 0; i < 7; ++i)
    {
      msg.data[i] = q_goal(i);
    }
    this->commanded_pose_pub_->publish(msg);
  }

  void HandShakeController::publish_actual_pose(const Vector7d &q_actual)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(7);
    for (int i = 0; i < 7; ++i)
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
