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
  using CInterface = controller_interface::InterfaceConfiguration;

  CInterface
  HandShakeController::command_interface_configuration() const
  {
    CInterface config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (int i = 1; i <= num_joints; ++i)
    {
      config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
    }
    return config;
  }

  CInterface
  HandShakeController::state_interface_configuration() const
  {
    CInterface config;
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
      Q1_ = initial_q_ + dQ1_;
      Q2_ = initial_q_ + dQ2_;

      double T_half = M_PI / omega_base_;
      int k_total = this->handshake_n_oscillations_;
      double T_total = static_cast<double>(k_total) * T_half;

      // Elapsed time since handshake start, clamped
      double t_now = controller_elapsed_time_ - handshake_start_time_;
      t_now = std::clamp(t_now, 0.0, T_total);

      // Envelope E(t) (same as before)
      double E = compute_envelope(t_now, T_half, T_total);

      for (int j = 0; j < num_joints; ++j)
        RCLCPP_INFO(get_node()->get_logger(),
                    "Adaptive state %d initialized as: %d, %f, %f, %f, %f, %f, %f",
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

    if (blending_)
    {
      double a = std::min(1.0, blend_t_ / blend_T_);
      double s = a * a * (3.0 - 2.0 * a); // smoothstep
      {
        k_curr_ = (1.0 - s) * k_curr_ + s * k_target_;
        d_curr_ = (1.0 - s) * d_curr_ + s * d_target_;
      }
      blend_t_ += period.seconds();
      if (a >= 1.0)
        blending_ = false;
    }

    // velocity filtering & PD-based torque calculation (unchanged)
    const double kAlpha = 0.99;
    dq_filtered_ = (1 - kAlpha) * dq_filtered_ + kAlpha * dq_;
    Vector7d tau_d_calculated =
        k_curr_.cwiseProduct(q_goal - q_) + d_curr_.cwiseProduct(-dq_filtered_);
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
      auto_declare<std::string>("arm_id", "panda");
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
    RCLCPP_INFO(get_node()->get_logger(), "Configuring HandShakeController");
    try
    {
      RCLCPP_INFO(get_node()->get_logger(), "Getting parameters");
      auto res = get_parameters();
      if(res != CallbackReturn::SUCCESS)
        return res;

      RCLCPP_INFO(get_node()->get_logger(), "Parameters received");

      dq_filtered_.setZero();

      RCLCPP_INFO(get_node()->get_logger(), "Getting robot_description from robot_state_publisher");
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

      RCLCPP_INFO(get_node()->get_logger(), "Setting up publishers and subscribers");
      setTopicPublishersSubscribers();
      RCLCPP_INFO(get_node()->get_logger(), "Setting up action servers");
      setActionServers();
      RCLCPP_INFO(get_node()->get_logger(), "Action servers set up successfully");

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

  double HandShakeController::min_jerk(double tau)
  {
    tau = std::clamp(tau, 0.0, 1.0);
    double tau_3 = tau * tau * tau;
    return 10.0 * tau_3 - 15.0 * tau_3 * tau + 6.0 * tau_3 * tau * tau;
  }

  double HandShakeController::compute_envelope(double t_now, double T_half, double T_total)
  {
    if (t_now < T_half)
      return min_jerk(t_now / T_half);
    else if (t_now > (T_total - T_half))
      return min_jerk((T_total - t_now) / T_half);
    else
      return 1.0;
  }

} // namespace franka_handshake_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_handshake_controllers::HandShakeController,
                       controller_interface::ControllerInterface)
