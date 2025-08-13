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

#include <franka_handshake_controllers/handshake_controller.hpp>
#include <franka_handshake_controllers/robot_utils.hpp>
#include <std_msgs/msg/float64.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>

namespace franka_handshake_controllers
{

  void HandShakeController::freq_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    hs_freq_ = msg->data;
    RCLCPP_INFO(get_node()->get_logger(), "Handshake frequency updated to: %f Hz", hs_freq_);
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
    updateJointStates();
    Vector7d q_goal = initial_q_;

    // Vector7d QS = initial_q_;
    // Vector7d Q1 = initial_q_;
    // Vector7d Q2 = initial_q_;

    // // add dQ_ to Q
    // Q1 += dQ1_;
    // Q2 += dQ2_;

    // elapsed_time_ = elapsed_time_ + period.seconds();

    // // Use hs_freq_ for cosine frequency
    // double omega = 2 * M_PI * hs_freq_;
    // double alpha = 0.5 + 0.5 * std::cos(omega * elapsed_time_);
    // static double last_print_time = 0.0;
    // if (elapsed_time_ - last_print_time >= 1.0) {
    //   RCLCPP_INFO(get_node()->get_logger(), "Current handshake omega: %f", omega);
    //   last_print_time = elapsed_time_;
    // }
    // // now move parameterically between Q1 and Q2
    // q_goal = (1 - alpha) * Q1 + alpha * Q2;

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
      hs_freq_ = 0.4; // default frequency
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

    // Create subscriber for handshake frequency
    freq_sub_ = get_node()->create_subscription<std_msgs::msg::Float64>(
      "franka_handshake_freq", 10,
      std::bind(&HandShakeController::freq_callback, this, std::placeholders::_1));

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn HandShakeController::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    updateJointStates();
    dq_filtered_.setZero();
    initial_q_ = q_;
    elapsed_time_ = 0.0;

    // read from params
    auto dQ1 = get_node()->get_parameter("dQ1").as_double_array();
    auto dQ2 = get_node()->get_parameter("dQ2").as_double_array();

    for(int i = 0; i < 7; i++)
    {
      dQ1_(i) = dQ1.at(i);
      dQ2_(i) = dQ2.at(i);
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

} // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_handshake_controllers::HandShakeController,
                       controller_interface::ControllerInterface)
