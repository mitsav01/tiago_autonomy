#include <string>
#include <vector>
#include <memory>

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

namespace tiago_joint_point_controllers
{
class JointPointCommandController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override {
    try {
      auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
      auto_declare<std::string>("interface_name", hardware_interface::HW_IF_POSITION);
    } catch (const std::exception & e) {
      return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration command_interface_configuration() const override {
    controller_interface::InterfaceConfiguration conf;
    conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint : joints_) {
      conf.names.push_back(joint + "/" + interface_name_);
    }
    return conf;
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override {
    return {controller_interface::interface_configuration_type::NONE};
  }

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
    joints_ = get_node()->get_parameter("joints").as_string_array();
    interface_name_ = get_node()->get_parameter("interface_name").as_string();
    if (joints_.empty()) return controller_interface::CallbackReturn::ERROR;

    point_sub_ = get_node()->create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>(
      "~/joint_commands",
      rclcpp::SensorDataQoS(),
      [this](const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr msg) {
        rt_buffer_.writeFromNonRT(*msg);
      });
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
    rt_buffer_.reset();
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
    rt_buffer_.reset();
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override {
    auto last_point = rt_buffer_.readFromRT();
    if (!last_point || last_point->positions.size() != command_interfaces_.size()) {
      return controller_interface::return_type::OK;
    }

    for (size_t i = 0; i < command_interfaces_.size(); ++i) {
      command_interfaces_[i].set_value(last_point->positions[i]);
    }
    return controller_interface::return_type::OK;
  }

private:
  std::vector<std::string> joints_;
  std::string interface_name_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr point_sub_;
  realtime_tools::RealtimeBuffer<trajectory_msgs::msg::JointTrajectoryPoint> rt_buffer_;
};
}  // namespace tiago_joint_point_controllers

PLUGINLIB_EXPORT_CLASS(
  tiago_joint_point_controllers::JointPointCommandController,
  controller_interface::ControllerInterface)