#ifndef MESKO_HARDWARE_HPP_
#define MESKO_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "mesko_hardware/visibility_control.h"
#include "mesko_hardware/wifi_comms.hpp"

namespace mesko_hardware
{
  class MeskoHardware : public hardware_interface::SystemInterface
  {
    struct Config
    {
      float loop_rate = 0.0;
      std::string device = "";
      int baud_rate = 0;
      int timeout_ms = 0;
    };

    public:
      RCLCPP_SHARED_PTR_DEFINITIONS(MeskoHardware)

      MESKO_HARDWARE_PUBLIC
      hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;

      MESKO_HARDWARE_PUBLIC
      hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;

      MESKO_HARDWARE_PUBLIC
      std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

      MESKO_HARDWARE_PUBLIC
      std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

      MESKO_HARDWARE_PUBLIC
      hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

      MESKO_HARDWARE_PUBLIC
      hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

      MESKO_HARDWARE_PUBLIC
      hardware_interface::return_type read(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

      MESKO_HARDWARE_PUBLIC
      hardware_interface::return_type write(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

    private:
      ESP32UDPComms comms_;
      Config cfg_;
      std::vector<double> hw_commands_;
      std::vector<double> hw_states_position_;

  };

}  

#endif  // 
