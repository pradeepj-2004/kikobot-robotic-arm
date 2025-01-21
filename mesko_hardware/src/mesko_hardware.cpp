#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"


// #include "mesko_hardware/mesko_hardware.hpp"
#include "mesko_hardware/mesko_hardware_wifi.hpp"

using namespace std;

namespace mesko_hardware
{

    //*************** ON INIT FUNCTION ****************** //
    std::vector<double> initial_values;
    int i=0;
    std::once_flag flag;
    
    hardware_interface::CallbackReturn MeskoHardware::on_init(const hardware_interface::HardwareInfo & info)
    {
    if (
        hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Parameter settings
    initial_values.resize(info_.joints.size(), 0.0);

    cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
    cfg_.device = info_.hardware_parameters["device"];
    cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
    cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);

    hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    
    hw_states_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    
    for (const hardware_interface::ComponentInfo & joint : info_.joints)
    {
        //Command interfaces 
        if (joint.command_interfaces.size() != 1)
        {
        RCLCPP_FATAL(
            rclcpp::get_logger("MeskoHardware"),
            "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
        RCLCPP_FATAL(
            rclcpp::get_logger("MeskoHardware"),
            "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
            joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
        }

        //State interfaces 
        if (joint.state_interfaces.size() != 1)
        {
        RCLCPP_FATAL(
            rclcpp::get_logger("MeskoHardware"),
            "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;

        
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
        RCLCPP_FATAL(
            rclcpp::get_logger("MeskoHardware"),
            "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
        }
    

        // initial position value fetching

        double joint_initial{0.0};
        double temp;
        auto get_initial_value =
        [this,&joint_initial](const hardware_interface::InterfaceInfo & interface_info) {
          
          if (!interface_info.initial_value.empty()) {
            try {
              joint_initial = std::stod(interface_info.initial_value);
            } catch (std::invalid_argument &) {
              std::cout<<"error"<<std::endl;
            }
          }
          return joint_initial;
        };

        if (joint.state_interfaces[0].name == "position") {
           temp = get_initial_value(joint.state_interfaces[0]);
            
        }
        initial_values[i]= temp;
        i=i+1;
        // initial position value fetching
    }
    i=0;

    RCLCPP_INFO(rclcpp::get_logger("MeskoHardware"), "Successfully initialised!");


    return hardware_interface::CallbackReturn::SUCCESS;

    }

    //*************** ON INIT FUNCTION ****************** //






    //*************** ON CONFIGURE FUNCTION ****************** //

    hardware_interface::CallbackReturn MeskoHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {

    
    for (uint i=0; i< hw_states_position_.size();i++)
    {      
        hw_states_position_[i]=0.0;
        hw_commands_[i]=0.0;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("MeskoHardware"), "Successfully configured!");
    return hardware_interface::CallbackReturn::SUCCESS;

    }
    //*************** ON CONFIGURE FUNCTION ****************** //







    //*************** ON ACTIVATE FUNCTION ****************** //

    hardware_interface::CallbackReturn MeskoHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
    RCLCPP_INFO(rclcpp::get_logger("MeskoHardware"), "Activating ...please wait...");
    if (comms_.connected()){comms_.disconnect();}

    // comms_.connect(cfg_.device,cfg_.baud_rate,cfg_.timeout_ms);

    const std::string esp32_ip = "192.168.200.136"; // Change to your ESP32's IP
    const int udp_port_1 = 8888;
    const int udp_port_2 = 7777;
    comms_.connect(esp32_ip, udp_port_1,udp_port_2);

    RCLCPP_INFO(rclcpp::get_logger("MeskoHardware"), "Sucessfully Activated");

    return hardware_interface::CallbackReturn::SUCCESS;
    }
    //*************** ON ACTIVATE FUNCTION ****************** //






    //*************** EXPORT STATE INTERFACES ***************//

    std::vector<hardware_interface::StateInterface> MeskoHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        
        for (uint i=0; i< info_.joints.size();i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name,
            hardware_interface::HW_IF_POSITION,&hw_commands_[i]));
        
        }
    return state_interfaces;
    }

    //*************** EXPORT STATE INTERFACES ***************//




    //*************** EXPORT COMMAND INTERFACES ***************//
    std::vector<hardware_interface::CommandInterface> MeskoHardware::export_command_interfaces()
    {

    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (uint i=0; i< info_.joints.size();i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name,
            hardware_interface::HW_IF_POSITION,&hw_commands_[i]));
        }
    
    return command_interfaces;
    }

    //*************** EXPORT COMMAND INTERFACES ***************//




    //*************** READ FUNCTION ****************** //

    hardware_interface::return_type MeskoHardware::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
    // std::vector<double>v = comms_.read_joint_values();
    std::vector<double>v = comms_.read_joint_values("e\r");
    hw_states_position_[0]=v[0];
    hw_states_position_[1]=v[1];
    hw_states_position_[2]=v[2];
    hw_states_position_[3]=v[3];
    hw_states_position_[4]=v[4];
    hw_states_position_[5]=v[5];
    return hardware_interface::return_type::OK;
    }




    //*************** WRITE FUNCTION ****************** //
    hardware_interface::return_type MeskoHardware::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
      static bool initial_values_set = false; 

      if (!initial_values_set) {
          
          hw_commands_[0]=initial_values[0];
          hw_commands_[1]=initial_values[1];
          hw_commands_[2]=initial_values[2];
          hw_commands_[3]=initial_values[3];
          hw_commands_[4]=initial_values[4];
          hw_commands_[5]=initial_values[5];

          initial_values_set = true; // Mark as set
          RCLCPP_INFO(rclcpp::get_logger("MeskoHardware"), "Initialising position of arm ...please wait...");
      }
      
      else {
          
          comms_.set_joints_values(hw_commands_[0], hw_commands_[1], hw_commands_[2],
                                  hw_commands_[3], hw_commands_[4], hw_commands_[5]);
          //std::cout<<"Hello"<<std::endl;
      }
    return hardware_interface::return_type::OK;
    }


    //*************** ON DE-ACTIVATE FUNCTION ****************** //

    hardware_interface::CallbackReturn MeskoHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
    RCLCPP_INFO(rclcpp::get_logger("MeskoHardware"), "Deactivating ...please wait...");
    comms_.disconnect();
    RCLCPP_INFO(rclcpp::get_logger("MeskoHardware"), "Successfully deactivated!");
    return hardware_interface::CallbackReturn::SUCCESS;

    }
    //*************** ON DE-ACTIVATE FUNCTION ****************** //



}  // namespace blue_arm_hardware

//This is for making this blue_arm_hardware into a ros2 plugin
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(mesko_hardware::MeskoHardware, hardware_interface::SystemInterface)
