#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <vector>
#include <thread>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"

#include <std_msgs/msg/float32_multi_array.hpp>
#include <control_toolbox/pid.hpp>
#include <gpiod.hpp>

namespace speedy_control
{
class SpeedyHardwareInterface : public hardware_interface::SystemInterface
{
public:
    /**
     * @brief Helper para manipulação segura de PWM via SysFS.
     */
    struct SafePWM {
        bool init(int channel, int64_t period_ns, const std::string& name);
        bool set_duty_ns(int64_t duty_ns);
        void stop();
    private:
        int fd_duty_ = -1;
        int channel_ = 0;
        int64_t period_ns_ = 20000000;
        std::string chip_path_;
        std::string name_;
        void write_sysfs(const std::string& path, const std::string& value);
    };

    RCLCPP_SHARED_PTR_DEFINITIONS(SpeedyHardwareInterface)

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    void hall_interrupt_loop();
    rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter>& params);

    template<typename T>
    T parse_param(const hardware_interface::HardwareInfo& info, const std::string& name, T default_val) {
        auto it = info.hardware_parameters.find(name);
        if (it != info.hardware_parameters.end()) {
            if constexpr (std::is_same_v<T, double>) return std::stod(it->second);
            if constexpr (std::is_same_v<T, int>) return std::stoi(it->second);
            if constexpr (std::is_same_v<T, int64_t>) return std::stoll(it->second);
            if constexpr (std::is_same_v<T, bool>) return it->second == "true" || it->second == "1";
        }
        return default_val;
    }

    rclcpp::Logger logger_ = rclcpp::get_logger("SpeedyHardwareInterface");
    
    // Configurações de Hardware (Pinos e Canais)
    struct { 
        int motor_en_1=17; 
        int motor_en_2=27; 
        int motor_fwd=12; 
        int motor_fwd_ch=0; 
        int motor_rev=13; 
        int motor_rev_ch=1; 
        int servo=18; 
        int servo_ch=2; 
        int hall=24; 
    } cfg_pins_;

    // Cinemática
    struct { 
        double wheel_radius=0.0325; 
        double gear_ratio=2.3; 
        int magnets_per_rev=4; 
    } cfg_kinematics_;

    // PID e Filtros
    struct { 
        double kp=0.6; 
        double ki=0.4; 
        double kd=0.0; 
        double filter_alpha=0.02; 
        double deadband_m_s=0.01; 
    } cfg_pid_;

    // Motor de Tração
    struct { 
        int64_t period_ns=50000; 
        double min_effort=0.05; 
        double max_effort=1.0;
        int64_t deadband_ms=10; 
        bool invert_direction=false; 
    } cfg_motor_;

    // Servo de Direção
    struct { 
        double pwm_freq_hz=50.0; 
        double center_pulse_ms=1.5; 
        double max_deflection_pulse_ms=1.0; 
        double max_angle_left_deg=45.0;
        double max_angle_right_deg=45.0;
        double nominal_range_deg=45.0;
        double trim_deg=0.0;
        double poly_a2=0.000333;
        double poly_a1=0.028469;
        double poly_a0=1.481;
        bool use_polynomial=true;
        double max_speed_deg_per_sec=0.0; 
        bool invert_direction=false; 
    } cfg_steer_;

    // Estados de Runtime
    double hw_velocity_cmd_ = 0.0, hw_velocity_state_ = 0.0;
    double hw_position_state_ = 0.0;
    double hw_steering_cmd_ = 0.0, hw_steering_state_ = 0.0;
    
    SafePWM pwm_motor_fwd_, pwm_motor_rev_, pwm_servo_;
    control_toolbox::Pid pid_controller_;
    
    double filtered_velocity_ = 0.0;
    int8_t last_direction_ = 0;
    bool in_deadband_ = false;
    rclcpp::Time deadband_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    double current_steering_deg_ = 0.0;
    
    // Nó de Parâmetros Dinâmicos e Debug
    rclcpp::Node::SharedPtr param_node_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr debug_pub_;
    std::thread param_thread_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
    std::atomic<bool> stop_threads_{false};
    std::atomic<bool> update_pid_{false};
    double new_kp_=0.6, new_ki_=0.4, new_kd_=0.0;

    // Sensor Hall e Enable (GPIOd)
    std::unique_ptr<gpiod::line_request> hall_request_;
    std::unique_ptr<gpiod::line_request> en_request_;
    std::unique_ptr<gpiod::line_request> en2_request_;
    std::thread hall_thread_;
    std::atomic<int64_t> hall_pulse_count_{0};
    int64_t prev_pulse_count_ = 0;
};
}
