#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
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

#include <control_toolbox/pid.hpp>
#include <gpiod.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>

#include "speedy_control/MotorDriver.hpp"
#include "speedy_control/HallEncoder.hpp"
#include "speedy_control/ServoDriver.hpp"

namespace speedy_control
{
class SpeedyHardwareInterface : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(SpeedyHardwareInterface)

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter>& params);

    template<typename T>
    T parse_param(const hardware_interface::HardwareInfo& info, const std::string& name, T default_val) {
        auto it = info.hardware_parameters.find(name);
        if (it != info.hardware_parameters.end()) {
            if constexpr (std::is_same_v<T, double>) return std::stod(it->second);
            if constexpr (std::is_same_v<T, int>) return std::stoi(it->second);
            if constexpr (std::is_same_v<T, int64_t>) return std::stoll(it->second);
            if constexpr (std::is_same_v<T, bool>) {
                std::string val = it->second;
                std::transform(val.begin(), val.end(), val.begin(), ::tolower);
                return val == "true" || val == "1";
            }
        }
        return default_val;
    }

    rclcpp::Logger logger_ = rclcpp::get_logger("SpeedyHardwareInterface");
    
    // Configurações de Hardware (Pinos e Canais)
    struct {
        int motor_en_1=17;
        int motor_en_2=27;
        int motor_fwd_ch=0;
        int motor_rev_ch=1;
        int servo=15;
        int hall=24;
        int hall_en=23;
        int hall_fl=25;   // roda dianteira esquerda
        int hall_fr=22;   // roda dianteira direita
    } cfg_pins_;

    // Cinemática
    struct {
        double wheel_radius=0.0325;
        double wheelbase=0.251;
        double gear_ratio=2.3;
        int magnets_per_rev=6;
        int front_magnets_per_rev=10;  // ímãs por volta — medição direta na roda dianteira
    } cfg_kinematics_;

    // PID e Filtros (laço interno em rad/s; deadband configurado em m/s)
    struct {
        double kp=0.5;
        double ki=0.1;
        double kd=0.0;
        double filter_alpha=0.1;
        double deadband_m_s=0.02;  // config humana em m/s; convertido p/ rad/s na write()
        double zero_timeout_s=0.15;  // motor off + sem pulsos por este tempo → velocidade = 0
    } cfg_pid_;

    // Motor de Tração
    struct {
        int64_t period_ns=50000;
        double min_effort=0.05;
        int64_t deadband_ms=10;
        bool invert_direction=false;
        double max_linear_velocity=1.0;   // m/s — backstop de segurança
        double velocity_feedforward=0.0;  // duty (0..1) na velocidade máxima
    } cfg_motor_;

    // Servo de Direção
    struct {
        double center_pulse_ms=1.5;
        double max_deflection_pulse_ms=1.0;
        double max_angle_left_deg=45.0;
        double max_angle_right_deg=45.0;
        double trim_deg=0.0;
        double max_speed_deg_per_sec=0.0;
        bool invert_direction=false;
        // Regressão polinomial: pulse_ms = a0 + a1*deg + a2*deg²
        bool use_polynomial=false;
        std::array<double, 3> poly_coefs={0.0, 0.0, 0.0};
    } cfg_steer_;

    // Estados de Runtime
    double hw_velocity_cmd_ = 0.0, hw_velocity_state_ = 0.0;
    double hw_drive_pos_ = 0.0;  // posição integrada (rad) — evita NaN no TF do drive_joint
    double hw_steering_cmd_ = 0.0, hw_steering_state_ = 0.0;
    double hw_vel_fl_ = 0.0, hw_vel_fr_ = 0.0;
    double hw_pos_fl_ = 0.0, hw_pos_fr_ = 0.0;

    MotorDriver motor_;
    ServoDriver servo_;
    HallEncoder encoder_;
    HallEncoder encoder_fl_;
    HallEncoder encoder_fr_;
    control_toolbox::Pid pid_controller_;
    double filtered_velocity_ = 0.0;
    double filtered_vel_fl_ = 0.0, filtered_vel_fr_ = 0.0;

    // Protege os structs cfg_* (escritos no param_callback, lidos em read()/write()
    // por threads distintas). Os ganhos do PID usam o padrão atómico separado abaixo.
    std::mutex cfg_mutex_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr front_odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr servo_pulse_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motor_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motor_raw_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motor_filtered_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr motor_effort_pub_;

    // Nó de Parâmetros Dinâmicos
    rclcpp::Node::SharedPtr param_node_;
    std::thread param_thread_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
    std::atomic<bool> stop_threads_{false};
    std::atomic<bool> update_pid_{false};
    double new_kp_=0.5, new_ki_=0.1, new_kd_=0.0;

    // Enable da ponte H (GPIOd) — habilitado na ativação.
    std::unique_ptr<gpiod::line_request> en_request_;
    std::unique_ptr<gpiod::line_request> en2_request_;
    int64_t prev_pulse_count_ = 0;
    int64_t prev_pulse_fl_ = 0, prev_pulse_fr_ = 0;
    double last_wheel_dir_ = 1.0;
    double t_idle_rear_ = 0.0, t_idle_fl_ = 0.0, t_idle_fr_ = 0.0;  // tempo sem pulsos (parada limpa)
};
}
