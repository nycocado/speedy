#include "speedy_control/SpeedyHardwareInterface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>
#include <unistd.h>

namespace speedy_control
{
// ---------------------------------------------------------------------------------
// IMPLEMENTAÇÃO DA CLASSE SafePWM
// ---------------------------------------------------------------------------------

void SpeedyHardwareInterface::SafePWM::write_sysfs(const std::string& path,
                                                   const std::string& value)
{
    std::ofstream fs(path);
    if (fs.is_open())
    {
        fs << value;
    }
}

bool SpeedyHardwareInterface::SafePWM::init(int channel, int64_t period_ns,
                                            const std::string& name)
{
    name_ = name;
    channel_ = channel;
    period_ns_ = period_ns;

    // Busca robusta: Pi 3 (pwmchip0) ou Pi 5 (pwmchip2/6/etc)
    chip_path_ = "/sys/class/pwm/pwmchip0";
    if (!std::filesystem::exists(chip_path_))
    {
        for (int i = 1; i < 10; i++)
        {
            std::string p = "/sys/class/pwm/pwmchip" + std::to_string(i);
            if (std::filesystem::exists(p))
            {
                chip_path_ = p;
                break;
            }
        }
    }

    if (!std::filesystem::exists(chip_path_))
    {
        return false;
    }

    std::string pwm_path = chip_path_ + "/pwm" + std::to_string(channel);
    if (!std::filesystem::exists(pwm_path))
    {
        write_sysfs(chip_path_ + "/export", std::to_string(channel));
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    // Configuração inicial obrigatória
    write_sysfs(pwm_path + "/period", std::to_string(period_ns));
    write_sysfs(pwm_path + "/enable", "1");

    std::string duty_path = pwm_path + "/duty_cycle";
    fd_duty_ = open(duty_path.c_str(), O_WRONLY);

    return (fd_duty_ >= 0);
}

bool SpeedyHardwareInterface::SafePWM::set_duty_ns(int64_t duty_ns)
{
    if (fd_duty_ >= 0)
    {
        if (duty_ns > period_ns_)
            duty_ns = period_ns_;
        if (duty_ns < 0)
            duty_ns = 0;

        char buf[32];
        int len = snprintf(buf, sizeof(buf), "%lld\n", (long long)duty_ns);
        if (pwrite(fd_duty_, buf, len, 0) < 0)
        {
            return false; // Erro de I/O físico com o driver sysfs
        }
        return true;
    }
    return false; // Arquivo não está aberto
}

void SpeedyHardwareInterface::SafePWM::stop()
{
    set_duty_ns(0);
    if (fd_duty_ >= 0)
    {
        close(fd_duty_);
        fd_duty_ = -1;
    }
}

// ---------------------------------------------------------------------------------
// SPEEDY HARDWARE INTERFACE
// ---------------------------------------------------------------------------------

hardware_interface::CallbackReturn SpeedyHardwareInterface::on_init(
    const hardware_interface::HardwareInfo& info)
{
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    if (info_.joints.size() != 2)
    {
        RCLCPP_FATAL(logger_, "SpeedyHardwareInterface requires exactly 2 joints.");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Lendo Pinos e Canais
    cfg_pins_.motor_en_1 = parse_param(info, "pin_motor_en_1", cfg_pins_.motor_en_1);
    cfg_pins_.motor_en_2 = parse_param(info, "pin_motor_en_2", cfg_pins_.motor_en_2);
    cfg_pins_.motor_fwd = parse_param(info, "pin_motor_fwd", cfg_pins_.motor_fwd);
    cfg_pins_.motor_fwd_ch =
        parse_param(info, "pin_motor_fwd_ch", cfg_pins_.motor_fwd_ch);
    cfg_pins_.motor_rev = parse_param(info, "pin_motor_rev", cfg_pins_.motor_rev);
    cfg_pins_.motor_rev_ch =
        parse_param(info, "pin_motor_rev_ch", cfg_pins_.motor_rev_ch);
    cfg_pins_.servo = parse_param(info, "pin_servo", cfg_pins_.servo);
    cfg_pins_.servo_ch = parse_param(info, "pin_servo_ch", cfg_pins_.servo_ch);
    cfg_pins_.hall = parse_param(info, "pin_hall", cfg_pins_.hall);

    // Lendo Cinemática e PID
    cfg_kinematics_.wheel_radius =
        parse_param(info, "wheel_radius", cfg_kinematics_.wheel_radius);
    cfg_kinematics_.gear_ratio =
        parse_param(info, "gear_ratio", cfg_kinematics_.gear_ratio);
    cfg_kinematics_.magnets_per_rev =
        parse_param(info, "magnets_per_rev", cfg_kinematics_.magnets_per_rev);
    cfg_pid_.kp = parse_param(info, "pid_kp", cfg_pid_.kp);
    cfg_pid_.ki = parse_param(info, "pid_ki", cfg_pid_.ki);
    cfg_pid_.kd = parse_param(info, "pid_kd", cfg_pid_.kd);
    cfg_pid_.filter_alpha =
        parse_param(info, "velocity_filter_alpha", cfg_pid_.filter_alpha);
    cfg_pid_.deadband_m_s =
        parse_param(info, "pid_deadband_m_s", cfg_pid_.deadband_m_s);

    // Lendo Motor
    cfg_motor_.period_ns =
        parse_param(info, "motor_period_ns", cfg_motor_.period_ns);
    cfg_motor_.min_effort =
        parse_param(info, "motor_min_effort", cfg_motor_.min_effort);
    cfg_motor_.deadband_ms = parse_param(info, "motor_deadband_ms",
                                         (int64_t)cfg_motor_.deadband_ms);
    cfg_motor_.invert_direction =
        parse_param(info, "motor_invert_direction", cfg_motor_.invert_direction);

    // Lendo Servo
    cfg_steer_.pwm_freq_hz = parse_param(info, "steering_pwm_freq_hz", cfg_steer_.pwm_freq_hz);
    cfg_steer_.center_pulse_ms = parse_param(info, "steering_center_pulse_ms", cfg_steer_.center_pulse_ms);
    cfg_steer_.max_deflection_pulse_ms = parse_param(info, "steering_max_deflection_ms", cfg_steer_.max_deflection_pulse_ms);
    cfg_steer_.max_angle_left_deg = parse_param(info, "max_steering_angle_left_deg", cfg_steer_.max_angle_left_deg);
    cfg_steer_.max_angle_right_deg = parse_param(info, "max_steering_angle_right_deg", cfg_steer_.max_angle_right_deg);
    cfg_steer_.trim_deg = parse_param(info, "steering_trim_deg", cfg_steer_.trim_deg);
    cfg_steer_.max_speed_deg_per_sec = parse_param(info, "steering_max_speed_deg_per_sec", cfg_steer_.max_speed_deg_per_sec);
    cfg_steer_.invert_direction = parse_param(info, "steering_invert_direction", cfg_steer_.invert_direction);

    current_steering_deg_ = 0.0;
    deadband_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);


    pid_controller_.initPid(cfg_pid_.kp, cfg_pid_.ki, cfg_pid_.kd, 1.0, -1.0,
                            true);
    new_kp_ = cfg_pid_.kp;
    new_ki_ = cfg_pid_.ki;
    new_kd_ = cfg_pid_.kd;

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
SpeedyHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[0].name, hardware_interface::HW_IF_VELOCITY,
        &hw_velocity_state_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[1].name, hardware_interface::HW_IF_POSITION,
        &hw_steering_state_));
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
SpeedyHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[0].name, hardware_interface::HW_IF_VELOCITY,
        &hw_velocity_cmd_));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[1].name, hardware_interface::HW_IF_POSITION,
        &hw_steering_cmd_));
    return command_interfaces;
}

hardware_interface::CallbackReturn SpeedyHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(logger_, "[HARDWARE] Activating Direct RPi PWM Hardware Interface...");

    // Inicializa PWMs
    if (!pwm_motor_fwd_.init(cfg_pins_.motor_fwd_ch, cfg_motor_.period_ns,
                             "Motor_FWD"))
        RCLCPP_ERROR(logger_, "FWD PWM Init Failed");
    if (!pwm_motor_rev_.init(cfg_pins_.motor_rev_ch, cfg_motor_.period_ns,
                             "Motor_REV"))
        RCLCPP_ERROR(logger_, "REV PWM Init Failed");
    
    int64_t servo_period_ns = static_cast<int64_t>(1e9 / cfg_steer_.pwm_freq_hz);
    if (!pwm_servo_.init(cfg_pins_.servo_ch, servo_period_ns, "Servo"))
        RCLCPP_ERROR(logger_, "Servo PWM Init Failed");

    // Parâmetros Dinâmicos
    param_node_ = std::make_shared<rclcpp::Node>("speedy_hardware");
    param_node_->declare_parameter("pid_kp", cfg_pid_.kp);
    param_node_->declare_parameter("pid_ki", cfg_pid_.ki);
    param_node_->declare_parameter("pid_kd", cfg_pid_.kd);
    param_node_->declare_parameter("velocity_filter_alpha",
                                   cfg_pid_.filter_alpha);
    param_node_->declare_parameter("pid_deadband_m_s", cfg_pid_.deadband_m_s);
    param_node_->declare_parameter("motor_min_effort", cfg_motor_.min_effort);
    param_node_->declare_parameter("motor_deadband_ms",
                                   static_cast<int>(cfg_motor_.deadband_ms));
    param_node_->declare_parameter("steering_center_pulse_ms", cfg_steer_.center_pulse_ms);
    param_node_->declare_parameter("steering_max_deflection_ms", cfg_steer_.max_deflection_pulse_ms);
    param_node_->declare_parameter("steering_max_speed_deg_per_sec", cfg_steer_.max_speed_deg_per_sec);
    param_node_->declare_parameter("max_steering_angle_left_deg", cfg_steer_.max_angle_left_deg);
    param_node_->declare_parameter("max_steering_angle_right_deg", cfg_steer_.max_angle_right_deg);
    param_node_->declare_parameter("steering_trim_deg", cfg_steer_.trim_deg);
    param_node_->declare_parameter("motor_invert_direction", cfg_motor_.invert_direction);
    param_node_->declare_parameter("steering_invert_direction", cfg_steer_.invert_direction);
    param_node_->declare_parameter("wheel_radius", cfg_kinematics_.wheel_radius);
    param_node_->declare_parameter("gear_ratio", cfg_kinematics_.gear_ratio);
    param_node_->declare_parameter("magnets_per_rev", cfg_kinematics_.magnets_per_rev);

    param_cb_handle_ = param_node_->add_on_set_parameters_callback(
        std::bind(&SpeedyHardwareInterface::param_callback, this,
                  std::placeholders::_1));

    stop_threads_ = false;
    param_thread_ = std::thread(
        [this]()
        {
            rclcpp::executors::SingleThreadedExecutor exec;
            exec.add_node(param_node_);
            while (!stop_threads_ && rclcpp::ok())
            {
                exec.spin_once(std::chrono::milliseconds(100));
            }
        });

    // Sensor Hall (Raspberry Pi 5 RP1)
    try
    {
        std::string chip_name = "/dev/gpiochip4";
        auto chip = gpiod::chip(chip_name);

        // Ativa os Pinos de Enable do Motor (EN1 e EN2)
        en_request_ = std::make_unique<gpiod::line_request>(
            chip.prepare_request()
                .set_consumer("speedy_en1")
                .add_line_settings(
                    cfg_pins_.motor_en_1,
                    gpiod::line_settings()
                        .set_direction(gpiod::line::direction::OUTPUT)
                        .set_output_value(gpiod::line::value::ACTIVE))
                .do_request());

        en2_request_ = std::make_unique<gpiod::line_request>(
            chip.prepare_request()
                .set_consumer("speedy_en2")
                .add_line_settings(
                    cfg_pins_.motor_en_2,
                    gpiod::line_settings()
                        .set_direction(gpiod::line::direction::OUTPUT)
                        .set_output_value(gpiod::line::value::ACTIVE))
                .do_request());

        hall_request_ = std::make_unique<gpiod::line_request>(
            chip.prepare_request()
                .set_consumer("speedy_hall")
                .add_line_settings(
                    cfg_pins_.hall,
                    gpiod::line_settings()
                        .set_direction(gpiod::line::direction::INPUT)
                        .set_edge_detection(gpiod::line::edge::RISING)
                        .set_bias(gpiod::line::bias::PULL_UP))
                .do_request());

        hall_thread_ =
            std::thread(&SpeedyHardwareInterface::hall_interrupt_loop, this);
        RCLCPP_INFO(logger_, "[HARDWARE] Motor Enable (GPIO %d and %d) and Hall sensor (GPIO %d) active on %s",
                    cfg_pins_.motor_en_1, cfg_pins_.motor_en_2, cfg_pins_.hall, chip_name.c_str());
    }

    catch (const std::exception& e)
    {
        RCLCPP_ERROR(logger_, "Hall init failed: %s", e.what());
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SpeedyHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    pwm_motor_fwd_.stop();
    pwm_motor_rev_.stop();
    pwm_servo_.stop();

    stop_threads_ = true;
    if (param_thread_.joinable())
        param_thread_.join();
    if (hall_thread_.joinable())
        hall_thread_.join();
    if (hall_request_)
        try
        {
            hall_request_->release();
        }
        catch (...)
        {
        }
    if (en_request_)
        try
        {
            en_request_->release();
        }
        catch (...)
        {
        }
    if (en2_request_)
        try
        {
            en2_request_->release();
        }
        catch (...)
        {
        }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
SpeedyHardwareInterface::read(const rclcpp::Time& time,
                              const rclcpp::Duration& period)
{
    (void)time;
    double dt = period.seconds();
    if (dt <= 0)
        dt = 0.02;

    int64_t current_pulses = hall_pulse_count_.load();
    int64_t delta_pulses = current_pulses - prev_pulse_count_;
    prev_pulse_count_ = current_pulses;

    double distance_per_pulse =
        (2.0 * M_PI * cfg_kinematics_.wheel_radius) /
        (cfg_kinematics_.gear_ratio * cfg_kinematics_.magnets_per_rev);
    double raw_velocity = (delta_pulses * distance_per_pulse) / dt;
    filtered_velocity_ = (cfg_pid_.filter_alpha * raw_velocity) +
                         ((1.0 - cfg_pid_.filter_alpha) * filtered_velocity_);

    hw_velocity_state_ =
        (last_direction_ < 0) ? -filtered_velocity_ : filtered_velocity_;

    hw_steering_state_ = hw_steering_cmd_;
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
SpeedyHardwareInterface::write(const rclcpp::Time& time,
                               const rclcpp::Duration& period)
{
    if (update_pid_.exchange(false))
    {
        cfg_pid_.kp = new_kp_;
        cfg_pid_.ki = new_ki_;
        cfg_pid_.kd = new_kd_;
        pid_controller_.setGains(cfg_pid_.kp, cfg_pid_.ki, cfg_pid_.kd, 1.0,
                                 -1.0, true);
        pid_controller_.reset();
    }

    // Função auxiliar para validar comandos (limpa NaN e Infinitos provenientes de transições)
    auto safe_val = [](double val) -> double {
        return std::isfinite(val) ? val : 0.0;
    };

    double current_velocity_cmd = safe_val(hw_velocity_cmd_);
    double current_steering_cmd = safe_val(hw_steering_cmd_);

    // ====== MOTOR ======
    double target_velocity = std::clamp(current_velocity_cmd, -1.0, 1.0);
    double effort = 0.0;
    bool skip_motor_write = false;

    if (std::abs(target_velocity) >= cfg_pid_.deadband_m_s)
    {
        double error = target_velocity - hw_velocity_state_;
        effort = target_velocity +
                 pid_controller_.computeCommand(
                     error, period.nanoseconds() ? period.nanoseconds() : 20000000);
    }
    else
    {
        pid_controller_.reset();
    }

    effort = std::clamp(effort, -1.0, 1.0);
    if (cfg_motor_.invert_direction)
        effort = -effort;

    if (std::abs(effort) < cfg_motor_.min_effort)
    {
        pwm_motor_fwd_.set_duty_ns(0);
        pwm_motor_rev_.set_duty_ns(0);
        last_direction_ = 0;
        in_deadband_ = false;
    }
    else
    {
        int8_t target_direction = (effort > 0) ? 1 : -1;
        if (target_direction != last_direction_ && last_direction_ != 0)
        {
            if (!in_deadband_)
            {
                pwm_motor_fwd_.set_duty_ns(0);
                pwm_motor_rev_.set_duty_ns(0);
                in_deadband_ = true;
                deadband_start_time_ = time;
            }
            else if ((time - deadband_start_time_).seconds() >=
                     (cfg_motor_.deadband_ms / 1000.0))
            {
                in_deadband_ = false;
            }
            else
            {
                // Estamos no dead-time da ponte H. 
                // Apenas marcamos para nao escrever no motor, mas CONTINUAMOS pro servo.
                skip_motor_write = true; 
            }
        }
        else
        {
            in_deadband_ = false;
        }

        if (!skip_motor_write) {
            last_direction_ = target_direction;
            int64_t duty = static_cast<int64_t>(std::abs(effort) * cfg_motor_.period_ns);
            if (target_direction > 0)
            {
                pwm_motor_rev_.set_duty_ns(0);
                pwm_motor_fwd_.set_duty_ns(duty);
            }
            else
            {
                pwm_motor_fwd_.set_duty_ns(0);
                pwm_motor_rev_.set_duty_ns(duty);
            }
        }
    }

    // ====== SERVO (AGORA CONFIGURADO EM GRAUS/MS) ======
    // O comando do ROS 2 chega em radianos. Convertemos para graus:
    double steer_cmd_deg = current_steering_cmd * (180.0 / M_PI);
    
    if (cfg_steer_.invert_direction)
        steer_cmd_deg = -steer_cmd_deg;

    // Limitamos o comando e aplicamos o trim (Tudo em graus!)
    double max_angle = (steer_cmd_deg >= 0) ? cfg_steer_.max_angle_left_deg : cfg_steer_.max_angle_right_deg;
    if (max_angle < 0.1) max_angle = 0.1; // Evita divisão por zero

    steer_cmd_deg = std::clamp(steer_cmd_deg, -max_angle, max_angle);
    double target_deg = steer_cmd_deg + cfg_steer_.trim_deg;

    // Aplicamos limite de velocidade angular em graus/segundo (se configurada)
    if (cfg_steer_.max_speed_deg_per_sec <= 0.0)
    {
        current_steering_deg_ = target_deg;
    }
    else
    {
        double max_delta_deg = cfg_steer_.max_speed_deg_per_sec * period.seconds();
        double error_deg = target_deg - current_steering_deg_;
        current_steering_deg_ += std::clamp(error_deg, -max_delta_deg, max_delta_deg);
    }

    // Convertemos de GRAUS para pulso em MILISSEGUNDOS (ms)
    // A proporção linear é baseada na deflexão máxima configurada.
    double target_pulse_ms = cfg_steer_.center_pulse_ms + 
                             (current_steering_deg_ / max_angle) * cfg_steer_.max_deflection_pulse_ms;

    // Trava de Segurança Crítica: Garante que o pulso nunca exceda os limites físicos do servo (mesmo com trim alto)
    double min_safe_pulse = cfg_steer_.center_pulse_ms - cfg_steer_.max_deflection_pulse_ms;
    double max_safe_pulse = cfg_steer_.center_pulse_ms + cfg_steer_.max_deflection_pulse_ms;
    target_pulse_ms = std::clamp(target_pulse_ms, min_safe_pulse, max_safe_pulse);

    // Finalmente, convertemos de Milissegundos (ms) para Nanosegundos (ns) para enviar ao sysfs
    int64_t target_duty_ns = static_cast<int64_t>(target_pulse_ms * 1000000.0);

    if (!pwm_servo_.set_duty_ns(target_duty_ns)) {
        RCLCPP_ERROR_THROTTLE(logger_, *param_node_->get_clock(), 1000, 
                              "SysFS Write Failed! Check Servo PWM driver.");
    }

    return hardware_interface::return_type::OK;
}

void SpeedyHardwareInterface::hall_interrupt_loop()
{
    while (!stop_threads_ && rclcpp::ok())
    {
        if (hall_request_ &&
            hall_request_->wait_edge_events(std::chrono::milliseconds(100)))
        {
            gpiod::edge_event_buffer events;
            hall_request_->read_edge_events(events, 1);
            hall_pulse_count_.fetch_add(1, std::memory_order_relaxed);
        }
    }
}

rcl_interfaces::msg::SetParametersResult SpeedyHardwareInterface::param_callback(
    const std::vector<rclcpp::Parameter>& params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto& param : params)
    {
        if (param.get_name() == "pid_kp")
        {
            new_kp_ = param.as_double();
            update_pid_ = true;
        }
        else if (param.get_name() == "pid_ki")
        {
            new_ki_ = param.as_double();
            update_pid_ = true;
        }
        else if (param.get_name() == "pid_kd")
        {
            new_kd_ = param.as_double();
            update_pid_ = true;
        }
        else if (param.get_name() == "velocity_filter_alpha")
            cfg_pid_.filter_alpha = param.as_double();
        else if (param.get_name() == "pid_deadband_m_s")
            cfg_pid_.deadband_m_s = param.as_double();
        else if (param.get_name() == "motor_min_effort")
            cfg_motor_.min_effort = param.as_double();
        else if (param.get_name() == "motor_deadband_ms")
            cfg_motor_.deadband_ms = param.as_int();
        else if (param.get_name() == "steering_center_pulse_ms")
            cfg_steer_.center_pulse_ms = param.as_double();
        else if (param.get_name() == "steering_max_deflection_ms")
            cfg_steer_.max_deflection_pulse_ms = param.as_double();
        else if (param.get_name() == "steering_max_speed_deg_per_sec")
            cfg_steer_.max_speed_deg_per_sec = param.as_double();
        else if (param.get_name() == "max_steering_angle_left_deg")
            cfg_steer_.max_angle_left_deg = param.as_double();
        else if (param.get_name() == "max_steering_angle_right_deg")
            cfg_steer_.max_angle_right_deg = param.as_double();
        else if (param.get_name() == "steering_trim_deg")
            cfg_steer_.trim_deg = param.as_double();
        else if (param.get_name() == "motor_invert_direction")
            cfg_motor_.invert_direction = param.as_bool();
        else if (param.get_name() == "steering_invert_direction")
            cfg_steer_.invert_direction = param.as_bool();
        else if (param.get_name() == "wheel_radius")
            cfg_kinematics_.wheel_radius = param.as_double();
        else if (param.get_name() == "gear_ratio")
            cfg_kinematics_.gear_ratio = param.as_double();
        else if (param.get_name() == "magnets_per_rev")
            cfg_kinematics_.magnets_per_rev = param.as_int();
    }
    return result;
}

} // namespace speedy_control
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(speedy_control::SpeedyHardwareInterface,
                       hardware_interface::SystemInterface)
