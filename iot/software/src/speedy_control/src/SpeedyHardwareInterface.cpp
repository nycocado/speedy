#include "speedy_control/SpeedyHardwareInterface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <thread>

namespace speedy_control
{
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

    if (info_.joints.size() < 2)
    {
        RCLCPP_FATAL(logger_, "SpeedyHardwareInterface requires at least 2 joints (drive + steering).");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Lendo Pinos e Canais
    cfg_pins_.motor_en_1 = parse_param(info, "pin_motor_en_1", cfg_pins_.motor_en_1);
    cfg_pins_.motor_en_2 = parse_param(info, "pin_motor_en_2", cfg_pins_.motor_en_2);
    cfg_pins_.motor_fwd_ch =
        parse_param(info, "pin_motor_fwd_ch", cfg_pins_.motor_fwd_ch);
    cfg_pins_.motor_rev_ch =
        parse_param(info, "pin_motor_rev_ch", cfg_pins_.motor_rev_ch);
    cfg_pins_.servo = parse_param(info, "pin_servo", cfg_pins_.servo);
    cfg_pins_.hall = parse_param(info, "pin_hall", cfg_pins_.hall);
    cfg_pins_.hall_en = parse_param(info, "pin_hall_en", cfg_pins_.hall_en);
    cfg_pins_.hall_fl = parse_param(info, "pin_hall_fl", cfg_pins_.hall_fl);
    cfg_pins_.hall_fr = parse_param(info, "pin_hall_fr", cfg_pins_.hall_fr);

    // Lendo Cinemática e PID
    cfg_kinematics_.wheel_radius =
        parse_param(info, "wheel_radius", cfg_kinematics_.wheel_radius);
    cfg_kinematics_.wheelbase =
        parse_param(info, "wheelbase", cfg_kinematics_.wheelbase);
    cfg_kinematics_.gear_ratio =
        parse_param(info, "gear_ratio", cfg_kinematics_.gear_ratio);
    cfg_kinematics_.magnets_per_rev =
        parse_param(info, "magnets_per_rev", cfg_kinematics_.magnets_per_rev);
    cfg_kinematics_.front_magnets_per_rev =
        parse_param(info, "front_magnets_per_rev", cfg_kinematics_.front_magnets_per_rev);
    cfg_pid_.kp = parse_param(info, "pid_kp", cfg_pid_.kp);
    cfg_pid_.ki = parse_param(info, "pid_ki", cfg_pid_.ki);
    cfg_pid_.kd = parse_param(info, "pid_kd", cfg_pid_.kd);
    cfg_pid_.filter_alpha =
        parse_param(info, "velocity_filter_alpha", cfg_pid_.filter_alpha);
    cfg_pid_.deadband_m_s =
        parse_param(info, "pid_deadband_m_s", cfg_pid_.deadband_m_s);
    cfg_pid_.zero_timeout_s =
        parse_param(info, "velocity_zero_timeout_ms", 150.0) / 1000.0;

    // Lendo Motor
    cfg_motor_.period_ns =
        parse_param(info, "motor_period_ns", cfg_motor_.period_ns);
    cfg_motor_.min_effort =
        parse_param(info, "motor_min_effort", cfg_motor_.min_effort);
    cfg_motor_.deadband_ms = parse_param(info, "motor_deadband_ms",
                                         (int64_t)cfg_motor_.deadband_ms);
    cfg_motor_.invert_direction =
        parse_param(info, "motor_invert_direction", cfg_motor_.invert_direction);
    cfg_motor_.max_linear_velocity =
        parse_param(info, "max_linear_velocity", cfg_motor_.max_linear_velocity);
    cfg_motor_.velocity_feedforward =
        parse_param(info, "velocity_feedforward", cfg_motor_.velocity_feedforward);

    // Lendo Servo
    cfg_steer_.center_pulse_ms = parse_param(info, "steering_center_pulse_ms", cfg_steer_.center_pulse_ms);
    cfg_steer_.max_deflection_pulse_ms = parse_param(info, "steering_max_deflection_ms", cfg_steer_.max_deflection_pulse_ms);
    cfg_steer_.max_angle_left_deg = parse_param(info, "max_steering_angle_left_deg", cfg_steer_.max_angle_left_deg);
    cfg_steer_.max_angle_right_deg = parse_param(info, "max_steering_angle_right_deg", cfg_steer_.max_angle_right_deg);
    cfg_steer_.trim_deg = parse_param(info, "steering_trim_deg", cfg_steer_.trim_deg);
    cfg_steer_.max_speed_deg_per_sec = parse_param(info, "steering_max_speed_deg_per_sec", cfg_steer_.max_speed_deg_per_sec);
    cfg_steer_.invert_direction = parse_param(info, "steering_invert_direction", cfg_steer_.invert_direction);
    cfg_steer_.use_polynomial = parse_param(info, "steering_use_polynomial", cfg_steer_.use_polynomial);
    cfg_steer_.poly_coefs[0] = parse_param(info, "steering_poly_a0", cfg_steer_.poly_coefs[0]);
    cfg_steer_.poly_coefs[1] = parse_param(info, "steering_poly_a1", cfg_steer_.poly_coefs[1]);
    cfg_steer_.poly_coefs[2] = parse_param(info, "steering_poly_a2", cfg_steer_.poly_coefs[2]);

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
        info_.joints[0].name, hardware_interface::HW_IF_POSITION,
        &hw_drive_pos_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[1].name, hardware_interface::HW_IF_POSITION,
        &hw_steering_state_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "front_left_wheel_joint", hardware_interface::HW_IF_POSITION,
        &hw_pos_fl_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "front_left_wheel_joint", hardware_interface::HW_IF_VELOCITY,
        &hw_vel_fl_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "front_right_wheel_joint", hardware_interface::HW_IF_POSITION,
        &hw_pos_fr_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "front_right_wheel_joint", hardware_interface::HW_IF_VELOCITY,
        &hw_vel_fr_));
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

    // Inicializa PWMs do motor. Falha aqui é CRÍTICA: sem tração controlável a
    // ativação não pode prosseguir (evita um sistema "ativo" com motor morto).
    if (!motor_.init(cfg_pins_.motor_fwd_ch, cfg_pins_.motor_rev_ch,
                     cfg_motor_.period_ns))
    {
        RCLCPP_FATAL(logger_,
                     "[HARDWARE] Falha ao inicializar PWM do motor — abortando ativação.");
        motor_.stop();
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Parâmetros Dinâmicos
    param_node_ = std::make_shared<rclcpp::Node>("speedy_hardware");
    param_node_->declare_parameter("pid_kp", cfg_pid_.kp);
    param_node_->declare_parameter("pid_ki", cfg_pid_.ki);
    param_node_->declare_parameter("pid_kd", cfg_pid_.kd);
    param_node_->declare_parameter("velocity_filter_alpha",
                                   cfg_pid_.filter_alpha);
    param_node_->declare_parameter("pid_deadband_m_s", cfg_pid_.deadband_m_s);
    param_node_->declare_parameter("motor_min_effort", cfg_motor_.min_effort);
    param_node_->declare_parameter("max_linear_velocity", cfg_motor_.max_linear_velocity);
    param_node_->declare_parameter("velocity_feedforward", cfg_motor_.velocity_feedforward);
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
    param_node_->declare_parameter("steering_use_polynomial", cfg_steer_.use_polynomial);
    param_node_->declare_parameter("steering_poly_a0", cfg_steer_.poly_coefs[0]);
    param_node_->declare_parameter("steering_poly_a1", cfg_steer_.poly_coefs[1]);
    param_node_->declare_parameter("steering_poly_a2", cfg_steer_.poly_coefs[2]);
    param_node_->declare_parameter("wheel_radius", cfg_kinematics_.wheel_radius);
    param_node_->declare_parameter("gear_ratio", cfg_kinematics_.gear_ratio);
    param_node_->declare_parameter("magnets_per_rev", cfg_kinematics_.magnets_per_rev);

    front_odom_pub_ = param_node_->create_publisher<nav_msgs::msg::Odometry>(
        "/front_wheels/odom", rclcpp::QoS(10));
    servo_pulse_pub_ = param_node_->create_publisher<std_msgs::msg::Float64>(
        "/steering/pulse_ms", rclcpp::QoS(10));
    motor_cmd_pub_          = param_node_->create_publisher<std_msgs::msg::Float64>("/motor/cmd_vel",      rclcpp::QoS(10));
    motor_raw_vel_pub_      = param_node_->create_publisher<std_msgs::msg::Float64>("/motor/raw_vel",      rclcpp::QoS(10));
    motor_filtered_vel_pub_ = param_node_->create_publisher<std_msgs::msg::Float64>("/motor/filtered_vel", rclcpp::QoS(10));
    motor_effort_pub_       = param_node_->create_publisher<std_msgs::msg::Float64>("/motor/effort",       rclcpp::QoS(10));

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

    // Servo (pigpio, timing DMA). Falha NÃO é crítica: servo fica inativo (centrado).
    if (!servo_.init(cfg_pins_.servo))
    {
        RCLCPP_ERROR(logger_, "pigpio_start falhou — servo inativo (GPIO %d)",
                     cfg_pins_.servo);
    }
    else
    {
        RCLCPP_INFO(logger_, "[HARDWARE] Servo GPIO %d centrado (1500us)",
                    cfg_pins_.servo);
    }

    // Enable da ponte H (gpiod) — mantidos ativos enquanto o hardware estiver ativo.
    try
    {
        std::string chip_name = "/dev/gpiochip4";
        if (!std::filesystem::exists(chip_name))
            chip_name = "/dev/gpiochip0";
        auto chip = gpiod::chip(chip_name);

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

        RCLCPP_INFO(logger_, "[HARDWARE] Enable da ponte H ativo em %s — EN(%d,%d)",
                    chip_name.c_str(), cfg_pins_.motor_en_1, cfg_pins_.motor_en_2);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(logger_, "GPIO enable (gpiod) init failed: %s", e.what());
    }

    // Odometria Hall via libgpiod (thread dedicada no HallEncoder).
    // O enable (pin_hall_en) ativa todos os sensores Hall (motor + rodas dianteiras).
    if (!encoder_.start(cfg_pins_.hall, cfg_pins_.hall_en))
        RCLCPP_ERROR(logger_, "Hall encoder init failed (GPIO %d)", cfg_pins_.hall);
    else
        RCLCPP_INFO(logger_, "[HARDWARE] Hall encoder ativo (GPIO %d)", cfg_pins_.hall);

    if (!encoder_fl_.start(cfg_pins_.hall_fl))
        RCLCPP_ERROR(logger_, "Hall encoder dianteiro esquerdo init failed (GPIO %d)", cfg_pins_.hall_fl);
    else
        RCLCPP_INFO(logger_, "[HARDWARE] Hall encoder dianteiro esquerdo ativo (GPIO %d)", cfg_pins_.hall_fl);

    if (!encoder_fr_.start(cfg_pins_.hall_fr))
        RCLCPP_ERROR(logger_, "Hall encoder dianteiro direito init failed (GPIO %d)", cfg_pins_.hall_fr);
    else
        RCLCPP_INFO(logger_, "[HARDWARE] Hall encoder dianteiro direito ativo (GPIO %d)", cfg_pins_.hall_fr);

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SpeedyHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    motor_.stop();
    servo_.stop();
    encoder_.stop();
    encoder_fl_.stop();
    encoder_fr_.stop();
    front_odom_pub_.reset();
    servo_pulse_pub_.reset();
    motor_cmd_pub_.reset();
    motor_raw_vel_pub_.reset();
    motor_filtered_vel_pub_.reset();
    motor_effort_pub_.reset();

    stop_threads_ = true;
    if (param_thread_.joinable())
        param_thread_.join();

    auto release = [](std::unique_ptr<gpiod::line_request>& req)
    {
        if (req)
        {
            try
            {
                req->release();
            }
            catch (...)
            {
            }
            req.reset();
        }
    };
    release(en_request_);
    release(en2_request_);

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

    // Snapshot dos parâmetros usados aqui (protege contra escrita concorrente no param_callback).
    double gear_ratio, filter_alpha, wheel_radius, wheelbase, zero_timeout_s;
    int magnets_per_rev;
    {
        std::lock_guard<std::mutex> lock(cfg_mutex_);
        gear_ratio = cfg_kinematics_.gear_ratio;
        magnets_per_rev = cfg_kinematics_.magnets_per_rev;
        filter_alpha = cfg_pid_.filter_alpha;
        wheel_radius = cfg_kinematics_.wheel_radius;
        wheelbase = cfg_kinematics_.wheelbase;
        zero_timeout_s = cfg_pid_.zero_timeout_s;
    }
    // front_magnets_per_rev não é parâmetro dinâmico — leitura directa é segura.
    const int front_magnets = cfg_kinematics_.front_magnets_per_rev;

    // --- Roda traseira (motor) ---
    int64_t current_pulses = encoder_.pulses();
    int64_t delta_pulses = current_pulses - prev_pulse_count_;
    prev_pulse_count_ = current_pulses;

    const double rad_per_pulse =
        (2.0 * M_PI) / (gear_ratio * magnets_per_rev);
    double raw_velocity_rad_s = (delta_pulses * rad_per_pulse) / dt;
    filtered_velocity_ = filter_alpha * raw_velocity_rad_s + (1.0 - filter_alpha) * filtered_velocity_;

    // Determina a direção baseada no motor.
    // Se o motor está desligado (coasting), mantém o último sentido conhecido.
    int8_t motor_dir = motor_.direction();
    if (motor_dir != 0) {
        last_wheel_dir_ = static_cast<double>(motor_dir);
    }

    // Parada limpa: com o motor desligado e sem pulsos por > timeout, zera a estimativa
    // (corta a cauda da EMA). O gate em motor_dir==0 evita falso-zero no creep comandado,
    // onde os pulsos chegam mais espaçados que o timeout.
    t_idle_rear_ = (delta_pulses != 0) ? 0.0 : (t_idle_rear_ + dt);
    if (motor_dir == 0 && t_idle_rear_ >= zero_timeout_s)
        filtered_velocity_ = 0.0;

    // Estado expõe velocidade filtrada para suavidade no visual e controle.
    hw_velocity_state_ = last_wheel_dir_ * filtered_velocity_;
    hw_drive_pos_ += hw_velocity_state_ * dt;

    // --- Rodas dianteiras (passivas — medição directa, sem gear ratio) ---
    const double rad_per_pulse_front =
        (front_magnets > 0) ? (2.0 * M_PI) / front_magnets : 0.0;

    int64_t fl_pulses = encoder_fl_.pulses();
    int64_t fr_pulses = encoder_fr_.pulses();
    int64_t d_fl = fl_pulses - prev_pulse_fl_;
    int64_t d_fr = fr_pulses - prev_pulse_fr_;
    prev_pulse_fl_ = fl_pulses;
    prev_pulse_fr_ = fr_pulses;

    filtered_vel_fl_ = filter_alpha * (d_fl * rad_per_pulse_front / dt) + (1.0 - filter_alpha) * filtered_vel_fl_;
    filtered_vel_fr_ = filter_alpha * (d_fr * rad_per_pulse_front / dt) + (1.0 - filter_alpha) * filtered_vel_fr_;

    // Parada limpa por roda (mesmo critério da traseira; last_wheel_dir_ já vem do bloco acima).
    t_idle_fl_ = (d_fl != 0) ? 0.0 : (t_idle_fl_ + dt);
    t_idle_fr_ = (d_fr != 0) ? 0.0 : (t_idle_fr_ + dt);
    if (motor_dir == 0 && t_idle_fl_ >= zero_timeout_s) filtered_vel_fl_ = 0.0;
    if (motor_dir == 0 && t_idle_fr_ >= zero_timeout_s) filtered_vel_fr_ = 0.0;

    hw_vel_fl_ = last_wheel_dir_ * filtered_vel_fl_;
    hw_vel_fr_ = last_wheel_dir_ * filtered_vel_fr_;
    hw_pos_fl_ += hw_vel_fl_ * dt;
    hw_pos_fr_ += hw_vel_fr_ * dt;

    // Proteção contra NaN/Inf na leitura da direção (evita quebrar o TF tree)
    if (std::isfinite(hw_steering_cmd_)) {
        hw_steering_state_ = hw_steering_cmd_;
    } else {
        hw_steering_state_ = 0.0;
    }

    // Publica odometria das rodas dianteiras no tópico /front_wheels/odom.
    // linear.x: velocidade linear média das rodas dianteiras (m/s)
    // angular.z: curvatura derivada do modelo bicycle (v * tan(steer) / L)
    if (front_odom_pub_)
    {
        const double v_front =
            (hw_vel_fl_ + hw_vel_fr_) / 2.0 * wheel_radius;
        const double omega =
            (wheelbase > 1e-6)
                ? v_front * std::tan(hw_steering_state_) / wheelbase
                : 0.0;

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";
        odom_msg.twist.twist.linear.x = v_front;
        odom_msg.twist.twist.angular.z = omega;
        // Diagonal da covariância do twist. vx é medição real (rodas dianteiras).
        // angular.z é derivado do ângulo COMANDADO ao servo (não medido) → covariância
        // alta para o EKF não confiar nele; a rotação real vem do giroscópio/LiDAR.
        odom_msg.twist.covariance[0]  = 0.01;  // vx
        odom_msg.twist.covariance[7]  = 1e6;   // vy (não medido)
        odom_msg.twist.covariance[14] = 1e6;   // vz
        odom_msg.twist.covariance[21] = 1e6;   // wx
        odom_msg.twist.covariance[28] = 1e6;   // wy
        odom_msg.twist.covariance[35] = 1e6;   // wz (derivado do comando, não medido)
        front_odom_pub_->publish(odom_msg);
    }

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

    // Snapshot dos parâmetros dinâmicos (protege contra escrita concorrente no param_callback).
    decltype(cfg_motor_) motor_cfg;
    decltype(cfg_steer_) steer_cfg;
    double wheel_radius, deadband_m_s;
    {
        std::lock_guard<std::mutex> lock(cfg_mutex_);
        motor_cfg = cfg_motor_;
        steer_cfg = cfg_steer_;
        wheel_radius = cfg_kinematics_.wheel_radius;
        deadband_m_s = cfg_pid_.deadband_m_s;
    }

    // Função auxiliar para validar comandos (limpa NaN e Infinitos provenientes de transições)
    auto safe_val = [](double val) -> double {
        return std::isfinite(val) ? val : 0.0;
    };

    double current_velocity_cmd = safe_val(hw_velocity_cmd_);
    double current_steering_cmd = safe_val(hw_steering_cmd_);

    // ====== MOTOR (setpoint e feedback em rad/s) ======
    // hw_velocity_cmd_ é a velocidade angular da roda em rad/s (convenção ros2_control
    // para junta contínua). Backstop de segurança: limita à velocidade angular
    // correspondente à velocidade linear máxima (max_linear_velocity / wheel_radius).
    double omega_limit =
        (wheel_radius > 1e-6)
            ? (motor_cfg.max_linear_velocity / wheel_radius)
            : 1e9;
    double target_omega = std::clamp(current_velocity_cmd, -omega_limit, omega_limit);
    double effort = 0.0;

    // Feedback já está em rad/s (calculado em read()); aplica o sinal do sentido atual.
    double pid_velocity =
        (motor_.direction() < 0) ? -filtered_velocity_ : filtered_velocity_;
    // Deadband é configurado em m/s (intuitivo); converte para rad/s da roda.
    double deadband_omega = (wheel_radius > 1e-6)
                                ? (deadband_m_s / wheel_radius)
                                : 0.0;
    if (std::abs(target_omega) >= deadband_omega)
    {
        // Converte as velocidades de volta para linear (m/s) para o PID trabalhar
        // em uma escala intuitiva, correspondendo aos ganhos do hardware.yaml.
        double target_linear = target_omega * wheel_radius;
        double current_linear = pid_velocity * wheel_radius;
        double error_linear = target_linear - current_linear;

        // Feedforward: fração de duty (0..1) que produz a velocidade máxima, escalada
        // linearmente até o alvo. velocity_feedforward=0 → controle puramente PID.
        double feedforward =
            (omega_limit > 1e-6)
                ? motor_cfg.velocity_feedforward * (target_omega / omega_limit)
                : 0.0;
        effort = feedforward +
                 pid_controller_.computeCommand(
                     error_linear, period.nanoseconds() ? period.nanoseconds() : 20000000);
    }
    else
    {
        pid_controller_.reset();
    }

    effort = std::clamp(effort, -1.0, 1.0);

    if (motor_cmd_pub_) {
        std_msgs::msg::Float64 m;
        m.data = target_omega * wheel_radius;
        motor_cmd_pub_->publish(m);
    }
    if (motor_raw_vel_pub_) {
        std_msgs::msg::Float64 m;
        m.data = hw_velocity_state_ * wheel_radius;
        motor_raw_vel_pub_->publish(m);
    }
    if (motor_filtered_vel_pub_) {
        std_msgs::msg::Float64 m;
        m.data = pid_velocity * wheel_radius;
        motor_filtered_vel_pub_->publish(m);
    }
    if (motor_effort_pub_) {
        std_msgs::msg::Float64 m;
        m.data = effort;
        motor_effort_pub_->publish(m);
    }

    motor_.apply(effort, time, motor_cfg.deadband_ms, motor_cfg.min_effort,
                 motor_cfg.invert_direction);

    // ====== SERVO ======
    // O comando chega em rad. O ServoDriver faz conversão p/ graus, trim, limites,
    // rate-limit e mapeamento (linear/polinomial) com trava de segurança no pulso.
    ServoDriver::Config servo_cfg;
    servo_cfg.center_pulse_ms = steer_cfg.center_pulse_ms;
    servo_cfg.max_deflection_pulse_ms = steer_cfg.max_deflection_pulse_ms;
    servo_cfg.max_angle_left_deg = steer_cfg.max_angle_left_deg;
    servo_cfg.max_angle_right_deg = steer_cfg.max_angle_right_deg;
    servo_cfg.trim_deg = steer_cfg.trim_deg;
    servo_cfg.max_speed_deg_per_sec = steer_cfg.max_speed_deg_per_sec;
    servo_cfg.invert_direction = steer_cfg.invert_direction;
    servo_cfg.use_polynomial = steer_cfg.use_polynomial;
    servo_cfg.poly_coefs = steer_cfg.poly_coefs;
    servo_.apply(current_steering_cmd, period.seconds(), servo_cfg);

    if (servo_pulse_pub_)
    {
        std_msgs::msg::Float64 pulse_msg;
        pulse_msg.data = servo_.current_pulse_ms();
        servo_pulse_pub_->publish(pulse_msg);
    }

    return hardware_interface::return_type::OK;
}

rcl_interfaces::msg::SetParametersResult SpeedyHardwareInterface::param_callback(
    const std::vector<rclcpp::Parameter>& params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    // Serializa as escritas na config com as leituras (snapshot) de read()/write().
    // Os ganhos do PID continuam a usar o staging atómico (new_kp_/update_pid_).
    std::lock_guard<std::mutex> lock(cfg_mutex_);
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
        else if (param.get_name() == "max_linear_velocity")
            cfg_motor_.max_linear_velocity = param.as_double();
        else if (param.get_name() == "velocity_feedforward")
            cfg_motor_.velocity_feedforward = param.as_double();
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
        else if (param.get_name() == "steering_use_polynomial")
            cfg_steer_.use_polynomial = param.as_bool();
        else if (param.get_name() == "steering_poly_a0")
            cfg_steer_.poly_coefs[0] = param.as_double();
        else if (param.get_name() == "steering_poly_a1")
            cfg_steer_.poly_coefs[1] = param.as_double();
        else if (param.get_name() == "steering_poly_a2")
            cfg_steer_.poly_coefs[2] = param.as_double();
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
