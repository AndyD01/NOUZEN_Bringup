#include "nouzen_hardware_interface.hpp"
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>
#include <sstream>
#include <algorithm>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/select.h>

namespace nouzen_bringup
{

hardware_interface::CallbackReturn NouzenHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get parameters from URDF
  serial_port_ = info_.hardware_parameters["serial_port"];
  serial_baud_ = std::stoi(info_.hardware_parameters["serial_baud"]);
  loop_rate_ = std::stod(info_.hardware_parameters.count("loop_rate") ? 
                         info_.hardware_parameters["loop_rate"] : "30.0");

  // Initialize PID parameters
  kp_ = std::stod(info_.hardware_parameters.count("kp") ? 
                  info_.hardware_parameters["kp"] : "180.0");
  kd_ = std::stod(info_.hardware_parameters.count("kd") ? 
                  info_.hardware_parameters["kd"] : "60.0");
  ki_ = std::stod(info_.hardware_parameters.count("ki") ? 
                  info_.hardware_parameters["ki"] : "0.0");
  ko_ = std::stod(info_.hardware_parameters.count("ko") ? 
                  info_.hardware_parameters["ko"] : "50.0");

  // Initialize core joint storage (4 motors)
  hw_commands_.resize(info_.joints.size(), 0.0);
  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);
  hw_efforts_.resize(info_.joints.size(), 0.0);
  last_encoder_ticks_.resize(info_.joints.size(), 0);
  joint_configs_.resize(info_.joints.size());
  
  // Initialize telemetry storage
  battery_voltages_.resize(1, 0.0);
  motor_speeds_.resize(info_.joints.size(), 0.0);
  error_status_.resize(1, 0);

  // Initialize serial port as closed
  fd_serial_ = -1;
  first_read_ = true;

  // Configure each joint
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto& joint = info_.joints[i];
    
    if (joint.parameters.find("encoder_ticks_per_revolution") != joint.parameters.end()) {
        joint_configs_[i].encoder_ticks_per_revolution = 
            std::stoi(joint.parameters.at("encoder_ticks_per_revolution"));
    } else {
        joint_configs_[i].encoder_ticks_per_revolution = 1760;
    }
    
    if (joint.parameters.find("motor_index") != joint.parameters.end()) {
        joint_configs_[i].motor_index = std::stoi(joint.parameters.at("motor_index"));
    } else {
        joint_configs_[i].motor_index = i;
    }
    
    joint_configs_[i].joint_name = joint.name;

    RCLCPP_INFO(rclcpp::get_logger("NouzenHardwareInterface"), 
                "Joint %s: Motor=%d, TicksPerRev=%d", 
                joint.name.c_str(), 
                joint_configs_[i].motor_index,
                joint_configs_[i].encoder_ticks_per_revolution);
  }

  // Validate interfaces
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("NouzenHardwareInterface"),
        "Joint '%s' has %zu command interfaces. 1 expected.", 
        joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("NouzenHardwareInterface"),
        "Joint '%s' has '%s' command interface. '%s' expected.", 
        joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), 
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() < 3)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("NouzenHardwareInterface"),
        "Joint '%s' has %zu state interfaces. At least 3 expected.", 
        joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("NouzenHardwareInterface"), 
              "NOUZEN Hardware Interface initialized");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn NouzenHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  fd_serial_ = init_serial(serial_port_, serial_baud_);
  
  if (fd_serial_ == -1) {
    RCLCPP_ERROR(rclcpp::get_logger("NouzenHardwareInterface"), 
                 "Failed to open serial port: %s", serial_port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  // Salveaza valorile din URDF inainte de read
  double urdf_kp = kp_;
  double urdf_kd = kd_;
  double urdf_ki = ki_;
  double urdf_ko = ko_;

  if (read_pid_params()) {
    RCLCPP_INFO(rclcpp::get_logger("NouzenHardwareInterface"), 
                "ESP32 communication OK - PID: Kp=%.2f Kd=%.2f Ki=%.2f Ko=%.2f",
                kp_, kd_, ki_, ko_);
  } else {
    RCLCPP_WARN(rclcpp::get_logger("NouzenHardwareInterface"), 
                "Could not read PID params from ESP32, using defaults");
  }

  if (reset_encoders()) {
    RCLCPP_INFO(rclcpp::get_logger("NouzenHardwareInterface"), 
                "Encoders reset to zero");
  }

  // Trimite valorile din URDF, nu ce a returnat ESP32
  if (update_pid_params(urdf_kp, urdf_kd, urdf_ki, urdf_ko)) {
    RCLCPP_INFO(rclcpp::get_logger("NouzenHardwareInterface"),
                "PID sent to ESP32: Kp=%.2f Kd=%.2f Ki=%.2f Ko=%.2f",
                urdf_kp, urdf_kd, urdf_ki, urdf_ko);
  } else {
    RCLCPP_WARN(rclcpp::get_logger("NouzenHardwareInterface"),
                "Could not send PID to ESP32, ESP32 keeps its own values");
  }

  RCLCPP_INFO(rclcpp::get_logger("NouzenHardwareInterface"), 
              "Successfully configured");
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> NouzenHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, "speed_feedback", &motor_speeds_[i]));
  }
  
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "esp32_controller", "battery_voltage", &battery_voltages_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "esp32_controller", "error_status", &error_status_[0]));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> NouzenHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn NouzenHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (auto i = 0u; i < hw_commands_.size(); i++)
  {
    if (std::isnan(hw_commands_[i]))
    {
      hw_commands_[i] = 0.0;
    }
  }

  first_read_ = true;
  last_read_time_ = rclcpp::Clock().now();

  // Reduced delay for faster startup
  RCLCPP_INFO(rclcpp::get_logger("NouzenHardwareInterface"), 
              "Waiting for ESP32 to stabilize...");
  std::this_thread::sleep_for(std::chrono::milliseconds(200));  // 300ms -> 200ms
  
  tcflush(fd_serial_, TCIOFLUSH);
  std::this_thread::sleep_for(std::chrono::milliseconds(30));   // 50ms -> 30ms
  
  std::vector<int32_t> dummy_encoders(4, 0);
  if (read_encoders(dummy_encoders)) {
    RCLCPP_INFO(rclcpp::get_logger("NouzenHardwareInterface"), 
                "✅ Warm-up read SUCCESS: [%d %d %d %d]",
                dummy_encoders[0], dummy_encoders[1], 
                dummy_encoders[2], dummy_encoders[3]);
  } else {
    RCLCPP_WARN(rclcpp::get_logger("NouzenHardwareInterface"), 
                "⚠️ Warm-up read failed (will retry in main loop)");
  }

  RCLCPP_INFO(rclcpp::get_logger("NouzenHardwareInterface"), 
              "Successfully activated");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn NouzenHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::vector<int32_t> zero_speeds(4, 0);
  set_motor_speeds(zero_speeds);
  close_serial();

  RCLCPP_INFO(rclcpp::get_logger("NouzenHardwareInterface"), 
              "Successfully deactivated");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type NouzenHardwareInterface::read(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    std::vector<int32_t> encoder_ticks(4, 0);
    if (!read_encoders(encoder_ticks)) {
        static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
        RCLCPP_WARN_THROTTLE(
            rclcpp::get_logger("NouzenHardwareInterface"),
            steady_clock,
            5000,
            "Failed to read encoders from ESP32");
        return hardware_interface::return_type::ERROR;
    }

    double dt = period.seconds();
    
    for (size_t i = 0; i < joint_configs_.size(); i++) {
        int motor_idx = joint_configs_[i].motor_index;
        int32_t current_ticks = encoder_ticks[motor_idx];
        
        if (!first_read_) {
            int32_t delta_ticks = current_ticks - last_encoder_ticks_[i];
            
            if (std::abs(delta_ticks) > 1000000) {
                delta_ticks = 0;
            }
            
            if (dt > 0.001 && dt < 0.2) {
                double ticks_per_second = static_cast<double>(delta_ticks) / dt;
                double new_velocity = (ticks_per_second * 2.0 * M_PI) / 
                                     joint_configs_[i].encoder_ticks_per_revolution;
                
                if (std::abs(new_velocity) <= 50.0 && std::isfinite(new_velocity)) {
                    hw_velocities_[i] = new_velocity;
                }
                
                if (std::abs(hw_velocities_[i]) < 1e-6) {
                    hw_velocities_[i] = 0.0;
                }
                
                hw_positions_[i] += hw_velocities_[i] * dt;
            }
        } else {
            hw_velocities_[i] = 0.0;
        }
        
        last_encoder_ticks_[i] = current_ticks;
    }

    if (first_read_) {
        first_read_ = false;
        RCLCPP_INFO(rclcpp::get_logger("NouzenHardwareInterface"), 
                    "First encoder read complete");
    }
    
    last_read_time_ = time;
    
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type NouzenHardwareInterface::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
    std::vector<int32_t> motor_speeds_ticks(4, 0);
    
    for (size_t i = 0; i < joint_configs_.size(); i++) {
        int motor_idx = joint_configs_[i].motor_index;
        
        double clamped_velocity = hw_commands_[i];
        if (std::abs(hw_commands_[i]) > 15.0 || !std::isfinite(hw_commands_[i])) {
            clamped_velocity = std::copysign(15.0, hw_commands_[i]);
        }
        
        motor_speeds_ticks[motor_idx] = velocity_to_ticks_per_frame(
            clamped_velocity,
            joint_configs_[i].encoder_ticks_per_revolution,
            period.seconds()
        );
    }
    
    if (!set_motor_speeds(motor_speeds_ticks)) {
        static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
        RCLCPP_WARN_THROTTLE(
            rclcpp::get_logger("NouzenHardwareInterface"),
            steady_clock,
            5000,
            "Failed to send motor speeds to ESP32");
        return hardware_interface::return_type::ERROR;
    }
    
    return hardware_interface::return_type::OK;
}

int NouzenHardwareInterface::init_serial(const std::string& port_name, int baud_rate)
{
    int fd = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        RCLCPP_ERROR(rclcpp::get_logger("NouzenHardwareInterface"), 
                     "Failed to open serial port: %s", port_name.c_str());
        return -1;
    }

    fcntl(fd, F_SETFL, 0);

    struct termios options;
    tcgetattr(fd, &options);
    
    speed_t speed;
    switch(baud_rate) {
        case 115200: speed = B115200; break;
        case 57600: speed = B57600; break;
        case 38400: speed = B38400; break;
        case 19200: speed = B19200; break;
        case 9600: speed = B9600; break;
        default: speed = B115200; break;
    }
    
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 1;
    
    tcsetattr(fd, TCSANOW, &options);
    tcflush(fd, TCIOFLUSH);
    
    RCLCPP_INFO(rclcpp::get_logger("NouzenHardwareInterface"), 
                "Serial port %s opened successfully", port_name.c_str());
    return fd;
}

void NouzenHardwareInterface::close_serial()
{
    if (fd_serial_ != -1) {
        close(fd_serial_);
        fd_serial_ = -1;
    }
}

bool NouzenHardwareInterface::send_command(const std::string& cmd)
{
    if (fd_serial_ == -1) return false;
    
    tcflush(fd_serial_, TCOFLUSH);  // Flush output buffer only
    
    std::string full_cmd = cmd + "\n";
    ssize_t written = ::write(fd_serial_, full_cmd.c_str(), full_cmd.length());
    tcdrain(fd_serial_);  // Wait for transmission to complete
    
    return written == static_cast<ssize_t>(full_cmd.length());
}

std::string NouzenHardwareInterface::read_response()
{
    if (fd_serial_ == -1) return "";
    
    char buffer[256];
    std::string response;
    
    fd_set set;
    struct timeval timeout;
    FD_ZERO(&set);
    FD_SET(fd_serial_, &set);
    timeout.tv_sec = 0;
    timeout.tv_usec = 100000;  // REDUCE de la 150ms la 100ms (10ms)
    
    int rv = select(fd_serial_ + 1, &set, NULL, NULL, &timeout);
    if (rv <= 0) {
        return "";
    }
    
    ssize_t bytes_read = ::read(fd_serial_, buffer, sizeof(buffer) - 1);
    if (bytes_read > 0) {
        buffer[bytes_read] = '\0';
        response = std::string(buffer);
        
        while (!response.empty() && 
               (response.back() == '\n' || response.back() == '\r')) {
            response.pop_back();
        }
    }
    
    return response;
}

bool NouzenHardwareInterface::read_encoders(std::vector<int32_t>& encoder_values)
{
    // 2 attempts with 120ms timeout each
    for (int attempt = 0; attempt < 2; attempt++) {
        if (attempt > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        if (!send_command("e")) {
            continue;
        }
        
        std::string response = read_response();
        if (response.empty()) {
            continue;
        }
        
        // Remove "OK" if present
        size_t ok_pos = response.find("OK");
        if (ok_pos != std::string::npos) {
            response = response.substr(0, ok_pos);
        }
        
        // Trim whitespace
        response.erase(0, response.find_first_not_of(" \t\n\r"));
        response.erase(response.find_last_not_of(" \t\n\r") + 1);
        
        std::istringstream iss(response);
        encoder_values.resize(4);
        
        bool parse_success = true;
        for (int i = 0; i < 4; i++) {
            if (!(iss >> encoder_values[i])) {
                parse_success = false;
                break;
            }
        }
        
        if (parse_success) {
            return true;
        }
    }
    
    return false;
}

bool NouzenHardwareInterface::set_motor_speeds(const std::vector<int32_t>& speeds)
{
    if (speeds.size() != 4) return false;
    
    std::ostringstream oss;
    oss << "m " << speeds[0] << " " << speeds[1] << " " 
        << speeds[2] << " " << speeds[3];
    
    return send_command(oss.str());
}

bool NouzenHardwareInterface::reset_encoders()
{
    return send_command("r");
}

bool NouzenHardwareInterface::update_pid_params(double kp, double kd, double ki, double ko)
{
    std::ostringstream oss;
    oss << "u " << kp << " " << kd << " " << ki << " " << ko;
    
    if (send_command(oss.str())) {
        kp_ = kp;
        kd_ = kd;
        ki_ = ki;
        ko_ = ko;
        return true;
    }
    
    return false;
}

bool NouzenHardwareInterface::read_pid_params()
{
    if (!send_command("p")) {
        return false;
    }
    
    std::string response = read_response();
    if (response.empty()) {
        return false;
    }
    
    std::istringstream iss(response);
    double kp, kd, ki, ko;
    
    if (iss >> kp >> kd >> ki >> ko) {
        kp_ = kp;
        kd_ = kd;
        ki_ = ki;
        ko_ = ko;
        return true;
    }
    
    return false;
}

double NouzenHardwareInterface::ticks_to_radians(int32_t ticks, int ticks_per_rev)
{
    return (static_cast<double>(ticks) * 2.0 * M_PI) / static_cast<double>(ticks_per_rev);
}

int32_t NouzenHardwareInterface::velocity_to_ticks_per_frame(
    double velocity_rad_s, int ticks_per_rev, double dt)
{
    double ticks_per_second = (velocity_rad_s / (2.0 * M_PI)) * ticks_per_rev;
    int32_t ticks_per_frame = static_cast<int32_t>(ticks_per_second * dt);
    
    return ticks_per_frame;
}

}  // namespace nouzen_bringup

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    nouzen_bringup::NouzenHardwareInterface, hardware_interface::SystemInterface)