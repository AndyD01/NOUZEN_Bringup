#ifndef NOUZEN_HARDWARE_INTERFACE_HPP_
#define NOUZEN_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace nouzen_bringup
{

struct JointConfig {
   int encoder_ticks_per_revolution; 
   std::string joint_name;
   int motor_index;  // 0-3 for motors M1-M4
};

class NouzenHardwareInterface : public hardware_interface::SystemInterface
{
public:
 RCLCPP_SHARED_PTR_DEFINITIONS(NouzenHardwareInterface)

 hardware_interface::CallbackReturn on_init(
   const hardware_interface::HardwareInfo & info) override;

 hardware_interface::CallbackReturn on_configure(
   const rclcpp_lifecycle::State & previous_state) override;

 std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

 std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

 hardware_interface::CallbackReturn on_activate(
   const rclcpp_lifecycle::State & previous_state) override;

 hardware_interface::CallbackReturn on_deactivate(
   const rclcpp_lifecycle::State & previous_state) override;

 hardware_interface::return_type read(
   const rclcpp::Time & time, const rclcpp::Duration & period) override;

 hardware_interface::return_type write(
   const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
 // Parameters
 std::string serial_port_;  // ESP32 serial port (e.g., /dev/ttyUSB0)
 int serial_baud_;          // 115200
 double loop_rate_;         // Control loop frequency (Hz)
 
 // Joint configuration
 std::vector<JointConfig> joint_configs_;
 
 // Communication
 int fd_serial_;  // Serial file descriptor
 
 // Core joint states (4 motors)
 std::vector<double> hw_commands_;      // Commanded velocities (rad/s)
 std::vector<double> hw_positions_;     // Encoder positions (rad)
 std::vector<double> hw_velocities_;    // Measured velocities (rad/s)
 std::vector<double> hw_efforts_;       // Motor efforts (placeholder)
 
 // TELEMETRY STATE INTERFACES
 std::vector<double> battery_voltages_;      // Battery voltage
 std::vector<double> motor_speeds_;          // Motor speeds (ticks/frame)
 std::vector<double> error_status_;          // Error flags
 
 // Encoder tracking
 std::vector<int32_t> last_encoder_ticks_;   // Last encoder values
 rclcpp::Time last_read_time_;
 bool first_read_;
 
 // PID parameters (stored for reference)
 double kp_, kd_, ki_, ko_;
 
 // ESP32 Serial Protocol Commands
 static constexpr char CMD_READ_ENCODERS = 'e';
 static constexpr char CMD_SET_MOTOR_SPEEDS = 'm';
 static constexpr char CMD_SET_MOTOR_PWM = 'o';
 static constexpr char CMD_RESET_ENCODERS = 'r';
 static constexpr char CMD_UPDATE_PID = 'u';
 static constexpr char CMD_READ_PID = 'p';
 static constexpr char CMD_CALIBRATE = 'c';
 
 // Helper methods
 int init_serial(const std::string& port_name, int baud_rate);
 void close_serial();
 bool send_command(const std::string& cmd);
 std::string read_response();
 
 // ESP32 protocol methods
 bool read_encoders(std::vector<int32_t>& encoder_values);
 bool set_motor_speeds(const std::vector<int32_t>& speeds);
 bool reset_encoders();
 bool update_pid_params(double kp, double kd, double ki, double ko);
 bool read_pid_params();
 
 // Conversions
 double ticks_to_radians(int32_t ticks, int ticks_per_rev);
 int32_t velocity_to_ticks_per_frame(double velocity_rad_s, int ticks_per_rev, double dt);
};

}  // namespace nouzen_bringup

#endif  // NOUZEN_HARDWARE_INTERFACE_HPP_
