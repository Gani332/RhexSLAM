// giga_hardware_interface.cpp

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <chrono>
#include <string>
#include <vector>
#include <memory>
#include <fstream>
#include <iostream>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

using namespace hardware_interface;

class GigaHardwareInterface : public SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
  {
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
      return CallbackReturn::ERROR;
    }

    last_joint_commands_.resize(info.joints.size(), 0.0);
    joint_positions_.resize(info.joints.size(), 0.0);
    joint_commands_.resize(info.joints.size(), 0.0);

    // Open serial port
    serial_fd_ = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("GigaHW"), "Failed to open serial port: %s", strerror(errno));
    return CallbackReturn::ERROR;
    }


    struct termios tty{};
    if (tcgetattr(serial_fd_, &tty) != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("GigaHW"), "Failed to get termios");
      return CallbackReturn::ERROR;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tcsetattr(serial_fd_, TCSANOW, &tty);

    return CallbackReturn::SUCCESS;
  }

  std::vector<StateInterface> export_state_interfaces() override
  {
    std::vector<StateInterface> states;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      states.emplace_back(StateInterface(info_.joints[i].name, HW_IF_POSITION, &joint_positions_[i]));
    }
    return states;
  }

  std::vector<CommandInterface> export_command_interfaces() override
  {
    std::vector<CommandInterface> cmds;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      cmds.emplace_back(CommandInterface(info_.joints[i].name, HW_IF_VELOCITY, &joint_commands_[i]));
    }
    return cmds;
  }
return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
{
  char buffer[256];
  int bytes_read = ::read(serial_fd_, buffer, sizeof(buffer) - 1);
  if (bytes_read <= 0) {
    RCLCPP_WARN(rclcpp::get_logger("GigaHW"), "No data read from serial.");
    return return_type::OK;
  }

  buffer[bytes_read] = '\0';
  serial_buffer_ += buffer;

  // Check for complete line
  size_t newline_pos = serial_buffer_.find('\n');
  if (newline_pos == std::string::npos) {
    return return_type::OK;  // Wait for more data
  }

  std::string line = serial_buffer_.substr(0, newline_pos);
  serial_buffer_.erase(0, newline_pos + 1);  // remove the line we just read
  RCLCPP_INFO(rclcpp::get_logger("GigaHW"), "Raw line: %s", line.c_str());

  std::stringstream ss(line);
  std::string token;
  std::map<std::string, double> parsed_angles;

  while (std::getline(ss, token, ',')) {
    size_t colon_pos = token.find(':');
    if (colon_pos == std::string::npos) {
      RCLCPP_WARN(rclcpp::get_logger("GigaHW"), "Parse failed on token: %s", token.c_str());
      continue;
    }

    std::string name = token.substr(0, colon_pos);
    std::string value = token.substr(colon_pos + 1);

    try {
      double angle = std::stod(value);
      parsed_angles[name] = angle;
    } catch (const std::exception &e) {
      RCLCPP_WARN(rclcpp::get_logger("GigaHW"), "Failed to parse value for %s: %s", name.c_str(), e.what());
    }
  }

  auto set_if_found = [&](const std::string &label, const std::string &joint_name) {
    auto it = parsed_angles.find(label);
    if (it != parsed_angles.end()) {
      for (size_t i = 0; i < info_.joints.size(); ++i) {
        if (info_.joints[i].name == joint_name) {
          joint_positions_[i] = it->second;
          break;
        }
      }
    }
  };

  set_if_found("L1", "front_left_leg_joint");
  set_if_found("L2", "centre_left_leg_joint");
  set_if_found("L3", "back_left_leg_joint");
  set_if_found("R1", "front_right_leg_joint");
  set_if_found("R2", "centre_right_leg_joint");
  set_if_found("R3", "back_right_leg_joint");
  return return_type::OK;
}

return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
{
  std::ostringstream ss;

  for (size_t i = 0; i < joint_commands_.size(); ++i) {
    double velocity = joint_commands_[i];
    int speed = static_cast<int>(velocity * 800.0);  // scale to motor units
    speed = std::clamp(speed, -800, 800);
    ss << speed;
    if (i != joint_commands_.size() - 1) ss << ",";
  }

  ss << "\n";
  std::string out = ss.str();
  ::write(serial_fd_, out.c_str(), out.size());

  return return_type::OK;
}


private:
  int serial_fd_{-1};
  std::vector<double> joint_positions_;
  std::vector<double> joint_commands_;
  std::vector<double> last_joint_commands_;
  std::string serial_buffer_;
};

PLUGINLIB_EXPORT_CLASS(GigaHardwareInterface, hardware_interface::SystemInterface)
