// Copyright 2023 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "kai/kai_hardware.hpp"
#include <string>
#include <vector>
#include <iostream> 
#include <curl/curl.h>
#include <cmath>
// Callback function for data received from the server
size_t WriteCallback(void *contents, size_t size, size_t nmemb, std::string *userp) {
    userp->append((char*)contents, size * nmemb);
    return size * nmemb;
}

double radiansToDegrees(double radians) {
    return radians * (180.0 / 3.14);
}

// Function to make a GET request to the specified URL
void makeRequest(const std::string& value_1 = "",const std::string& value_2 = "",const std::string& value_3 = "") {
    CURL *curl;
    CURLcode res;
    std::string readBuffer;

    curl = curl_easy_init();
    if(curl) {
        std::string url = "http://192.168.55.194:80/move?v1="+value_1+"&v2="+value_2+"&v3="+value_3;
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);

        // Perform the request, res will get the return code
        res = curl_easy_perform(curl);

        // Check for errors
        if(res != CURLE_OK)
            std::cerr << "curl_easy_perform() failed: " << curl_easy_strerror(res) << std::endl;
        else
            std::cout << "Response from server: " << readBuffer << std::endl;

        // Always cleanup
        curl_easy_cleanup(curl);
    }
}

namespace kai
{
CallbackReturn RobotSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // robot has 6 joints and 2 interfaces
  joint_position_.assign(6, 0);
  joint_position_command_.assign(6, 0);
  

  for (const auto & joint : info_.joints)
  {
    for (const auto & interface : joint.state_interfaces)
    {
      joint_interfaces[interface.name].push_back(joint.name);
    }
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RobotSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  int ind = 0;
  for (const auto & joint_name : joint_interfaces["position"])
  {
    state_interfaces.emplace_back(joint_name, "position", &joint_position_[ind++]);
  }


  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobotSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  int ind = 0;
  for (const auto & joint_name : joint_interfaces["position"])
  {
    command_interfaces.emplace_back(joint_name, "position", &joint_position_command_[ind++]);
  }


  return command_interfaces;
}

return_type RobotSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // TODO(pac48) set sensor_states_ values from subscriber

  // for (auto i = 0ul; i < joint_velocities_command_.size(); i++)
  // {
  //   joint_velocities_[i] = joint_velocities_command_[i];
  //   joint_position_[i] += joint_velocities_command_[i] * period.seconds();
  // }

  for (auto i = 0ul; i < joint_position_command_.size(); i++)
  {
    joint_position_[i] = joint_position_command_[i];
  }

  return return_type::OK;
}

return_type RobotSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
      std::cout << radiansToDegrees(joint_position_command_[0]) << " , "<<radiansToDegrees(joint_position_command_[1]) <<" , "<< radiansToDegrees(joint_position_command_[2]) << std::endl;
      makeRequest(std::to_string(radiansToDegrees(joint_position_command_[0])),std::to_string(radiansToDegrees(joint_position_command_[1])),std::to_string(radiansToDegrees(joint_position_command_[2])));  

  return return_type::OK;
}

}  // namespace kai

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  kai::RobotSystem, hardware_interface::SystemInterface)
