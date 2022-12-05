/*
Copyright 2022 Naveen Mangla
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

/**
 * @file roomba.cpp
 * @author Naveen Mangla (nmangla@umd.edu)
 * @brief Roomba Walker, avoid obstacles if any
 * @version 0.1
 * @date 2022-12-05
 * @copyright Copyright (c) 2022
 *
 */
#include <roomba.hpp>

// Main function
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoomBa>());
  rclcpp::shutdown();
  return 0;
}
