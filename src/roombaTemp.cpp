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
 * @file roombaTemp.cpp
 * @author Naveen Mangla (nmangla@umd.edu)
 * @brief Roomba Walker, avoid obstacles if any 
 * @version 0.1
 * @date 2022-12-05
 * @copyright Copyright (c) 2022
 * 
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>


using std::placeholders::_1;
using namespace std::chrono_literals;

using LASER = sensor_msgs::msg::LaserScan;
using TWIST = geometry_msgs::msg::Twist;


// Bot state 
typedef enum {
  FORWARD = 0,
  STOP,
  TURN,
} StateType;


class RoomBa : public rclcpp::Node {
 public:

  RoomBa() :
    Node("walker"),
    state_(FORWARD)
  {
    auto pubTopicName = "cmd_vel";
    publisher_ = this->create_publisher<TWIST> (pubTopicName, 10); 

    // Publisher callback 
    auto processCallback = std::bind(&RoomBa::callback, this);
    timer_ = this->create_wall_timer (100ms, processCallback); 

    auto subCallback = std::bind(&RoomBa::subscribe_callback, this, _1);
    auto default_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
    subscription_ = this->create_subscription<LASER>("scan", default_qos ,subCallback); // subscribing  to topic scan
    
  }
  private:
    /**
     * @brief Publisher callback function
     * 
     */
    void callback(){
      auto message = TWIST();
      
      if (laser_.header.stamp.sec == 0){
        message.linear.x = 0.3;
        publisher_->publish(message);  
        return;}
    // state machine (Mealy -- output on transition)
      switch (state_) {

      case FORWARD:
        message.linear.x = 0.3;
        RCLCPP_INFO(this->get_logger(), "Moving forward");
        
        if (hasObstacle()) {      // check transition
          state_ = STOP;
          publisher_->publish (message);
          RCLCPP_INFO_STREAM (this->get_logger(), "State = STOP");
        } 
        break;
      case STOP:
        RCLCPP_INFO(this->get_logger(), "Stopped");

        if (hasObstacle()) {    // check transition
          state_ = TURN;
          // message.linear.x = 0;
          message.angular.z = 0.3;
          publisher_->publish (message);
          RCLCPP_INFO_STREAM (this->get_logger(), "State = TURN");

        } else {
          state_ = FORWARD;
          message.linear.x = 0.3;
          publisher_->publish (message);
          RCLCPP_INFO_STREAM (this->get_logger(), "State = FORWARD");
        }
        break;
      case TURN:
        RCLCPP_INFO(this->get_logger(), "Turning");
        if (! hasObstacle()) {    // check transition
          state_ = FORWARD;
          message.linear.x = 0.3;
          publisher_->publish (message);
          RCLCPP_INFO_STREAM (this->get_logger(), "State = FORWARD");
        }
        break;
      }

      /**
       * @brief Subsciber callback
       * 
       */
      }
      void subscribe_callback(const LASER& msg) {
          laser_ = msg;
      }     
    /**
     * @brief Checking whether obstacle is present
     * 
     * @return true 
     * @return false 
     */
    bool hasObstacle(){
        for(int i{}; i < num_readings ;i++){
          // Here 1 is range to check
            if(laser_.ranges[i] < 1 || (laser_.ranges[359-i] < 1) ){
                auto dist = (laser_.ranges[i] + laser_.ranges[359-i])/2;
                RCLCPP_INFO(this->get_logger(), "Obstacle at  '%f'm ",dist);
                return true;
                
            } 
        } return false;
    }   


  rclcpp::Subscription<LASER>::SharedPtr subscription_;
  rclcpp::Publisher<TWIST>::SharedPtr    publisher_;
  rclcpp::TimerBase::SharedPtr           timer_;
  LASER                                  laser_; // laser message 
  StateType                              state_; // state
  int num_readings = 15;  // angle to check, -15 to 15
};

// Main function
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoomBa>());
  rclcpp::shutdown();
  return 0;
}