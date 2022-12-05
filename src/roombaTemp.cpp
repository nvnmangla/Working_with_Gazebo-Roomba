#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>


using std::placeholders::_1;
using namespace std::chrono_literals;

using LASER = sensor_msgs::msg::LaserScan;
using TWIST = geometry_msgs::msg::Twist;


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

    // Publisher callback function
    auto processCallback = std::bind(&RoomBa::callback, this);
    timer_ = this->create_wall_timer (100ms, processCallback);

    auto subCallback = std::bind(&RoomBa::subscribe_callback, this, _1);
    auto default_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
    subscription_ = this->create_subscription<LASER>("scan", default_qos ,subCallback);
    
  }
  private:
    void callback(){
      if (laser_.header.stamp.sec == 0)
        return;
       auto message = TWIST();
    // state machine (Mealy -- output on transition)
      switch (state_) {

      case FORWARD:
        message.linear.x = 0.1;
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
          message.linear.x = 0;
          message.angular.z = -0.1;
          publisher_->publish (message);
          RCLCPP_INFO_STREAM (this->get_logger(), "State = TURN");

        } else {
          state_ = FORWARD;
          message.linear.x = 0.1;
          publisher_->publish (message);
          RCLCPP_INFO_STREAM (this->get_logger(), "State = FORWARD");
        }
        break;
      case TURN:
        RCLCPP_INFO(this->get_logger(), "Turning");

        if (! hasObstacle()) {    // check transition
          state_ = FORWARD;
          message.linear.x = 0.1;
          publisher_->publish (message);
          RCLCPP_INFO_STREAM (this->get_logger(), "State = FORWARD");
        }
        break;
      }

      }
      void subscribe_callback(const LASER& msg) {
          laser_ = msg;
      }     

    bool hasObstacle(){
        
       
        // laser_.angle_increment = 0.1;
        


        for(int i{}; i < num_readings ;i++){
            if(laser_.ranges[i] < 2 || (laser_.ranges[359-i] < 2) ){
                RCLCPP_INFO(this->get_logger(), "Obstacle at '%f' '%f'",laser_.ranges[i],laser_.ranges[359-i]);
                return true;
                
            } 
        } return false;
    }   


  rclcpp::Subscription<LASER>::SharedPtr subscription_;
  rclcpp::Publisher<TWIST>::SharedPtr    publisher_;
  rclcpp::TimerBase::SharedPtr           timer_;
  LASER                                  laser_;
  StateType                              state_;
  int num_readings = 15;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoomBa>());
  rclcpp::shutdown();
  return 0;
}