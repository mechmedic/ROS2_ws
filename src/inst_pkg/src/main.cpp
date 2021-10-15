#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "DxlMaster.h"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  DxlMaster dxl;  int p = 0;
  dxl.InitializeDriver(MODEMDEVICE,BAUDRATE,PROTOCOL_VERSION);
  getchar();
  dxl.GetDynamixelInfo(1);
  getchar();
  dxl.GetDynamixelInfo(2);
  getchar();
  dxl.SetJointMode(1,0,4095);
  getchar();
  dxl.SetJointMode(2,0,1023);
  getchar();

  printf("Enabling...\n");
  dxl.EnableTorque(1);
  getchar();
  printf("Enabling...\n");
  dxl.EnableTorque(2);

  printf("1...\n");
  dxl.SetJpos(1, 2000);
  getchar();
  dxl.GetJpos(1,p);
  printf("Position %d\n",p);
  printf("2...\n");
  dxl.SetJpos(2, 500);
  getchar();
  dxl.GetJpos(2,p);
  printf("Position %d\n",p);
  getchar();

  



  rclcpp::init(argc, argv);
  //rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  for(int i=1; i<3; i++)  {   dxl.DisableTorque(i);  }
  dxl.Disconnect();
  return 0;
}