#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "input_pkg/msg/haptic_cmd.hpp"


using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher")
    {
      // dxl = std::make_unique<DxlMaster>();
      // dxl->InitializeDriver(MODEMDEVICE,BAUDRATE,PROTOCOL_VERSION);
      // dxl->GetDynamixelInfo(1);
      // dxl->GetDynamixelInfo(2);
      // dxl->SetWheelMode(1);
      // dxl->SetWheelMode(2);

      // printf("Enabling...\n");
      // dxl->EnableTorque(1);
      // dxl->EnableTorque(2);

      // The "KEEP_LAST" history setting tells DDS to store a fixed-size buffer of values before they
      // are sent, to aid with recovery in the event of dropped messages.
      // "depth" specifies the size of this buffer.
      // In this example, we are optimizing for performance and limited resource usage (preventing
      // page faults), instead of reliability. Thus, we set the size of the history buffer to 1.
      auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
      
      // From http://www.opendds.org/qosusages.html: "A RELIABLE setting can potentially block while
      // trying to send." Therefore set the policy to best effort to avoid blocking during execution.
      qos.best_effort();
      joystick_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", qos, 
                                    std::bind(&MinimalPublisher::JstckCallbacks, this,std::placeholders::_1));
    }

  private:
    // CKim - Subscriber for joystick
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr  joystick_subscriber_;

    // CKim - Call back for joystick messages
    void JstckCallbacks(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
      //RCLCPP_INFO(this->get_logger(), "Jstck messages %.2f, %.2f", msg->axes[0], msg->axes[3]);
      //controller_.left_x_axis_  = msg->axes[0];       //controller_.left_y_axis_  = msg->axes[1];
      //controller_.right_x_axis_ = msg->axes[3];       //controller_.right_y_axis_ = msg->axes[4];

      // MX28 0~2047 (0X7FF) can be used, and the unit is about 0.114rpm.
      // If a value in the range of 0~1023 is used, it is stopped by setting to 0 while rotating to CCW direction.
      // If a value in the range of 1024~2047 is used, it is stopped by setting to 1024 while rotating to CW direction.
      // That is, the 10th bit becomes the direction bit to control the direction.
      int vel[2];     int Amp = 600;
      vel[0] = Amp*fabs(msg->axes[0]);
      vel[1] = Amp*fabs(msg->axes[3]);
      if(msg->axes[0] < 0)  {   vel[0] += 1024;   }   
      if(msg->axes[3] < 0)  {   vel[1] += 1024;   }   
      // dxl->SetVelAll(vel);
    }

    // std::unique_ptr<DxlMaster>  dxl;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  printf("Spinning ended\n");
  rclcpp::shutdown();
  // for(int i=1; i<3; i++)  {   dxl.DisableTorque(i);  }
  // dxl.Disconnect();
  return 0;
}








// -------------------------------

// -------------------------------------------------------- //
// CKim - This ROS node reads HapticDevice command from
// network and publishes it
// -------------------------------------------------------- //

// #include <stdio.h>
// #include <stdlib.h>
// #include <unistd.h>
// #include <string.h>
// #include <arpa/inet.h>
// #include <sys/socket.h>
// #include <sys/types.h>
// #include <netinet/in.h>
// #include <string>
// #include <time.h>
// #include "ros/ros.h" // ROS 기본 헤더 파일
// #include <SkillMate/HapticCommand.h>        // CKim - Automatically generated message Header

// #include <iostream>
// #include <fstream>

// using namespace std;

// int main(int argc, char* argv[])
// {
// 	int sock;
// 	struct sockaddr_in server_addr;

//     if(argc<3)
// 	{
//         printf("Usage : %s <IP> <port> \n", argv[3]);
// 		exit(1);
// 	 }

//     // Client Socket
// 	sock = socket(PF_INET, SOCK_STREAM, 0);

// 	//=======
//     ros::init(argc, argv, "HapticCmdPublisher");    //노드 초기화
//     ros::NodeHandle nh; //노드 핸들 선언

//     ros::Publisher haptic_pub =	nh.advertise<SkillMate::HapticCommand>("HapticCmd", 100);
//     ROS_INFO("Starting Haptic Node");
//     //====

// 	// Connect to address
// 	memset(&server_addr, 0, sizeof(server_addr));//서버 주소 초기화
//     server_addr.sin_family = AF_INET;
//     server_addr.sin_addr.s_addr = inet_addr(argv[1]);
//     server_addr.sin_port = htons(atoi(argv[2]));

//     ROS_INFO("Connecting to Server!\n");
// 	if(connect(sock, (struct sockaddr*)&server_addr, sizeof(server_addr))==-1)
// 	{
// 		printf("connect() error\n");
// 		close(sock);
// 		exit(1);
// 	}
//     ROS_INFO("Connected to Server!\n");

// 	//ros::Rate loop_rate(10);
// 	//ros::WallTime currTime = ros::WallTime::now();
// 	while(ros::ok())
// 	{
// 		// ----------------------------------------------------------- //
// 		// CKim - This code is using no protocol. Just 6 doubles
// 		// ----------------------------------------------------------- //
// 		int leng, real_recv_len, real_recv_byte;

//         leng = 6*sizeof(double) + 2*sizeof(int);
// 		char* str2 = new char[leng]; //길이 만큼 배열 동적할당

// 		if (str2 == (char *) 0) {
// 			printf("memory error!\n");
// 			exit(1);
// 		}

// 		memset(str2, 0, leng);
// 		real_recv_len = 0;
// 		real_recv_byte = 0;

// 		//받는 방법 2 : 받는 바이트 확인하며 받기
// 		while (real_recv_len < leng) {
// 			real_recv_byte = read(sock, &str2[real_recv_len],
// 					leng - real_recv_len);
// 			real_recv_len += real_recv_byte;
// 		}

// 		double* val = (double*) str2;
//         int* btn = (int*) (str2+6*sizeof(double));

//         SkillMate::HapticCommand msg;

// 		// CKim - First three element X, Y, Z increment. In mm
// 		// X: -/+ Left Right
// 		// Y: -/+ Down Up
// 		// Z: -/+ In / Out
// 		for(int i=0; i<3; i++)	{
// 			msg.array[i] = val[i];			}

// 		// CKim - Last three element Roll (Z) Pitch (X) Yaw (Y). In degree
// 		for(int i=3; i<6; i++)	{
//             msg.array[i] = val[i];  }//*2000.0;			}
// 		msg.btn[0] = btn[0];
// 		msg.btn[1] = btn[1];

// 		// ----------------------------------------------------------- //

// 		// 메시지를 퍼블리시 한다.
// 		haptic_pub.publish(msg);

// 		//loop_rate.sleep();
// 	}
// 	close(sock);
// 	return 0;
// }

// © 2022 GitHub, Inc.
// Terms
