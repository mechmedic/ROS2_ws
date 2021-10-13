
//This Code Modified by ARMANC KARAKOYUN
/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Ryu Woon Jung (Leon) */

//
// *********     Read and Write Example      *********
//
//
// Available DXL model on this example : All models using Protocol 1.0
// This example is tested with a DXL MX-28, and an USB2DYNAMIXEL
// Be sure that DXL MX properties are already set as %% ID : 1 / Baudnum : 34 (Baudrate : 57600)
//

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>
//  library addition starts ; for usleep() function.
#include <unistd.h>
// ends
//  ARMANC library addition starts 
#include <iostream>
#include <string>
// ends
#include "dynamixel_sdk.h"                                  // Uses Dynamixel SDK library
#include "joystick.h"
/// Control table address
#define ADDR_MX_TORQUE_ENABLE           24                  // Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION           30
#define ADDR_MX_PRESENT_POSITION        36

#define ADDR_MX_MOVING_SPEED            32
#define ADDR_MX_PRESENT_SPEED           38
#define ADDR_MX_GOAL_ACCELERATION       73
#define ADDR_MX_GOAL_PUNCH              48
#define ADDR_MX_STATUS_LED              25
#define ADDR_MX_CW_ANGLE_LIMIT          6
#define ADDR_MX_CCW_ANGLE_LIMIT         8


// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID                          1                  // Dynamixel ID: 1
#define DXL_ID1                         3
#define DXL_ID2                         2

#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
//Don't change this variables, range:0-4095, if you want to further you have to change CW AND CCW_ANGLE_LIMIT.
#define DXL_MINIMUM_POSITION_VALUE      1                 // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      4000                // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     10                  // Dynamixel moving status threshold


#define ESC_ASCII_VALUE                 0x1b
 
//---ARMANC partial modification starts---

XboxController Controller;


int getch()
{
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}

int kbhit(void)
{
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
#elif defined(_WIN32) || defined(_WIN64)
  return _kbhit();Present Voltage	
#endif
}

int main()
{
  if(Controller.initXboxController(XBOX_DEVICE) >= 0) {
		Controller.xbox = Controller.getXboxDataStruct();
		Controller.readXboxControllerInformation(Controller.xbox);

		printf("xbox controller detected\n\naxis:\t\t%d\nbuttons:\t%d\nidentifier:\t%s\n",
				Controller.xbox ->numOfAxis, Controller.xbox ->numOfButtons, Controller.xbox->identifier);
  }
  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  // Ensure that it was found and that we can use it
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  
  //---ARMANC partial modification starts---
    
  // This part is for configuration variable declaration.
  uint16_t dxl_present_speed     = 0;

  uint16_t dxl_moving_speed_cw   = 150;             // runs at (300*0.114)rpm 
  uint16_t dxl_moving_speed_ccw  = 1173;
  
  uint16_t dxl_moving_speed_cw_1   = 1023;          // runs at (300*0.114)rpm 
  uint16_t dxl_moving_speed_ccw_1  = 2047;
  
  uint8_t  dxl_error             = 0;               // Dynamixel error
  uint8_t  dxl_goal_punch        = 32;
  uint8_t  dxl_goal_acceleration = 254;
   //  partial modification ends...
  
  
  // Open port
  if (portHandler->openPort())
  {
    printf("Succeeded to open the port!\n");
  }
  else
  {
    printf("Failed to open the port!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }
  
  
  // Set port BAUDRATE AND BAUDRATE 1
  if (portHandler->setBaudRate(BAUDRATE))
  {
    printf("Succeeded to change the BAUDRATEs!\n");
  }
  else
  {
    printf("Failed to change the baudrate!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }
  
      printf("Press any key to enable torque!(or press ESC to quit!)\n");
    if (getch() == ESC_ASCII_VALUE)
    {
      
      return 0 ;
    }
    /*
    //ARMANC//---Motor ID=1 config starts

     dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
     printf("Torque enabled\n");
     dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_SPEED, dxl_present_speed, &dxl_error);
        
     dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_ACCELERATION, dxl_goal_acceleration, &dxl_error);  
     printf("Goal Acceleration enabled\n");
    
     dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_PUNCH, dxl_goal_punch, &dxl_error);  
     printf("Punch enabled\n");
    
     dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_CCW_ANGLE_LIMIT, 0, &dxl_error);  
     dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_CW_ANGLE_LIMIT, 0, &dxl_error); 

    //  ends*/

     //ARMANC//---Motor ID=1 config starts

     dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID2, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
     printf("Torque enabled\n");
     dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID2, ADDR_MX_PRESENT_SPEED, dxl_present_speed, &dxl_error);
        
     dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID2, ADDR_MX_GOAL_ACCELERATION, dxl_goal_acceleration, &dxl_error);  
     printf("Goal Acceleration enabled\n");
    
     dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID2, ADDR_MX_GOAL_PUNCH, dxl_goal_punch, &dxl_error);  
     printf("Punch enabled\n");
    
     dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID2, ADDR_MX_CCW_ANGLE_LIMIT, 0, &dxl_error);  
     dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID2, ADDR_MX_CW_ANGLE_LIMIT, 0, &dxl_error); 

    //  ends
    
    //ARMANC//---Motor ID=3 config starts

      dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID1, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
      printf("Torque enabled\n");
      dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID1, ADDR_MX_PRESENT_SPEED, dxl_present_speed, &dxl_error);
      
      dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID1, ADDR_MX_GOAL_ACCELERATION, dxl_goal_acceleration, &dxl_error);  
      printf("Goal Acceleration enabled\n");

      dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID1, ADDR_MX_GOAL_PUNCH, dxl_goal_punch, &dxl_error);  
      printf("Punch enabled\n");

      dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID1, ADDR_MX_CCW_ANGLE_LIMIT, 0, &dxl_error);  
      dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID1, ADDR_MX_CW_ANGLE_LIMIT, 0, &dxl_error);
    //  ends


  while(1)
  {       
    //ARMANC//---JOYSTICK INPUT READ

    Controller.readXboxData(Controller.xbox );
      //printf("stk_RightY = %d\n ,stk_RightTop =%d\n ,stk_LeftX = %d\n, stk_LeftY = %d\n" ,
      //Controller.xbox->stk_RightY,Controller.xbox->stk_RightTop,Controller.xbox->stk_LeftX,
      //Controller.xbox->stk_LeftY );
  
    
      //Motor for Linear Motion
      if ( Controller.xbox->btn_Y  > 0){
        //Clockwise 
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID2, ADDR_MX_MOVING_SPEED, dxl_moving_speed_cw_1, &dxl_error);
        dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID2, ADDR_MX_MOVING_SPEED, &dxl_present_speed, &dxl_error);
        
        printf("Backward motion is on = %f",float (dxl_present_speed*0.114));
        printf(" rpm\n") ;     
      }
         
      else if (Controller.xbox->btn_X > 0)
      {
        //Counter-clockwise
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID2, ADDR_MX_MOVING_SPEED, dxl_moving_speed_ccw_1, &dxl_error);
        dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID2, ADDR_MX_MOVING_SPEED, &dxl_present_speed, &dxl_error);
        printf("Forward motion is on= %f",float ((dxl_present_speed-1023)*0.114));
        printf(" rpm\n") ;    
      }
      else{
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID2, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
       
      }  
      // MOTOR for Rotation Motion
      if(Controller.xbox->stk_RightY > 0){
          //Clockwise
          dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID1, ADDR_MX_MOVING_SPEED, dxl_moving_speed_cw, &dxl_error);
          dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID1, ADDR_MX_MOVING_SPEED, &dxl_present_speed, &dxl_error);
          printf("Clockwise rotation is on= %f",float (dxl_present_speed*0.114));
          printf(" rpm\n") ;
           
          }
               
         else if(Controller.xbox->stk_RightY< 0)
        {
          //Counter-Clockwise
          dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID1, ADDR_MX_MOVING_SPEED, dxl_moving_speed_ccw, &dxl_error);
          dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID1, ADDR_MX_MOVING_SPEED, &dxl_present_speed, &dxl_error);
          printf("Counter-Clockwise rotation is on= %f",float (dxl_present_speed-1023)*0.114);
          printf(" rpm\n") ;
          }
       else
       {
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID1, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
      }  
 
  } 
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }
  else
  {
    printf("Dynamixel has been successfully connected \n");
  }

  // Disable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }

  // Close port
  portHandler->closePort();

  return 0;
  
 
}
