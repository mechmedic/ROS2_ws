// --------------------------------------------------------------------- //
// CKim - Oct. 14, 2021 : Class encapsulating Dynamixel
// --------------------------------------------------------------------- //

// // CKim - ROS include
// #include <ros/ros.h>

// // CKim - Headers for the published / subscribed message.
// #include <TeleOp/TeleOpCmd.h>
// #include <ObjWrist/WristStatus.h>

// CKim - These header files and definitions are for serial communications
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>	// 'file control'
#include <termios.h>
#include <stdio.h>
#include <signal.h>		// CKim - For catching 'ctrl-c' input
#include <sys/ioctl.h>
#include <linux/serial.h>

////////////////////////////start rt prempt include///////////////////////////////
#include <time.h>
#include <sched.h>
#include <sys/mman.h>
#include <string.h>
#include <unistd.h>
#include <malloc.h>
#include <pthread.h>
////////////////////////////end rt prempt include///////////////////////////////

#define _POSIX_SOURCE 1 /* POSIX compliant source */

#define MAXLOOF 50

///////////////////////////start rt prempt define///////////////////////////////
#define MY_PRIORITY (49) /* we use 49 as the PRREMPT_RT use 50 as the priority of kernel tasklets and interrupt handler by default */
#define POOLSIZE (1*1024*1024) /* The maximum stack size which is guaranteed safe to access without faulting */
#define NSEC_PER_SEC (1000000000) /* The number of nsecs per sec. */
#define N_EVER 100

#define NS  	(1)
#define US  	(1000 * NS)
#define MS  	(1000 * US)
#define SEC 	(1000 * MS)
////////////////////////////end rt prempt define///////////////////////////////


// ---------------- Dynamixel Defines --------------------------------- //
#include "dynamixel_sdk/dynamixel_sdk.h"      // CKim - Uses Dynamixel SDK library that is installed with the ROS

// Control table address
#define ADDR_MX_TORQUE_ENABLE           24                  // Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION           30
#define ADDR_MX_PRESENT_POSITION        36
#define ADDR_CW_ANGLE_LIM               6                   // Set this to 4095 to enable multiturn
#define ADDR_ANGLE_DIVIDER              22
#define ADDR_DIP_GAIN                   26

// Data Byte Length
#define LEN_MX_GOAL_POSITION            2
#define LEN_MX_PRESENT_POSITION         2
#define LEN_CW_ANGLE_LIM                2
#define LEN_ANGLE_DIVIDER               1
#define LEN_DIP_GAIN                    3                   // 1 byte per each D,I,P Gain

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL1_ID                         1                   // Dynamixel#1 ID: 1
#define DXL2_ID                         2                   // Dynamixel#2 ID: 2
#define DXL3_ID                         3                   // Dynamixel#3 ID: 3
#define BAUDRATE                        1000000

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define MULTITURN_ENABLE                4095

// CKim - tty means TeleTYpewriter, which used to be the input output 'terminals'
// of the computers system in the past. Now, it refers to the general 'terminals' of the device.
// Change this definition for the correct port. We will be using terminal of the COM port.
//#define MODEMDEVICE "/dev/ttyUSB0"		// This is for FTDI USB RS485 cable

#define LEFT_HAND  1
#define RIGHT_HAND 2

#define DEG2RAD 3.141592/180.0
#define RAD2DEG 180.0/3.141592

//#include "ChunKinematics.h"
//#include "WristKin.h"

class DxlMaster
{

public:
	DxlMaster();
	~DxlMaster();

    // CKim - Initialize Dynamixel .... RS485 port open and close...
    int InitializeDriver();
    void EnableTorque();
    void DisableTorque();
    void Disconnect();

    // CKim - Initialize ROS publishing and subscription
    //int InitializeROS();
////    void PublishState(unsigned char pState[]);

    // CKim - SetHands
   // void SetLeftHand();
   // void SetRightHand();

    // CKim - Motor Commands
    bool GetJpos(int* pPos);    // in counts
    bool SetJpos(int* pPos);
    bool SetMultiTurn(bool onoff);
    void SetGain(int id, const float* DIP);
    int SetOffset();

	//bool SolveInvKin(const double tgtDir[3], double mtrAng[3]);		// Angle in Degree
	//void SolveFwdKin(const double mtrAng[3], Mat3& R);				// Angle in Degree

    //    // CKim - Send Commands
//    void SetVel(int id, const int& vel);


    // CKim - This function is executed in separate thread.
    // Continuously reads and writes to CatheterDriver by RS485
    //static void* ThrCallback(void* pData);
   // static pthread_mutex_t mutex1;
    //int16_t m_TgtMtrCnt[3];
	//int16_t m_CurrMtrCnt[3];

private:

    // CKim - Callback for subscribed ROS message
	//void HapticBack(const TeleOp::TeleOpCmd msg);

    // CKim - ROS variables
    //ros::NodeHandle nh;
    //ros::Subscriber haptic_sub;
	//ros::Publisher pubWristStatus;
	//ObjWrist::WristStatus msgStatus;

    // CKim - Dynamixel Variables. Class for handling serial port, handling packets
    dynamixel::PortHandler*     m_portHandler;
    dynamixel::PacketHandler*   m_packetHandler;
    dynamixel::GroupSyncWrite*  m_groupSyncWrite;
    dynamixel::GroupBulkRead*   m_groupBulkRead;

    // CKim - Dynamixel Device IDs, baud rates
    int m_devIds[3];
    int m_baudrate;
    int m_cntPerRevolution;
    int m_resDivider;
    float m_tr[2];               // CKim - Gear transmission ratio. 7/6 1 Dynamixel turn rotates axis 7/6 turn

    // int m_Hand;     // 1 Left, 2 Right
    // double m_RyRx[2];
    // double m_prevSol[4];
    // double m_currSol[4];
    // double m_prevRyRx[2];
    // double m_grasp;
    int16_t m_MtrCntOffset[3];
		
    char m_portName[40];

	// Mat3 m_prevSlave;
	// Mat3 m_InitSlave;
	// Mat3 m_MasterToSlave;
	//int m_pedalFirst;
};

static inline void tsnorm(struct timespec *ts)
{
    while (ts->tv_nsec >= NSEC_PER_SEC)
    {
        ts->tv_nsec -= NSEC_PER_SEC;
        ts->tv_sec++;
    }
}
