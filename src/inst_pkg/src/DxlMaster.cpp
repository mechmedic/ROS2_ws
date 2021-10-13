
#include "DxlMaster.h"
#include <sys/time.h>
#include <stdio.h>
#define INTERVAL 20*MS

//pthread_mutex_t DxlMaster::mutex1 = PTHREAD_MUTEX_INITIALIZER;

DxlMaster::DxlMaster()
{
    for(int i=0; i<3; i++)
    {
        m_devIds[i] = i+1;  // 3 Devices 1,2,3
    }
    m_baudrate = BAUDRATE;

    m_groupSyncWrite = NULL;
    m_groupBulkRead = NULL;

    m_cntPerRevolution = 4095;
    m_resDivider = 1;
    m_tr[0] = -7.0/6.0;
    m_tr[1] = 7.0/6.0;
}

DxlMaster::~DxlMaster()
{

}

int DxlMaster::InitializeDriver()
{
    // CKim - Set port path, protocol version
    m_portHandler = dynamixel::PortHandler::getPortHandler(m_portName);
    m_packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    // CKim - Open port
    if (m_portHandler->openPort())    {
      printf("Succeeded to open the port!\n");
    }
    else    {
      printf("Failed to open the port!\n");
      return 0;
    }

    // CKim - Set port baudrate
    if (m_portHandler->setBaudRate(BAUDRATE))    {
      printf("Succeeded to change the baudrate!\n");
    }
    else    {
      printf("Failed to change the baudrate!\n");
      return 0;
    }

    // CKim - Initialize GroupSyncWrite instance. Set up parameters to write at the same time
    m_groupSyncWrite = new dynamixel::GroupSyncWrite(m_portHandler, m_packetHandler, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION);

    // CKim - Initialize GroupBulkRead instance. Set up parameters to bulk read
    m_groupBulkRead = new dynamixel::GroupBulkRead(m_portHandler, m_packetHandler);
    for(int i=0; i<3; i++)
    {
        // CKim - We will read present position of each actuators
        bool dxl_addparam_result = m_groupBulkRead->addParam(m_devIds[i], ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
        if (dxl_addparam_result != true)
        {
            fprintf(stderr, "[ID:%03d] grouBulkRead addparam failed", m_devIds[i]);
            return 0;
        }
    }

    return 1;
}

void DxlMaster::EnableTorque()
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error

    for(int i=0; i<3; i++)
    {
        dxl_comm_result = m_packetHandler->write1ByteTxRx(m_portHandler, m_devIds[i], ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
          printf("%s\n", m_packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
          printf("%s\n", m_packetHandler->getRxPacketError(dxl_error));
        }
        else
        {
          printf("Dynamixel#%d has been successfully connected \n", m_devIds[i]);
        }
    }
}

void DxlMaster::DisableTorque()
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error

    for(int i=0; i<3; i++)
    {
        dxl_comm_result = m_packetHandler->write1ByteTxRx(m_portHandler, m_devIds[i], ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
          printf("%s\n", m_packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
          printf("%s\n", m_packetHandler->getRxPacketError(dxl_error));
        }
        else
        {
          printf("Dynamixel#%d has been successfully disconnected \n", m_devIds[i]);
        }
    }
}

void DxlMaster::Disconnect()
{
    if(m_groupSyncWrite)    delete m_groupSyncWrite;
    if(m_groupBulkRead)    delete m_groupBulkRead;

    // Close port
    m_portHandler->closePort();
    //ROS_INFO("%s","Closing SerialPort");
}

bool DxlMaster::GetJpos(int *pPos)
{
	int dxl_comm_result = COMM_TX_FAIL;             // Communication result
	bool dxl_getdata_result = false;                 // GetParam result

	// CKim - Read current position. Bulkread is synchronized read
	dxl_comm_result = m_groupBulkRead->txRxPacket();
	if (dxl_comm_result != COMM_SUCCESS)
	{
		fprintf(stderr,"Sparta!! ");
		fprintf(stderr, "%s\n", m_packetHandler->getTxRxResult(dxl_comm_result));
		return 0;
	}

	for(int i=0; i<3; i++)
	{
		dxl_getdata_result = m_groupBulkRead->isAvailable(m_devIds[i], ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
		if (dxl_getdata_result != true)
		{
		  fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed", m_devIds[i]);
		  return 0;
		}

		// Get Dynamixel present position value
		pPos[i] = m_groupBulkRead->getData(m_devIds[i], ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
		//printf("%d  ",dxl_curr_position[i]);
	}
	//printf("\n ");

	return 1;

}

bool DxlMaster::SetJpos(int *pPos)
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    bool dxl_addparam_result = false;               // addParam result
    uint8_t param_goal_position[2];

    // CKim - Fill in data to write
    for(int i=0; i<3; i++)
    {
        // Allocate goal position value into byte array
        param_goal_position[0] = DXL_LOBYTE(pPos[i]);
        param_goal_position[1] = DXL_HIBYTE(pPos[i]);

        // Add Dynamixel#1 goal position value to the Syncwrite storage
        dxl_addparam_result = m_groupSyncWrite->addParam(m_devIds[i], param_goal_position);
        if (dxl_addparam_result != true)
        {
            fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", m_devIds[i]);
            return 0;
        }
    }

    // CKim - Synchronized write to multiple dynamixel
	dxl_comm_result = m_groupSyncWrite->txPacket();
    if (dxl_comm_result != COMM_SUCCESS)
    {
        fprintf(stderr, "%s\n", m_packetHandler->getTxRxResult(dxl_comm_result));
        return 0;
    }

    // Clear syncwrite parameter storage
    m_groupSyncWrite->clearParam();

    return 1;
}

bool DxlMaster::SetMultiTurn(bool onoff)
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error
    uint16_t flag;

    if(onoff)   {   flag = 4095;    }
    else        {   flag = 0;       }

    for(int i=0; i<3; i++)
    {
        dxl_comm_result = m_packetHandler->write2ByteTxRx(m_portHandler, m_devIds[i], ADDR_CW_ANGLE_LIM, flag, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", m_packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", m_packetHandler->getRxPacketError(dxl_error));
        }
        else
        {
            printf("Dynamixel#%d has been set to ", m_devIds[i]);
            if(onoff)   {   printf("MultiTurn mode\n"); }
            else        {   printf("Joint mode\n"); }
        }
    }

    return 1;
}

void DxlMaster::SetGain(int id, const float* DIP)
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error
    uint8_t dipGain[3];

    // CKim - Conversion formula from Dynamixel manual
    dipGain[0] = DIP[0]*250.0;
    dipGain[1] = DIP[1]*2.048;
    dipGain[2] = DIP[2]*8.0;

    for(int i=0;i<3; i++)
    {
        dxl_comm_result = m_packetHandler->write1ByteTxRx(m_portHandler, m_devIds[id], ADDR_DIP_GAIN+i, dipGain[i],&dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", m_packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", m_packetHandler->getRxPacketError(dxl_error));
        }
        else
        {
            printf("DIP[%d] of Dynamixel %d set\n",i,id);
        }
    }
}

int DxlMaster::SetOffset()
{
    int dxl_curr_position[3] = {0,0,0};  // Current position

    // CKim - Read position
    bool res = GetJpos(dxl_curr_position);
    if(!res)    {
        fprintf(stderr, "Error while reading position");
        return 0;
    }

    for(int i=0; i<3; i++)  {   m_MtrCntOffset[i] = dxl_curr_position[i];   }
    printf("Offset is %d %d %d\n", m_MtrCntOffset[0],m_MtrCntOffset[1],m_MtrCntOffset[2]);
    //for(int i=0; i<3; i++)  {   m_TgtMtrCnt[i] = m_MtrCntOffset[i];   }
}

// void* DxlMaster::ThrCallback(void* pData)
// {
// 	DxlMaster* pDriver = (DxlMaster*)pData;

//     bool res;

//     int dxl_goal_position[3] = {0,0,0};  // Goal position
//     int dxl_curr_position[3] = {0,0,0};  // Current position


//     //    // CKim - Data variables
//     //    int tgtPos[3];

// 	// CKim - RT variables
// 	long long int time, time_r;
// 	struct timespec t, t_real;
// 	struct sched_param param;
// 	ssize_t ret = 0;

// 	int errCnt = 0;

// 	// CKim - Increase priority (this thread will preempt the kernel functions)
// 	param.sched_priority = MY_PRIORITY;
// 	if(sched_setscheduler(0, SCHED_FIFO, &param) != 0)
// 	{
// 		perror("sched_setscheduler failed");
// 		return 0;//exit(-1);
// 	}

// 	// CKim - Set clock
// 	clock_gettime(CLOCK_REALTIME ,&t); /* start after one second */
// 	t.tv_nsec += 100*MS;
// 	tsnorm(&t);


//     while(ros::ok())
//     {
// 		// CKim - Update dxl_goal_position and save current position
// 		pthread_mutex_lock( &mutex1 );
// 		for(int i=0; i<3; i++)  {
// 			dxl_goal_position[i] = pDriver->m_TgtMtrCnt[i];
// 			//pDriver->m_CurrMtrCnt[i] = pDriver->m_TgtMtrCnt[i];
// 			pDriver->m_CurrMtrCnt[i] = dxl_curr_position[i];
// 		}
// 		pthread_mutex_unlock( &mutex1 );

// 		// CKim - Command position
// 		res = pDriver->SetJpos(dxl_goal_position);
// 		if(!res)    {
// 			fprintf(stderr, "Error while commanding position %d\n", errCnt);
// 			//return 0;
// 			errCnt++;
// 		}

// 		// CKim - Read position
// 		res = pDriver->GetJpos(dxl_curr_position);
// 		if(!res)    {
// 			fprintf(stderr, "Error while reading position %d\n", errCnt);
// 			//return 0;
// 			errCnt++;
// 		}


// 		// CKim - Check if loop ran within time
// 		t.tv_nsec += INTERVAL;
// 		tsnorm(&t);

// 		clock_gettime(CLOCK_REALTIME ,&t_real);
// 		time_r = (long long int)t_real.tv_nsec + (long long int)t_real.tv_sec*NSEC_PER_SEC;
// 		time  = (long long int)t.tv_nsec + (long long int)t.tv_sec*NSEC_PER_SEC;

// 		if(time < time_r){
// 			//_this->device_state = 0;
// 			//ROS_INFO("Failed to meet time");
// 			//return 0;
// 		}

// 		// CKim - Sleep for remaining time of the loop
// 		clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &t, NULL);


// 		//usleep(10000);

//       }

//     // CKim - Return Motor to original location at the end
//     for(int i=0; i<3; i++)  {   dxl_goal_position[i] = pDriver->m_MtrCntOffset[i];   }
//     //for(int i=0; i<3; i++)  {   dxl_goal_position[i] = -100;   }
//     pDriver->SetJpos(dxl_goal_position);
//     sleep(3);

//     return 0;
// }

// void DxlMaster::HapticBack(const TeleOp::TeleOpCmd msg)
// {
// 	using namespace ChunKinematics;

// 	static int init = 0;
// 	// ------------------------------------------------
// 	// CKim - First check pedal down. Runlevel/Sublevel is 3/3 when pedal is down and 2/3 when pedal is up
// 	bool pedalDown = msg.pedal;
// 	if(!pedalDown)  {  init = 0;		 return ;    }
// 	// ------------------------------------------------


// 	int currCnt[3];			int tgtCnt[3];			double currAng[3];

// 	// ---------------------------------------------------------------------------------------
// 	// CKim - First time pedal pressed. Calculate the reference
// 	if(pedalDown && init < 100)
// 	{
// 		pthread_mutex_lock( &mutex1 );
// 		for(int i=0; i<3; i++)  {
// 			currCnt[i] = m_CurrMtrCnt[i];
// 		}
// 		pthread_mutex_unlock( &mutex1 );

// 		for(int i=0; i<3; i++) {
// 			currAng[i] = (currCnt[i] - m_MtrCntOffset[i])*360.0/m_cntPerRevolution*m_resDivider*m_tr[i];
// 		}
// 		SolveFwdKin(currAng,m_InitSlave);

// 		//m_InitSlave = Mat3::Identity();
// 		//m_MasterTo
// 		m_prevSlave = m_InitSlave;

// 		init++;
// 		return;
// 	}
// 	// ---------------------------------------------------------------------------------------

// 	// ---------------------------------------------------------------------------------------
// 	double cmdDir[3];		double tgtDir[3];		double theta;				double phi;						double jAng[3] = {0,0,0};
// 	Mat3 deltaMaster;		Mat3 deltaSlave;		Mat3 tgtOrt;				Eigen::Vector3d w;			Mat3 currSlave;
// 	double scl = 0.8;	//1.2;
// 	if(m_Hand == LEFT_HAND)
// 	{
// 		// --------------------------------------------------------
// 		// CKim - Handle grasper button press
// 		if(msg.btnL[0] && !msg.btnL[1])   {   m_grasp -= 1;   }
// 		if(!msg.btnL[0] && msg.btnL[1])   {   m_grasp += 1;   }

// 		// CKim -Limt Grasping Angle
// 		if(m_grasp < -900)   {   m_grasp = -900;  }
// 		if(m_grasp > 0)      {   m_grasp = 0;     }
// 		// --------------------------------------------------------

// 		// --------------------------------------------------------
// 		// CKim - msg.ortL/R has orientation difference in axis and angle. Convert this to rotation matrix
// 		for(int i=0; i<3; i++)	{	w[i] = msg.ortL[i]*scl;	}
// 		//for(int i=0; i<3; i++)	{	w[i] = msg.ortL[i]*scl;	}		w[2] = 0;		// delta in global csys
// 		deltaMaster = Rodrigues3(w);
// 		deltaSlave = m_MasterToSlave * deltaMaster * m_MasterToSlave.transpose();
// //		deltaSlave = m_MasterToSlave * m_InitSlave.transpose() * deltaMaster * m_InitSlave * m_MasterToSlave.transpose();
// //		deltaSlave = m_InitSlave.transpose() * deltaMaster * m_InitSlave;

// 		// CKim - Multiply the increment of the orientation and calculate the target dir.  Calculate target dir
// 		tgtOrt = m_prevSlave*deltaSlave;
// 		//tgtOrt = deltaSlave*m_prevSlave; // delta in global csys
// 		for(int i=0; i<3; i++)	{	cmdDir[i] = tgtOrt(i,2)/(tgtOrt.block<3,1>(0,2)).norm();	}

// 		// CKim - Avoid singularity (polar angle phi close to zero) and limit(phi exceeding 89 degree) of the target direction
// 		theta = atan2(cmdDir[1],cmdDir[0]);		phi = acos(cmdDir[2]);
// //		if( (phi < (0.5*DEG2RAD)) || (phi > (89.5*DEG2RAD)) )
// //		{
// //			if(phi < (0.5*DEG2RAD))		{	phi = 0.5*DEG2RAD;		}
// //			if(phi > (89*DEG2RAD))		{	phi = 89.5*DEG2RAD;	}
// //		}
// 		if( (phi > (89.5*DEG2RAD)) )
// 		{
// 			phi = 89.5*DEG2RAD;
// 		}
// 		else
// 		{
// 			m_prevSlave = tgtOrt;
// 		}

// 		tgtDir[0] = sin(phi)*cos(theta);
// 		tgtDir[1] = sin(phi)*sin(theta);
// 		tgtDir[2] = cos(phi);
// 		//m_prevSlave = tgtOrt;
// 		// ---------------------------------------------------


// //		if(init%1000 == 0)
// //		{
// //			ROS_INFO("%.5f, %.5f, %.5f\n",tgtDir[0],tgtDir[1],tgtDir[2]);
// ////			ROS_INFO("%.5f, %.5f, %.5f\n",deltaSlave(0,0),deltaSlave(0,1),deltaSlave(0,2));
// ////			ROS_INFO("%.5f, %.5f, %.5f\n",deltaSlave(1,0),deltaSlave(1,1),deltaSlave(1,2));
// ////			ROS_INFO("%.5f, %.5f, %.5f\n",deltaSlave(2,0),deltaSlave(2,1),deltaSlave(2,2));

// ////			ROS_INFO("%.5f, %.5f, %.5f\n",m_InitSlave(0,0),m_InitSlave(0,1),m_InitSlave(0,2));
// ////			ROS_INFO("%.5f, %.5f, %.5f\n",m_InitSlave(1,0),m_InitSlave(1,1),m_InitSlave(1,2));
// ////			ROS_INFO("%.5f, %.5f, %.5f\n",m_InitSlave(2,0),m_InitSlave(2,1),m_InitSlave(2,2));

// ////			ROS_INFO("%.5f, %.5f, %.5f\n",R(0,0),R(0,1),R(0,2));
// ////			ROS_INFO("%.5f, %.5f, %.5f\n",R(1,0),R(1,1),R(1,2));
// ////			ROS_INFO("%.5f, %.5f, %.5f\n",R(2,0),R(2,1),R(2,2));
// //				ROS_INFO("\n");
// //		}
// //		init++;
// 	}
// 	// ---------------------------------------------------------------------------------------

// 	// ------------------------------------------------
// 	// CKim - Inverse Kin
// 	SolveInvKin(tgtDir,jAng);
// 	jAng[2] = m_grasp;

// 	// CKim - Convert MotorAngle to count - depends on wrist type
// 	tgtCnt[0] = m_MtrCntOffset[0] + jAng[0]/360.0*m_cntPerRevolution/m_resDivider/m_tr[0];
// 	tgtCnt[1] = m_MtrCntOffset[1] + jAng[1]/360.0*m_cntPerRevolution/m_resDivider/m_tr[1];
// 	tgtCnt[2] = m_MtrCntOffset[2] - jAng[2];///360.0*m_cntPerRevolution/m_resDivider/m_tr[1];

// 	for(int i=0; i<3; i++)  {
// 		if(abs(tgtCnt[i]) > 28672)
// 			ROS_INFO("Angle Limit %d at %d",i,tgtCnt[i]);
// 	}

// 	// CKim - Exchange data wit communication thread
// 	pthread_mutex_lock( &mutex1 );
// 	for(int i=0; i<3; i++)
// 	{
// 		m_TgtMtrCnt[i] = tgtCnt[i];
// 		currCnt[i] = m_CurrMtrCnt[i];
// 	}
// 	pthread_mutex_unlock( &mutex1 );
// 	// ------------------------------------------------

// 	// CKim FwdKin
// 	for(int i=0; i<3; i++) {
// 		currAng[i] = (currCnt[i] - m_MtrCntOffset[i])*360.0/m_cntPerRevolution*m_resDivider*m_tr[i];
// 	}
// 	SolveFwdKin(currAng,currSlave);
// //	SolveFwdKin(currAng,m_prevSlave);


// 	// CKim - Publish ROS msg
// 	for(int i=0; i<3; i++)
// 	{
// 		msgStatus.cmdDir[i] = currSlave(i,2);	//currAng[i];	//vSlave(i,2);
// 		msgStatus.mtrAng[i] = jAng[i];
// 		msgStatus.tgtDir[i] = tgtDir[i];
// 	}
// 	pubWristStatus.publish(msgStatus);
// }


//// CKim - Original HapticBack
//void DxlMaster::HapticBack(const TeleOp::TeleOpCmd msg)
//{

//	// ------------------------------------------------
//	// CKim - First check pedal down. Runlevel/Sublevel is 3/3 when pedal is down and 2/3 when pedal is up
//	bool pedalDown = msg.pedal;
//	if(!pedalDown)  {   return ;    }
//	// ------------------------------------------------

//	// CKim - Save cuirrent commanded orientation
//	for(int i=0; i<2; i++)
//	{
//		m_prevRyRx[i] = m_RyRx[i];
//	}

//	// CKim - Button is pressed. Gimbal angle differences inradian.
//	if(m_Hand == LEFT_HAND)
//	{
//		m_RyRx[0] -= (0.9*msg.ortL[0]);    //(0.6*cmd.array[3]);
//		m_RyRx[1] += (0.9*msg.ortL[1]);    //(0.6*cmd.array[4]);
//		if(msg.btnL[0] && !msg.btnL[1])   {   m_grasp -= 1;   }
//		if(!msg.btnL[0] && msg.btnL[1])   {   m_grasp += 1;   }
//	}
//	if(m_Hand == RIGHT_HAND)
//	{
//		m_RyRx[0] -= (0.9*msg.ortR[0]);    //(0.6*cmd.array[3]);
//		m_RyRx[1] += (0.9*msg.ortR[1]);    //(0.6*cmd.array[4]);
//		if(msg.btnR[0] && !msg.btnR[1])   {   m_grasp -= 2;   }
//		if(!msg.btnR[1] && msg.btnR[1])   {   m_grasp += 2;   }
//	}

//	// ---------------------------------------------------
//	// CKim - Limit Ry Rx below 90 degree
//	for(int i=0; i<2; i++)
//	{
//		if(fabs(m_RyRx[i]) > 89.5*DEG2RAD)
//		{
//			if(m_RyRx[i]>=0)    {   m_RyRx[i] = 89.5*DEG2RAD;   }
//			if(m_RyRx[i]<0)     {   m_RyRx[i] = -89.5*DEG2RAD;  }
//		}
//	}

//	// CKim -Limt Grasping Angle
//	if(m_grasp < -900)   {   m_grasp = -900;  }
//	if(m_grasp > 0)      {   m_grasp = 0;     }
//	// ---------------------------------------------------

//	// ---------------------------------------------------
//	// CKim - Calculate target wrist orientation and grasp
//	// Avoid sigularity near 0
//	double tgtDir[3];   double Ry, Rx;

//	if(fabs(m_RyRx[0]) < 1.5*DEG2RAD)
//	{
//		if(m_RyRx[0]>=0)    {   Ry = 1.5*DEG2RAD;   }
//		if(m_RyRx[0]<0)     {   Ry = -1.5*DEG2RAD;  }
//	}
//	else
//	{
//		Ry = m_RyRx[0];
//	}

//	if(fabs(m_RyRx[1]) < 1.5*DEG2RAD)
//	{
//		if(m_RyRx[1]>=0)    {   Rx = 1.5*DEG2RAD;   }
//		if(m_RyRx[1]<0)     {   Rx = -1.5*DEG2RAD;  }
//	}
//	else
//	{
//		Rx = m_RyRx[1];
//	}
//	// ---------------------------------------------------

//	// CKim - 2018 03/12 - Take into account the transformation btw omni and wrist csys.
//	tgtDir[0] = sin(Rx);
//	tgtDir[1] = -sin(Ry)*cos(Rx);
//	tgtDir[2] = cos(Ry)*cos(Rx);


//	// ------------------------------
//	// CKim - Inverse Kin
//	double jAng[3] = {0,0,0};
//	SolveInvKin(tgtDir,jAng);
//	jAng[2] = m_grasp;
//	// ------------------------------------------------

//	// CKim - Convert MotorAngle to count - depends on wrist type
//	pthread_mutex_lock( &mutex1 );
//	m_TgtMtrCnt[0] = m_MtrCntOffset[0] + jAng[0]/360.0*m_cntPerRevolution/m_resDivider/m_tr[0];
//	m_TgtMtrCnt[1] = m_MtrCntOffset[1] + jAng[1]/360.0*m_cntPerRevolution/m_resDivider/m_tr[1];
//	//m_TgtMtrCnt[2] = m_MtrCntOffset[2] + jAng[2];///360.0*m_cntPerRevolution/m_resDivider/m_tr[1];
//	m_TgtMtrCnt[2] = m_MtrCntOffset[2] - jAng[2];///360.0*m_cntPerRevolution/m_resDivider/m_tr[1];

//	for(int i=0; i<3; i++)  {
//		if(abs(m_TgtMtrCnt[i]) > 28672)
//			ROS_INFO("Angle Limit %d at %d",i,m_TgtMtrCnt[i]);
//	}
//	pthread_mutex_unlock( &mutex1 );
//}

