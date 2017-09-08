/*******************************************************************************
* Copyright (c) 2016, ROBOTIS CO., LTD.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of ROBOTIS nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/* Author: Ryu Woon Jung (Leon) */

//
// *********     Read and Write Example      *********
//
//
// Available Dynamixel model on this example : All models using Protocol 2.0
// This example is tested with a Dynamixel PRO 54-200, and an USB2DYNAMIXEL
// Be sure that Dynamixel PRO properties are already set as %% ID : 1 / Baudnum : 3 (Baudrate : 1000000)
//

#ifdef __linux__
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include "cForceSensor.h"


#include <fstream>
#include <iostream>
#include <string>

#include "dynamixel_sdk.h"                                  // Uses Dynamixel SDK library

// Control table address
#define ADDR_PRO_TORQUE_ENABLE          64                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          116
#define ADDR_PRO_PRESENT_POSITION       132
#define ADDR_PRO_GOAL_CURRENT           102
#define ADDR_PRO_PRESENT_CURRENT        126
#define ADDR_PRO_PRESENT_VELOCITY       128
#define HOME_OFFSET 20
#define DRIVE_MODE 10 

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID                          1                   // Dynamixel ID: 1
#define BAUDRATE                        3000000
#define DEVICENAME                      "COM3"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      1            // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      4000              // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MINIMUM_CURRENT_VALUE       20           // Dynamixel will rotate between this value
#define DXL_MAXIMUM_CURRENT_VALUE       0            // and

#define DXL_MOVING_STATUS_THRESHOLD      10                // Dynamixel moving status threshold
#define DXL_CURRENT_STATUS_THRESHOLD     1                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b

int getch()
{
#ifdef __linux__
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
#ifdef __linux__
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
  return _kbhit();
#endif
}



double ConeMax(double alpha, double phi, double F_actual, double limit, double R, double offset)
{
	double MomentSet[6] = { NAN,NAN,NAN,NAN,NAN,NAN };
	double T1, T2, N1, N2, Moment, M1;
	int j = 0;
	// case 1: T2 and two compressions
	if (1)
	{
		T1 = 0;
		T2 = limit;
		N1 = ((T2 * sin(alpha) - F_actual * sin(phi)) / cos(alpha) + (T2 * cos(alpha) - F_actual * cos(phi)) / sin(alpha)) / 2;
		N2 = ((T2 * sin(alpha) - F_actual * sin(phi)) / cos(alpha) - (T2 * cos(alpha) - F_actual * cos(phi)) / sin(alpha)) / 2;
		if (N1 >= -0.02 && N2 >= -0.02)
		{
			Moment = (N2 - N1) * R * tan(alpha);
			MomentSet[j] = Moment;
			j++;
		}
	}



	//case 2: T1 and two compressions
	if (1)
	{
		T1 = limit;
		T2 = 0;
		N1 = ((T1 * sin(alpha) - F_actual * sin(phi)) / cos(alpha) + (-T1 * cos(alpha) - F_actual * cos(phi)) / sin(alpha)) / 2;
		N2 = ((T1 * sin(alpha) - F_actual * sin(phi)) / cos(alpha) - (-T1 * cos(alpha) - F_actual * cos(phi)) / sin(alpha)) / 2;
		if (N1 >= -0.02 && N2 >= -0.02)
		{
			Moment = (N2 - N1) * R * tan(alpha);
			MomentSet[j] = Moment;
			j++;
		}
	}



	// case 3: two tensions and N2, T1 is limit
	if (1)
	{
		T1 = limit;
		N1 = 0;
		N2 = F_actual * cos(phi) * sin(alpha) + T1 * cos(alpha) * sin(alpha) - F_actual * sin(phi) * cos(alpha) + T1 * sin(alpha) * cos(alpha);
		T2 = F_actual * cos(phi) * cos(alpha) + T1 * cos(alpha) * cos(alpha) + F_actual * sin(phi) * sin(alpha) - T1 * sin(alpha) * sin(alpha);
		if (N2 >= -0.02 && T2 >= -0.02 && T2 <= limit + 0.02)
		{
			Moment = (N2 - N1) * R * tan(alpha);
			MomentSet[j] = Moment;
			j++;
		}
	}


	// case 4: two tensions and N2, T2 is limit
	if (1)
	{
		T2 = limit;
		N1 = 0;
		N2 = (F_actual * cos(phi) * sin(alpha) - T2 * cos(alpha) * sin(alpha) + F_actual * sin(phi) * cos(alpha) - T2 * sin(alpha) * cos(alpha)) / (-cos(2 * alpha));
		T1 = (F_actual * cos(phi) * cos(alpha) - T2 * cos(alpha) * cos(alpha) + F_actual * sin(phi) * sin(alpha) - T2 * sin(alpha) * sin(alpha)) / (-cos(2 * alpha));
		if (N2 >= -0.02 && T1 >= -0.02 && T1 <= limit + 0.02)
		{
			Moment = (N2 - N1) * R * tan(alpha);
			MomentSet[j] = Moment;
			j++;
		}
	}

	// case 5: two tensions and N1, T1 is limit
	if (1)
	{
		T1 = limit;
		N2 = 0;
		N1 = (F_actual * cos(phi) * sin(alpha) + T1 * cos(alpha) * sin(alpha) - F_actual * sin(phi) * cos(alpha) + T1 * sin(alpha) * cos(alpha)) / cos(2 * alpha);
		T2 = (F_actual * cos(phi) * cos(alpha) + T1 * cos(alpha) * cos(alpha) - F_actual * sin(phi) * sin(alpha) + T1 * sin(alpha) * sin(alpha)) / cos(2 * alpha);
		if (N1 >= -0.02 && T2 >= -0.02 && T2 <= limit + 0.02)
		{
			Moment = (N2 - N1) * R * tan(alpha);
			MomentSet[j] = Moment;
			j++;
		}
	}


	// case 6: two tensions and N1, T2 is limit
	if (1)
	{
		T2 = limit;
		N2 = 0;
		N1 = (F_actual * cos(phi) * sin(alpha) - T2 * cos(alpha) * sin(alpha) + F_actual * sin(phi) * cos(alpha) - T2 * sin(alpha) * cos(alpha)) / (-1);
		T1 = (F_actual * cos(phi) * cos(alpha) - T2 * cos(alpha) * cos(alpha) - F_actual * sin(phi) * sin(alpha) + T2 * sin(alpha) * sin(alpha)) / (-1);
		if (N1 >= -0.02 && T1 >= -0.02 && T1 <= limit + 0.02)
		{
			Moment = (N2 - N1) * R * tan(alpha);
			MomentSet[j] = Moment;
			j++;
		}
	}

	if (j > 1)
	{
		M1 = MomentSet[0];
		for (int k = 0; k < j - 1; k++)
		{
			M1 = max(M1, MomentSet[k + 1]);
			M1 = M1 - F_actual * cos(phi) * offset; // loading point offset from the intersection of the tangent lines
		}
	}
	else
		M1 = 0;
	return M1;
}

double ConeMin(double alpha, double phi, double F_actual, double limit, double R, double offset)
{
	double MomentSet[6] = { NAN,NAN,NAN,NAN,NAN,NAN };
	double T1, T2, N1, N2, Moment, M1;
	int j = 0;

	// case 1: T2 and two compressions
	if (1)
	{
		T1 = 0;
		T2 = limit;
		N1 = ((T2 * sin(alpha) - F_actual * sin(phi)) / cos(alpha) + (T2 * cos(alpha) - F_actual * cos(phi)) / sin(alpha)) / 2;
		N2 = ((T2 * sin(alpha) - F_actual * sin(phi)) / cos(alpha) - (T2 * cos(alpha) - F_actual * cos(phi)) / sin(alpha)) / 2;
		if (N1 >= -0.02 && N2 >= -0.02)
		{
			Moment = (N2 - N1) * R * tan(alpha);
			MomentSet[j] = Moment;
			j++;
		}
	}



	//case 2: T1 and two compressions
	if (1)
	{
		T1 = limit;
		T2 = 0;
		N1 = ((T1 * sin(alpha) - F_actual * sin(phi)) / cos(alpha) + (-T1 * cos(alpha) - F_actual * cos(phi)) / sin(alpha)) / 2;
		N2 = ((T1 * sin(alpha) - F_actual * sin(phi)) / cos(alpha) - (-T1 * cos(alpha) - F_actual * cos(phi)) / sin(alpha)) / 2;
		if (N1 >= -0.02 && N2 >= -0.02)
		{
			Moment = (N2 - N1) * R * tan(alpha);
			MomentSet[j] = Moment;
			j++;
		}
	}



	// case 3: two tensions and N2, T1 is limit
	if (1)
	{
		T1 = limit;
		N1 = 0;
		N2 = F_actual * cos(phi) * sin(alpha) + T1 * cos(alpha) * sin(alpha) - F_actual * sin(phi) * cos(alpha) + T1 * sin(alpha) * cos(alpha);
		T2 = F_actual * cos(phi) * cos(alpha) + T1 * cos(alpha) * cos(alpha) + F_actual * sin(phi) * sin(alpha) - T1 * sin(alpha) * sin(alpha);
		if (N2 >= -0.02 && T2 >= -0.02 && T2 <= limit + 0.02)
		{
			Moment = (N2 - N1) * R * tan(alpha);
			MomentSet[j] = Moment;
			j++;
		}
	}


	// case 4: two tensions and N2, T2 is limit
	if (1)
	{
		T2 = limit;
		N1 = 0;
		N2 = (F_actual * cos(phi) * sin(alpha) - T2 * cos(alpha) * sin(alpha) + F_actual * sin(phi) * cos(alpha) - T2 * sin(alpha) * cos(alpha)) / (-cos(2 * alpha));
		T1 = (F_actual * cos(phi) * cos(alpha) - T2 * cos(alpha) * cos(alpha) + F_actual * sin(phi) * sin(alpha) - T2 * sin(alpha) * sin(alpha)) / (-cos(2 * alpha));
		if (N2 >= -0.02 && T1 >= -0.02 && T1 <= limit + 0.02)
		{
			Moment = (N2 - N1) * R * tan(alpha);
			MomentSet[j] = Moment;
			j++;
		}
	}

	// case 5: two tensions and N1, T1 is limit
	if (1)
	{
		T1 = limit;
		N2 = 0;
		N1 = (F_actual * cos(phi) * sin(alpha) + T1 * cos(alpha) * sin(alpha) - F_actual * sin(phi) * cos(alpha) + T1 * sin(alpha) * cos(alpha)) / cos(2 * alpha);
		T2 = (F_actual * cos(phi) * cos(alpha) + T1 * cos(alpha) * cos(alpha) - F_actual * sin(phi) * sin(alpha) + T1 * sin(alpha) * sin(alpha)) / cos(2 * alpha);
		if (N1 >= -0.02 && T2 >= -0.02 && T2 <= limit + 0.02)
		{
			Moment = (N2 - N1) * R * tan(alpha);
			MomentSet[j] = Moment;
			j++;
		}
	}


	// case 6: two tensions and N1, T2 is limit
	if (1)
	{
		T2 = limit;
		N2 = 0;
		N1 = (F_actual * cos(phi) * sin(alpha) - T2 * cos(alpha) * sin(alpha) + F_actual * sin(phi) * cos(alpha) - T2 * sin(alpha) * cos(alpha)) / (-1);
		T1 = (F_actual * cos(phi) * cos(alpha) - T2 * cos(alpha) * cos(alpha) - F_actual * sin(phi) * sin(alpha) + T2 * sin(alpha) * sin(alpha)) / (-1);
		if (N1 >= -0.02 && T1 >= -0.02 && T1 <= limit + 0.02)
		{
			Moment = (N2 - N1) * R * tan(alpha);
			MomentSet[j] = Moment;
			j++;
		}
	}


	if (j > 1)
	{
		M1 = MomentSet[0];
		for (int k = 0; k < j - 1; k++)
		{
			M1 = min(M1, MomentSet[k + 1]);
			M1 = M1 - F_actual * cos(phi) * offset; // loading point offset from the intersection of the tangent lines
		}
	}
	else
		M1 = 0;
	return M1;
}



cForceSensor g_ForceSensor;
double ForceData[3];
double TorqueData[3];
double Forcebias[3];
double Torquebias[3];

int main()
{
	g_ForceSensor.Set_Calibration_File_Loc("FT5904.cal");
	g_ForceSensor.Initialize_Force_Sensor("dev1/ai0:5");




  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  int index = 0;
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  //int dxl_goal_position[2] = {DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE};         // Goal position
 // int16_t dxl_goal_current[2] = {DXL_MINIMUM_CURRENT_VALUE, DXL_MAXIMUM_CURRENT_VALUE};         // Goal Current
  int16_t dxl_goal_current = 0;
  int32_t dxl_goal_position = 0;

  uint8_t dxl_error_current = 0;                  // Dynamixel error Current
  uint8_t dxl_error_position = 0;                 // Dynamixel error Position
  uint8_t dxl_error_velocity = 0;                 // Dynamixel error Velocity
  int32_t dxl_present_position = 0;               // Present position
  int32_t dxl_present_velocity = 0;               // Present Velocity
  int16_t dxl_present_current = 0;               // Present position

  int32_t dxl_initial_position = 0;               // Present position


  double kp_impedance = 0.00015;//0.15
  double kd_impedance = 0.2;

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

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    printf("Succeeded to change the baudrate!\n");
  }
  else
  {
    printf("Failed to change the baudrate!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Enable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error_current);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->printTxRxResult(dxl_comm_result);
  }
  else if (dxl_error_current != 0)
  {
    packetHandler->printRxPacketError(dxl_error_current);
  }
  else
  {
    printf("Dynamixel has been successfully connected \n");
  }
  using namespace std;
  std::ofstream myfile;
  myfile.open("Plots.txt");
  clock_t startTime = clock(); //Start timer
  clock_t testTime;
  clock_t timePassed;
  double impedence_torque;
  double velthreshold=4;
  float counterclock;
  float hardstop = 600;
  float oldvel;

  //Cone variables 
  double M1;
  double M2;
  double alpha = 11.4*PI / 180;
  double phi;
  double F_actual;
  double limit = 20;
  double R = 0.11;
  double offset = 0.0;

  // Second time Cone Variables Trial 8 

  //Cone variables 

  alpha = 12.5*PI / 180;
  limit = 15;
  R = 0.115;
  offset = 0.03779;


  myfile << "stoptorque" << ' ';
  myfile << "goal_current" << ' ';
  myfile << "current" << ' ';
  myfile << "velocity" << ' ';
  myfile << "position" << ' ';
  myfile << "Fx" << ' ';
  myfile << "Fy" << ' ';
  myfile << "Fz" << ' ';
  myfile << "Mx" << ' ';
  myfile << "My" << ' ';
  myfile << "Mz" << ' ';
  myfile << "counterclock" << ' ';
  myfile << "Time" << endl;


  int i = g_ForceSensor.AcquireFTData();
  g_ForceSensor.GetForceReading(ForceData);
  g_ForceSensor.GetTorqueReading(TorqueData);
  Forcebias[0] = ForceData[0];
  Forcebias[1] = ForceData[1];
  Forcebias[2] = ForceData[2];
  Torquebias[0] = TorqueData[0];
  Torquebias[1] = TorqueData[1];
  Torquebias[2] = TorqueData[2];
  counterclock = 1;
  // Read intial position
  dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_initial_position, &dxl_error_current);
  if (dxl_comm_result != COMM_SUCCESS)
  {
	  packetHandler->printTxRxResult(dxl_comm_result);
  }
  else if (dxl_error_current != 0)
  {
	  packetHandler->printRxPacketError(dxl_error_current);
  }
  dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, HOME_OFFSET, dxl_initial_position, &dxl_error_current);
  if (dxl_comm_result != COMM_SUCCESS)
  {
	  packetHandler->printTxRxResult(dxl_comm_result);
  }
  else if (dxl_error_current != 0)
  {
	  packetHandler->printRxPacketError(dxl_error_current);
  }

  //printf("Force 1: %f\n", Forcebias[0]);
  float stoptorque;
  float x0;
  x0 = 0;
  stoptorque = 0;

  while(1)
  {
	  int i = g_ForceSensor.AcquireFTData();
	  g_ForceSensor.GetForceReading(ForceData);
	  g_ForceSensor.GetTorqueReading(TorqueData);
	  //printf("Forcebefore %f\n ", ForceData[0]);
	  ForceData[0]= ForceData[0] - Forcebias[0];
	  ForceData[1] = ForceData[1] - Forcebias[1];
	  ForceData[2] = ForceData[2] - Forcebias[2];
	  TorqueData[0] = TorqueData[0] - Torquebias[0];
	  TorqueData[1] = TorqueData[1] - Torquebias[1];
	  TorqueData[2] = TorqueData[2] - Torquebias[2];
	  //printf("Forcebias %f\n ", Forcebias[0]);
	  //printf("Forceafter %f\n ", ForceData[0]);



    //printf("Press any key to continue! (or press ESC to quit!)\n");
   // if (getch() == ESC_ASCII_VALUE)
     // break;

    // Write goal position
    //dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_POSITION, dxl_goal_position[index], &dxl_error_current);



   // do
   // {
      // Read present position
      dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error_current);
	  if (dxl_comm_result != COMM_SUCCESS)
	  {
		  packetHandler->printTxRxResult(dxl_comm_result);
	  }
	  else if (dxl_error_current != 0)
	  {
		 packetHandler->printRxPacketError(dxl_error_current);
	  }

	  // Read present Velocity 
	  dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_VELOCITY, (uint32_t*)&dxl_present_velocity, &dxl_error_velocity);
	  if (dxl_comm_result != COMM_SUCCESS)
	  {
		  packetHandler->printTxRxResult(dxl_comm_result);
	  }
	  else if (dxl_error_current != 0)
	  {
		 packetHandler->printRxPacketError(dxl_error_current);
	  }

	  // Read present current
	  dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_CURRENT, (uint16_t*)&dxl_present_current, &dxl_error_current);

      if (dxl_comm_result != COMM_SUCCESS)
      {
        packetHandler->printTxRxResult(dxl_comm_result);
      }
      else if (dxl_error_current != 0)
     {
        packetHandler->printRxPacketError(dxl_error_current);
      }

	  testTime = clock();
	  timePassed = testTime - startTime;


if (dxl_present_velocity > velthreshold || dxl_present_velocity < -velthreshold)
	//if(1)
{
//if (abs(dxl_present_position-dxl_initial_position) >hardstop)
	//dxl_goal_current = 0;
//else 

	//double Arm = 0.063;
	
	//double conetorque;
	float currentamp;
	float a = 0.0173;
	float b;
	b = TorqueData[1] - (a*ForceData[0]);
	//b = 0;
	float c = -0.01;
	//float c = 0;
	float d = 0.2;
	x0 = (d - b) / (a - c);
	double y0;
	if (ForceData[0] < 0)
		c = -c;
	//y0 = c*x0 + d;

    phi = atan2(ForceData[2], ForceData[0]);
	F_actual = sqrt((ForceData[0]* ForceData[0]) + (ForceData[2] * ForceData[2]));
	M1 = ConeMax(alpha, phi, F_actual, limit, R, offset);
	M2 = ConeMin(alpha, phi, F_actual, limit, R, offset);

	//stoptorque = 150;
	//y0 = c*ForceData[0] + d;

	int sign = abs(dxl_present_velocity) / dxl_present_velocity;
	if (sign>0)
		y0 = M1;
	if (sign<0)
		y0 = -M1;

	stoptorque = y0;
	//stoptorque = 0.1*stoptorque;
	//stoptorque = stoptorque;
	//stoptorque = 0.2;
	dxl_goal_current = -stoptorque/ (0.0056);


	//printf("stoptorque: %03f \n", stoptorque);
	//dxl_goal_current = 40;
	//printf("Currentamp: %03d GoalCurrent: %03d Force 1: %03f \n", currentamp,dxl_goal_current, ForceData[0]);
	//dxl_goal_current = (dxl_goal_current /20) ;
}

else 
{
	if (abs(dxl_present_position - dxl_goal_position) < DXL_MOVING_STATUS_THRESHOLD)
		impedence_torque = 0;
	float currentamp;
	stoptorque = (double)kp_impedance*((double)dxl_present_position - (double)dxl_initial_position);
	//printf("first:%03f kd:%03f Impedence:%03f\n",(dxl_present_position-dxl_goal_position),(dxl_present_velocity-0), impedence_torque);
	dxl_goal_current = -stoptorque / (0.0056);
	dxl_goal_current = 0;
}

oldvel = dxl_present_velocity;


	  if (dxl_goal_current>500)
	  {
	  printf("goal current too high");
		  printf("high goal current: %03d \n", dxl_goal_current);
		dxl_goal_current =500;
	  }



	  // Write goal current updates in each loop 
	  dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_CURRENT, dxl_goal_current, &dxl_error_current);

	  if (dxl_comm_result != COMM_SUCCESS)
	  {
		  packetHandler->printTxRxResult(dxl_comm_result);
	  }
	  else if (dxl_error_current != 0)
	  {
		  packetHandler->printRxPacketError(dxl_error_current);
	  }

	  //printf("InitialPos:%03d  PresPos:%03d Diff:%03d \n", dxl_initial_position,dxl_present_position, dxl_initial_position-dxl_present_position);

	  printf("[ID:%03d] GoalCurrent:%03d  PresCurrent:%03d PresVel:%03d PresPos:%03d  Force 1: %f \n", DXL_ID, dxl_goal_current, dxl_present_current, dxl_present_velocity,dxl_present_position - dxl_initial_position, ForceData[0]);

	  //printf("Force 1: %f\t Time: %f\r", ForceData[2]);


	  myfile << stoptorque << ' ';
	  myfile << dxl_goal_current << ' ';
	  myfile << dxl_present_current << ' ';
	  myfile << dxl_present_velocity << ' ';
	  myfile << dxl_present_position << ' ';
	  myfile << ForceData[0] << ' ';
	  myfile << ForceData[1] << ' ';
	  myfile << ForceData[2] << ' ';
	  myfile << TorqueData[0] << ' ';
	  myfile << TorqueData[1] << ' ';
	  myfile << TorqueData[2] << ' ';
	  myfile << counterclock<< ' ';
	  myfile << (1000 * timePassed / CLOCKS_PER_SEC) << endl;
  }

  // Disable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error_current);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->printTxRxResult(dxl_comm_result);
  }
  else if (dxl_error_current != 0)
  {
    packetHandler->printRxPacketError(dxl_error_current);
  }

  // Close port
  portHandler->closePort();

  return 0;
}
