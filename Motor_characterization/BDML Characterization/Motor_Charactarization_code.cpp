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

#define DXL_MOVING_STATUS_THRESHOLD      0                // Dynamixel moving status threshold
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


	double kp_impedance = 0.25;//0.15
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
	double velthreshold = 10;
	float counterclock;
	float hardstop = 600;
	float oldvel;

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
	//Write current position as goal position 
	dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error_position);
	dxl_goal_position = dxl_present_position;


	//printf("Force 1: %f\n", Forcebias[0]);
	float stoptorque;
	float x0;
	x0 = 0;
	stoptorque = 0;
	int counter = 0;

	while (1)
	{
		int i = g_ForceSensor.AcquireFTData();
		g_ForceSensor.GetForceReading(ForceData);
		g_ForceSensor.GetTorqueReading(TorqueData);
		//printf("Forcebefore %f\n ", ForceData[0]);
		ForceData[0] = ForceData[0] - Forcebias[0];
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



		if (counter < 50)
		{
			dxl_goal_current = 0;
		}
		else if (49< counter && counter < 100)
		{
			dxl_goal_current = 5;
		}
		else if (99< counter && counter < 150)
		{
			dxl_goal_current = 10;
		}
		else if (149< counter && counter < 200)
		{
			dxl_goal_current = 15;
		}
		else if (199< counter && counter < 250)
		{
			dxl_goal_current = 20;
		}
		else if (249< counter && counter < 300)
		{
			dxl_goal_current = 25;
		}
		else if (299< counter && counter < 350)
		{
			dxl_goal_current = 30;
		}
		else if (349< counter && counter < 400)
		{
			dxl_goal_current = 35;
		}
		else if (399< counter && counter < 450)
		{
			dxl_goal_current = 40;
		}
		else if (449< counter && counter < 500)
		{
			dxl_goal_current = 45;
		}
		else if (499< counter && counter < 550)
		{
			dxl_goal_current = 50;
		}
		else if (549< counter && counter < 600)
		{
			dxl_goal_current = 55;

		}
		else if (599< counter && counter < 650)
		{
			dxl_goal_current = 60;
		}
		else if (649< counter && counter < 700)
		{
			dxl_goal_current = 65;
		}
		else if (699< counter && counter < 750)
		{
			dxl_goal_current = 70;
		}
		else if (749< counter && counter < 800)
		{
			dxl_goal_current = 75;
		}
		else if (799 < counter && counter < 850)
		{
			dxl_goal_current = 80;
		}
		else if (849< counter && counter < 900)
		{
			dxl_goal_current = 85;
		}
		else if (899< counter && counter < 950)
		{
			dxl_goal_current = 90;
		}
		else if (949< counter && counter < 1000)
		{
			dxl_goal_current = 95;
		}
		else if (999< counter && counter < 1050)
		{
			dxl_goal_current = 100;
		}
		else if (1049< counter && counter < 1100)
		{
			dxl_goal_current = 105;
		}
		else if (1099< counter && counter < 1150)
		{
			dxl_goal_current = 110;
		}
		else if (1149< counter && counter < 1200)
		{
			dxl_goal_current = 115;
		}
		else if (1199< counter && counter < 1250)
		{
			dxl_goal_current = 120;
		}
		else if (1249< counter && counter < 1300)
		{
			dxl_goal_current = 125;
		}
		else if (1299< counter && counter < 1350)
		{
			dxl_goal_current = 130;
		}
		else if (1349< counter && counter < 1400)
		{
			dxl_goal_current = 135;
		}
		else if (1399< counter && counter < 1450)
		{
			dxl_goal_current = 140;
		}
		else if (1449< counter && counter < 1500)
		{
			dxl_goal_current = 145;
		}
		else if (1499< counter && counter < 1550)
		{
			dxl_goal_current = 150;
		}


		else
		{
			dxl_goal_current = 0;
		}

		counter++;
		if (counter == 1600)
			counter = 0;

		if (dxl_goal_current>500)
		{
			printf("goal current too high");
			printf("high goal current: %03d \n", dxl_goal_current);
			dxl_goal_current = 500;
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

		printf("[ID:%03d] GoalCurrent:%03d  PresCurrent:%03d PresVel:%03d PresPos:%03d  Force 1: %f \n", DXL_ID, dxl_goal_current, dxl_present_current, dxl_present_velocity, dxl_present_position - dxl_initial_position, ForceData[0]);

		//printf("Force 1: %f\t Time: %f\r", ForceData[2]);



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
		myfile << counterclock << ' ';
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
