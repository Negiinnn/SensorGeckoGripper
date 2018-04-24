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

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID                          1                   // Dynamixel ID: 1
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      1            // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      4000              // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MINIMUM_CURRENT_VALUE       20           // Dynamixel will rotate between this value
#define DXL_MAXIMUM_CURRENT_VALUE       0            // and

#define DXL_MOVING_STATUS_THRESHOLD      10               // Dynamixel moving status threshold
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

int main()
{
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
  //int16_t dxl_goal_current[2] = {DXL_MINIMUM_CURRENT_VALUE, DXL_MAXIMUM_CURRENT_VALUE};         // Goal position
  int16_t dxl_goal_current = 0; 
  int32_t dxl_goal_position = 0;


  uint8_t dxl_error_current = 0;                  // Dynamixel error Current
  uint8_t dxl_error_position = 0;                 // Dynamixel error Position
  uint8_t dxl_error_velocity = 0;                 // Dynamixel error Velocity
  int32_t dxl_present_position = 0;               // Present position
  int32_t dxl_present_velocity = 0;               // Present Velocity
  int16_t dxl_present_current = 0;               // Present position
 
  double kp_impedance=0.12; // 0.09 liked 
  double kd_impedance=0.0;//.4;

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


//Write current position as goal position 
     dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position,  &dxl_error_position);
dxl_goal_position= dxl_present_position;

  while(1)
  {
    //printf("Press any key to continue! (or press ESC to quit!)\n");
    //if (getch() == ESC_ASCII_VALUE)
      //break;

 // Write goal position
 //   dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_POSITION, dxl_goal_position[index], &dxl_error_position);    
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


   // do
    //{
      // Write Goal Current
      dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_CURRENT, dxl_goal_current, &dxl_error_current);
  
      //else if (count<2000)
      //{goal=(2000-count)*dxl_goal_current[index]/1000;}
      //goal= dxl_goal_current[index]-(((count/100)-10)*((count/100)-10));
      
      // Read present position
      dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position,  &dxl_error_position);
      // Read present Velocity 
	     dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_VELOCITY, (uint32_t*)&dxl_present_velocity,  	&dxl_error_velocity);
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
timePassed = testTime-startTime ;
//timePassed=testTime-timePassed;


printf("[ID:%03d] GoalCurrent:%03d  PresCurrent:%03d time:%03d present position:%03d\n", DXL_ID, dxl_goal_current, dxl_present_current,(double)1000*timePassed/CLOCKS_PER_SEC,dxl_present_position);
      //printf("[ID:%03d] GoalCurrent:%03d  PresCurrent:%03d PresVel:%03d PresPos:%03d Impedence:%03f\n", DXL_ID, dxl_goal_current, dxl_present_current,dxl_present_velocity,dxl_present_position, impedence_torque);

myfile <<dxl_goal_current<<' ';
myfile<<dxl_present_current << ' ';
myfile<<dxl_present_velocity<<' ';
myfile<<dxl_present_position<<' ';
myfile<<impedence_torque<<' ';
myfile<<(1000*timePassed/CLOCKS_PER_SEC)<<endl;

   // }
//while((abs(dxl_goal_current - dxl_present_current) > DXL_CURRENT_STATUS_THRESHOLD));
//while(count<2000);
  if (abs(dxl_present_position-dxl_goal_position)<DXL_MOVING_STATUS_THRESHOLD)
    impedence_torque=0;
    // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error_current);
    // if (dxl_comm_result != COMM_SUCCESS)
    // {
    // packetHandler->printTxRxResult(dxl_comm_result);
    // }
    // else if (dxl_error_current != 0)
    // {
    // packetHandler->printRxPacketError(dxl_error_current);
    // }


  else 
   impedence_torque= (double)-kp_impedance*((double)dxl_present_position-(double)dxl_goal_position)+(double)-kd_impedance*((double)dxl_present_velocity-(double)0);
   //printf("first:%03f kd:%03f Impedence:%03f\n",(dxl_present_position-dxl_goal_position),(dxl_present_velocity-0), impedence_torque);
   dxl_goal_current=(impedence_torque)/(1.5705);
   if (dxl_goal_current>300)
   {
    dxl_goal_current=300;
    printf("goal current too high");
   } 
   
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
