//*************************************************************************************
/** @file    shares.hpp
 *  @brief   File to house all the shared variables for the Master nxt
 *  @details Any shared variables which need to be externally avaiable to other
 * 			 files in this project will be declared below. This header will then
 * 			 be included in any other file which needs access to these variables.
 *
 *  Revised:
 *     \li 02-16-2015 ARB Original file
 *
 *  License:
 *		
 *   Copyright (C) 2015 Alex Baucom
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 * 
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.*/
//*************************************************************************************


/**************************************************************************************
 * PORT DEFINITIONS
 **************************************************************************************/

#include <Port.h>

//Master Sensors
extern const ePortS AuxLightPort; 
extern const ePortS MainLightPort;
extern const ePortS SonarPort;
extern const ePortS CommPort;

//Master Motors
extern const ePortM RightWheelPort;
extern const ePortM LeftWheelPort;



/**************************************************************************************
 * Display Stuff
 **************************************************************************************/

#include <Lcd.h>

//Create Lcd Resource - might need to use a mutex for this if issues arise
extern ecrobot::Lcd Display;

//LCD line definitions for each task (i.e. which line they are allowed to use)
#define INIT_LINE 0
#define MIND_LINE 1
#define NAV_LINE  2
#define COMM_LINE 3
#define DEBUG 4



/**************************************************************************************
 * Shared Global Variables
 **************************************************************************************/
#include "../lib/taskshare.hpp"

//------------MasterMind----------------

extern TaskShare<bool> task_MasterMindStart;

//------------Comm----------------
extern TaskShare<bool> task_CommStart;

//----------Nav----------------
#define NAV_IDLE 0
#define NAV_TO_SUPPLY 1
#define NAV_APPROACH_WALL 2
#define NAV_BACK_UP 3
#define NAV_TURN_AROUND 4
#define NAV_TO_SCORE   5

extern TaskShare<bool> task_NavStart;

extern TaskShare<U8> task_NavState;


//----------LineFollow----------------
extern TaskShare<bool> task_LFStart;

extern TaskShare<S16> black_limit;

//-----------Comm-----------------
//Shared vaiable to tell Comm task when to start
extern TaskShare<bool> task_CommStart;

//Shared vaiable to tell others when the comm task is ready
extern TaskShare<bool> CommReady;

//Shared variable containing the dataID information so the reciever knows what to do
extern TaskShare<U8> ShareMsgID;

//Shared vaiable to tell others when a message is ready to be recieved (Comm->SMind)
extern TaskShare<bool> MsgReady2Get;

//Shared vaiable to tell others when a message is ready to be sent (SMind->Comm)
extern TaskShare<bool> MsgReady2Send;






