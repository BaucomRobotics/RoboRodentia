//*************************************************************************************
/** @file    shares.hpp
 *  @brief   File to house all the shared variables for the Slave nxt
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
 * File Includes
 **************************************************************************************/
#include <Port.h>
#include <Lcd.h>


#include "../lib/taskshare.hpp"


/**************************************************************************************
 * Display Stuff
 **************************************************************************************/

//Create Lcd Resource - might need to use a mutex for this if issues arise
extern ecrobot::Lcd Display;

//LCD line definitions for each task (i.e. which line they are allowed to use)
#define INIT_LINE 0
#define MIND_LINE 1
#define LIFTER_LINE 3
#define CLAW_LINE 4
#define TOWER_LINE 2
#define COMM_LINE 5
#define DEBUG 6


/**************************************************************************************
 * PORT DEFINITIONS
 **************************************************************************************/

//Slave Sensors
extern const ePortS ClawTouchPort;
extern const ePortS BaseTouchPort;
extern const ePortS CommPort;

//Slave Motors
extern const ePortM LifterPort;
extern const ePortM ClawPort;
extern const ePortM TowerPort;


/**************************************************************************************
 * Global shared task variables
 **************************************************************************************/

//------------Slave Mind----------------

//Shared vaiable to tell SlaveMind task when to start
extern TaskShare<bool> task_SlaveMindStart;


//---------Lifter----------------------

//Shared vaiable to tell Lifter task when to start
extern TaskShare<bool> task_LifterStart;

//Shared variable to set lifter position
extern TaskShare<S32> moveLifterAbs;

//Shared variable to say when the lifter has arrived
extern TaskShare<bool> LifterArrived;


//-----------Claw-----------------
//Shared vaiable to tell Claw task when to start
extern TaskShare<bool> task_ClawStart;

//Shared variable to tell the claw task where to move
extern TaskShare<S32> moveClaw;

//Shared variable to tell SlaveMind when the claw has arrived
extern TaskShare<bool> ClawArrived;

//Definitions for moveClaw method
#define OPENCLAW  0
#define CLOSECLAW 1


//-----------Tower-----------------
//Shared vaiable to tell Tower task when to start
extern TaskShare<bool> task_TowerStart;

//Let other tasks know when the tower is initialized
extern TaskShare<bool> TowerArrived;


//-----------Comm-----------------
//Shared vaiable to tell Comm task when to start
extern TaskShare<bool> task_CommStart;

//Shared vaiable to tell others when the comm task is ready
extern TaskShare<bool> CommReady;

//Shared variable containing the dataID information so the reciever knows what to do
//This is sent as a union and decoded by the task getting the info
extern TaskShare<U8> ShareMsgID;

//Shared vaiable to tell others when a message is ready to be recieved (Comm->SMind)
extern TaskShare<bool> MsgReady2Get;

//Shared vaiable to tell others when a message is ready to be sent (SMind->Comm)
extern TaskShare<bool> MsgReady2Send;



