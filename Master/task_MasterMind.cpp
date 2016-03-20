//*************************************************************************************
/** @file    task_MasterMind.cpp
 *  @brief   High level management task of a robot.
 *  @details This is a task which handles high level management of a robot
 * 			 which is being entered in a competition at Cal Poly, SLO. It's primary
 * 			 responsibilites are navigation and movement as well as commanding the slave
 * 			 nxt.
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
 * Include ECROBOT Files
 **************************************************************************************/
#include <Lcd.h>
#include <Speaker.h>

/**************************************************************************************
 * Include Personally Written Files
 **************************************************************************************/
#include "../lib/taskshare.hpp"
#include "shares.hpp"
#include "../lib/ExtraFunctions.hpp"
#include "../lib/MessageClass.hpp"

/**************************************************************************************
 * Include NXTexpanded Lib Files
 **************************************************************************************/
#include "../../nxtOSEK/NXtpandedLib/src/NNxt.hpp"


/**************************************************************************************
 * Constants
 **************************************************************************************/


/**************************************************************************************
 * Global Vars
 **************************************************************************************/
ecrobot::Speaker mSpeak;





/**************************************************************************************
 * Send a message to slave
 **************************************************************************************/
/** @brief   Send a message to the slave
 *  @details Used to easily send a message to the slave
 * 		 	 nxt.
 * @param    msgID The message id from @c MessageClass corresponding to the 
 * 			 message to send.
 * 		
 */

void SendMsg(MessageClass::comDataID msgID)
{
	ShareMsgID.put((U8) msgID);
	MsgReady2Send.put(true);	
}



/**************************************************************************************
 * Wait for Ack
 **************************************************************************************/
/** @brief    Was there acknowledgement of message?
 *  @details 
 *  
 */

bool WaitForMsg(MessageClass::comDataID msgID)
{
	bool ack = false;	
	
	if (MsgReady2Get.get() && ShareMsgID.get() == (U8) msgID)
	{
		ack = true;
	}
	
	return ack;	
}



/**************************************************************************************
 * Easy function to write debug msgs
 **************************************************************************************/
/** @brief   Write sequential debug msgs
 *  @details This function writes messages on the screen one line
 * 			 at a time and then loops back and erases old stuff after a while.
 * @param    msg The message to write
 * 		
 */


void debug (const char* msg)
{
	static U8 curLine = 1;

	Display.clearRow(curLine);
	Display.cursor(0,curLine);
	Display.putf("s\n", msg);
	Display.disp();
	
	curLine++;
	if(curLine > 7) curLine = 1;
}



/**************************************************************************************
 * Task Mastermind Constructor
 **************************************************************************************/
/** @brief   Constructor for MasterMind task
 *  @details Will start all other tasks that need to be run and wait for them 
 * 			 to perform initialization before starting the main code exectution
 *  
 */

void constructor(void)
{
	task_CommStart.put(true);
	
	//Wait till comm task is done initializing
	while(CommReady.get() == false) {NNxt::sleep(50);}
	
	//Get nav system ready
	task_NavStart.put(true);
	
	//Wait until initializaiton of slave is complete
	while(MsgReady2Get.get() == false) {NNxt::sleep(50);}
	
	//Make sure we recieved the init done message
	if (ShareMsgID.get() == (U8) MessageClass::idInitDone)
	{
		Display.cursor(0,MIND_LINE);
		Display.putf("s\n", "MasterMind Ready");
		Display.disp();
	}
	//Otherwise throw an error
	else
	{
		mSpeak.playTone(500,1000,20);
		Display.cursor(0,MIND_LINE);
		Display.putf("s\n", "ERROR!!!");
		Display.cursor(0,DEBUG);
		Display.putf("d\n", ShareMsgID.get(),0);
		Display.disp();
		
		NNxt::sleep(10000);
	}
	
	task_LFStart.put(true);
	
}



/**************************************************************************************
 * Task Mastermind Run Method (infinte loop)
 **************************************************************************************/
/** @brief   Run loop for MasterMind task
 *  @details Will control the high level management of the robot
 *  
 */


void run(void)
{
	Display.clear(true);
	Display.putf("s\n", "Master Running");
	Display.disp();
	
	task_NavState.put(NAV_TO_SUPPLY);
	while (task_NavState.get() == NAV_TO_SUPPLY) {NNxt::sleep(50);}
	
	task_NavState.put(NAV_APPROACH_WALL);
	while (task_NavState.get() == NAV_APPROACH_WALL) {NNxt::sleep(50);}
	
	
	
// 	SendMsg(MessageClass::idPrepForGrabRings);
// 	
// 	while (WaitForMsg(MessageClass::idReadytoGrab) == false) {NNxt::sleep(50);}
// 	
// 	
// 	
// 	NNxt::sleep(1000);
// 	
// 	task_NavState.put(NAV_APPROACH_WALL);
// 	
// 	while (task_NavState.get() == NAV_APPROACH_WALL) {NNxt::sleep(50);}
// 	
// 	NNxt::sleep(1000);
// 	
// 	
// 	
// 	SendMsg(MessageClass::idGrabRings);
// 	
// 	while (WaitForMsg(MessageClass::idGrabbedRings) == false) {NNxt::sleep(50);}
// 	
// 	NNxt::sleep(1000);
// 	
// 	
// 	task_NavState.put(NAV_BACK_UP);
// 	
// 	while (task_NavState.get() == NAV_BACK_UP) {NNxt::sleep(50);}
// 	
// 	NNxt::sleep(1000);
// 	
// 	
// 	
// 	SendMsg(MessageClass::idPrepForGrabRings);
// 	
// 	while (WaitForMsg(MessageClass::idReadytoGrab) == false) {NNxt::sleep(50);}
// 	
	
// 	
// 	//Tell Slave to grab
// 	SendMsg(MessageClass::idGrabRings);
// 	
// 	debug("GRAB");
// 	
// 	while (WaitForMsg(MessageClass::idGrabbedRings) == false) {NNxt::sleep(50);}
// 	
// 	NNxt::sleep(1500);
// 	
// 	
// 	//Tell Slave to prep for placement
// 	SendMsg(MessageClass::idPrepForPlacement);
// 	
// 	debug("PREP2PLACE");
// 	
// 	while (WaitForMsg(MessageClass::idReadytoPlace) == false) {NNxt::sleep(50);}
// 	
// 	NNxt::sleep(1500);
// 	
// 	
// 	//Tell Slave to place rings
// 	SendMsg(MessageClass::idPlaceRings);
// 	
// 	debug("PLACE");
// 	
// 	while (WaitForMsg(MessageClass::idPlacedRings) == false) {NNxt::sleep(50);}
// 	
// 	NNxt::sleep(1500);	
// 	
// 	//Tell Slave to prep for grab
// 	SendMsg(MessageClass::idPrepForGrabRings);
// 	
// 	debug("PREP2GRAB");
// 	
// 	while (WaitForMsg(MessageClass::idReadytoGrab) == false) {NNxt::sleep(50);}
// 	
// 	NNxt::sleep(1000);
	
	
// 	U32 currentTime;
// 		
// 	while(true)
// 	{
// 		currentTime = NNxt::getTick();		
// 		
// 		//Let other tasks run		
// 		sleep_from_for(currentTime, 100);
// 	}
}





/**************************************************************************************
 * Task Mastermind
 **************************************************************************************/
/** @brief   Master Mind task
 *  @details Waits until the start variable is set. Then runs the 
*	     constructor and the run method. The run method will 
*	     *never* exit. And if it somehow does the task will just
*	      exit.  
 */

extern "C"
{


TASK(MasterMind)
{
	
	while(task_MasterMindStart.get() == false) 
	{
		//Let other tasks run		
		NNxt::sleep(100);
	}
	
	constructor();

	run();
	
	TerminateTask();
}

}