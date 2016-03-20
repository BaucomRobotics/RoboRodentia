//*************************************************************************************
/** @file    task_MComm.cpp
 *  @brief   A task that manages communication to the slave for the master NXT
 *  @details This task initializes the RS485 cable between the microcontrollers
 * 	     and handles all messaging between the two of them.
 *
 *  Revised:
 *     \li 03-04-2015 ARB Original file
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
 * Include C/C++ Files
 **************************************************************************************/
#include "../../yagarto-old/arm-none-eabi/include/c++/4.6.2/cstring"

/**************************************************************************************
 * Include ECROBOT Files
 **************************************************************************************/
#include <Rs485.h>
#include <Speaker.h>
#include <Lcd.h>

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
 * **************************************************************************************/
#define TIMEOUT 50


/**************************************************************************************
 * Global Variables
 **************************************************************************************/
TaskShare<bool> CommReady;
TaskShare<U8> ShareMsgID;
TaskShare<bool> MsgReady2Get;
TaskShare<bool> MsgReady2Send;

ecrobot::Speaker mSpeak;


/**************************************************************************************
 * Send Ack
 **************************************************************************************/
/** @brief   Send acknowledgement of message
 *  @details 
 *  
 */

void SendAck(void)
{
	MessageClass AckMsg;

	MessageClass::comDataID   msgID = MessageClass::idAckMsg;
 
	AckMsg.SendMsgSimple(msgID);
}



/**************************************************************************************
 * Wait for Ack
 **************************************************************************************/
/** @brief    Was there acknowledgement of message?
 *  @details 
 *  
 */

bool isAck(void)
{
	bool ack = false;	
	MessageClass AckMsg;	
	MessageClass::comDataID   msgID = MessageClass::idNoMsg;
	
	msgID = AckMsg.GetMsgSimple();
		
		if(msgID == MessageClass::idAckMsg)
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

U8 curLine = 1;

void debug (const char* msg)
{
	

	Display.clearRow(curLine);
	Display.cursor(0,curLine);
	Display.putf("s\n", msg);
	Display.disp();
	
	curLine++;
	if(curLine > 7) curLine = 1;
}


void debugnum (U8 msg, U8 dir)
{

	Display.clearRow(curLine);
	Display.cursor(0,curLine);
	Display.putf("dsd\n", msg,0, ",", dir,0);
	Display.disp();
	
	curLine++;
	if(curLine > 7) curLine = 1;
}

/**************************************************************************************
 * Task Comm Constructor
 **************************************************************************************/
/** @brief   Constructor for the comm task
 *  @details Sets up Wake message object and sends it to the slave to check if it is alive
 * 			 It waits for ack from the slave before proceeding
 *  
 */

void CommConstructor(void)
{
	//Initialze Shared Variables
	ShareMsgID.put(0);
	MsgReady2Get.put(false);
	MsgReady2Send.put(false);
	
	MessageClass WakeMsg;
	
	//Wait for Wake Message	
	while  (WakeMsg.GetMsgSimple() != MessageClass::idWakeMsg)
	{
		NNxt::sleep(200);
	}
	
	SendAck();
	CommReady.put(true);
	
	//Comm is now ready for operation.	
	//Notify user
	//mSpeak.playTone(500,50,20);
	Display.cursor(0,COMM_LINE);
	Display.putf("s\n", "Comm Ready");
	Display.disp();
	
}



/**************************************************************************************
 * Task Comm Run Method (infinte loop)
 **************************************************************************************/
/** @brief   Run method for the comm task
 *  @details Runs a loop which sends messages to the slave. It checks to make sure
 * 			 Ack is recieved and then waits until the slave has completed the requested actions
*/


void CommRun(void)
{
	//Task Vars
	enum state_t {IDLE, SEND, GET} state = IDLE;
	U32 currentTime;
	
	//Message Vars
	MessageClass* p_curMsg = new MessageClass;
	MessageClass::comDataID   msgID;

	
	//Go forever!
	while(true)
	{
		
		currentTime = NNxt::getTick();
		
		switch (state)
		{
			
			case IDLE:
				
				//Check if a there is an incomming message
				p_curMsg -> GetMsgSimple();
				if (p_curMsg -> isEmptySimple() == false)
				{
					state = GET;
					break;
				}
				
				//Check if there is a message to send
				if (MsgReady2Send.get() == true)
				{
					state = SEND;
				}
				
				break;
			
			
			case SEND:
				
				//Send the message
				msgID = static_cast<MessageClass::comDataID> (ShareMsgID.get());
				p_curMsg -> SendMsgSimple(msgID);
			
// 				debugnum(ShareMsgID.get(),1);
				
				MsgReady2Send.put(false);
				state = IDLE;
				break;
				
			
			case GET:
				
				//Get the message info
				msgID = p_curMsg -> GetMsgDataSimple();
				
				//Get message info
				ShareMsgID.put(msgID);
				MsgReady2Get.put(true);
				
// 				debugnum(ShareMsgID.get(),0);

				state = IDLE;	
			
				break;
					
		}//End switch
		
		//Let other tasks run
		sleep_from_for(currentTime, 10);
		
	}//End while
}
	



/**************************************************************************************
 * Task Comm
 **************************************************************************************/
/** @brief   Comm task
 *  @details Waits until the start variable is set. Then runs the 
*	     constructor and the run method. The run method will 
*	     *never* exit. And if it somehow does the task will just
*	      exit.  
 */


extern "C"
{


TASK(CommTask)
{
	//Wait until permission to start is given

	while(task_CommStart.get() != true) 
	{	
		//Let other tasks run
		NNxt::sleep(100);
	}
	
	//Runs once
	CommConstructor();

	//This loops forever
	CommRun();
	
	//shouldn't ever get here
	TerminateTask();
}

}
