//*************************************************************************************
/** @file    task_SlaveMind.cpp
 *  @brief   High level management task of a robot.
 *  @details This is a task which handles high level management of a robot
 * 			 which is being entered in a competition at Cal Poly, SLO. This 
 * 			 task is a slave to the master task and will recieve orders via the 
 * 			 serial communication cable/task. It's primary responsibilities include
 * 			 the grabber, lifter, and tower motors as well as the touch sensors
 * 			 associated with these parts.
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
 * Include C/C++ Files
 **************************************************************************************/
#include "../../yagarto-old/arm-none-eabi/include/c++/4.6.2/string"

/**************************************************************************************
 * Include ECROBOT Files
 **************************************************************************************/
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
 * Defined positions for lifter (in inches)
 **************************************************************************************/

//Old values, staying here for reference
// #define SUPPLY_LOWER      3.75
// #define SUPPLY_HIGHER     4.5
// 
// #define LOW_SCORE_LOWER   3.75
// #define LOW_SCORE_HIGHER  4.5
// 
// #define MID_SCORE_LOWER   7.75
// #define MID_SCORE_HIGHER  8.5
// 
// #define HIGH_SCORE_LOWER  11.5
// #define HIGH_SCORE_HIGHER 12.1

#define PRE_GRAB_HEIGHT   2.5
#define GRAB_HEIGHT       3.25
#define POST_GRAB_HEIGHT  3.75

#define PRE_RELEASE_HEIGHT  11.5
#define RELEASE_HEIGHT   	11.0
#define POST_RELEASE_HEIGHT 10.25

//For unit converstion
#define SCALE 571.51
#define ZERO  -1516

/**************************************************************************************
 * Converstion from height to degrees for lifter
 **************************************************************************************/
/** @brief   Convertion between height and degrees
 *  @details Used for the lifter run method to know how 
 * 			 many encoder ticks to tell the lifter method to turn
 * 			 based on how high, in inches, the claw needs to move.
 * @param    height The heigh, in inches, to move the claw to
 * @return   The number of encoder ticks the lifter motor needs to rotate
 * 		
 */

S32 InchestoDegrees(float height)
{
	return (height * SCALE) + ZERO;
	
}



/**************************************************************************************
 * Ready to check lifter and claw
 **************************************************************************************/
/** @brief   Let the task know if it is okay to check the lifter and claw
 *  @details Helps ensure that the task doesn't set the lifter or claw
 * 			 to move and then immideiately think they have already arrived.
 * @param    reset Pass true if the function needs to reset, otherwise it defualts to false
 * @return   True if it is okay to check lifter and claw position
 * 		
 */

bool ReadyToCheck(bool reset = false)
{
	static U8 times_called = 0;
	static bool OkaytoCheck = false;

	if (reset == true)
	{
		times_called = 0;
		OkaytoCheck = false;
	}
	
	if(times_called > 2)
	{
		OkaytoCheck = true;
	}
	else
	{
		OkaytoCheck = false;
		times_called++;
	}
	
	return OkaytoCheck;
}


/**************************************************************************************
 * Easy function to move lifter
 **************************************************************************************/
/** @brief   Moves lifter to specified height
 *  @details Easy method to call for lifter motion that 
 * 			 will return true if lifter has reached that position.
 * @param    height The height, in inches, to move the claw to
 * @return   True if lifter has arrived, false if not
 * 		
 */

bool MoveLift(float height)
{
	moveLifterAbs.put(InchestoDegrees(height));
	return (LifterArrived.get() && ReadyToCheck());	
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
 * Task Slavemind Constructor
 **************************************************************************************/
/** @brief   SlaveMind consturctor
 *  @details Initializes all the subsystems of the slave NXT one at a time
 * 		
 */


void SlaveMindConstructor(void)
{	
	//Start Comm Task
	task_CommStart.put(true);
	
	//Wait till comm task is done initializing
	while(CommReady.get() == false) {NNxt::sleep(50);}		
	
	//Get Tower ready
	task_TowerStart.put(true);
	
	//Wait till tower task is done initializing
	while(TowerArrived.get() == false) {NNxt::sleep(50);}	
	
	NNxt::sleep(250);
	
	//Start Lifter Task
	task_LifterStart.put(true);
	
	//Wait till lifter task is done initializing
	while(LifterArrived.get() == false) {NNxt::sleep(50);}	
	
	NNxt::sleep(250);
	
	//Start Claw Task
	task_ClawStart.put(true);
	
	//Wait till claw task is done initializing
	while(ClawArrived.get() == false) {NNxt::sleep(50);}
	
	ShareMsgID.put((U8) MessageClass::idInitDone);
	MsgReady2Send.put(true);
	
	Display.cursor(0,MIND_LINE);
	Display.putf("s\n", "SlaveMind Ready");
	Display.disp();
	
}



/**************************************************************************************
 * Task Slavemind Run Method (infinte loop)
 **************************************************************************************/
/** @brief   SlaveMind run method
 *  @details Currently tells the lifter to move up and down and the claw to
 * 		     open and close so that it can grab some rings. In the future it will 
 * 		     be modified to take messages from the master and move the proper subsystems
 * 		
 */


void SlaveMindRun(void)
{
	
	enum state_t {IDLE, PREP2GRAB, GRAB, PREP2PLACE, PLACE} state = IDLE;

	U32 currentTime;
	MessageClass::comDataID curMsgID = MessageClass::idNoMsg;
	U8 grabStage = 0;
	U8 placeStage = 0;
	
	Display.clear(true);
	Display.putf("s\n", "Slave Running");
	Display.disp();
	
	while(true)
	{		
		currentTime = NNxt::getTick();
		
		switch (state)
		{
			case IDLE:
			
				//debug("IDLE");
				
				//Check if there is a new message
				if(MsgReady2Get.get() == true)
				{
					//Read the new message
					curMsgID = static_cast<MessageClass::comDataID> (ShareMsgID.get());
					
					//Move to proper state
					if (curMsgID == MessageClass::idPrepForGrabRings) state = PREP2GRAB;
					if (curMsgID == MessageClass::idGrabRings) state = GRAB;
					if (curMsgID == MessageClass::idPrepForPlacement) state = PREP2PLACE;
					if (curMsgID == MessageClass::idPlaceRings) state = PLACE;
					
					//Reset shared variables
					MsgReady2Get.put(false);
					ShareMsgID.put(MessageClass::idNoMsg);
				}
				
				break;			
			
			case PREP2GRAB:
				
				//debug("PREP2GRAB");
				
				//Move Lifter to bottom, open claw
				moveClaw.put(OPENCLAW);
				MoveLift(PRE_GRAB_HEIGHT);
				
				//Wait till action is completed
				if(ReadyToCheck() && ClawArrived.get() && LifterArrived.get())
				{
					//Return to idle state
					state = IDLE;
					
					//Reset check block
					ReadyToCheck(true);
					
					//Send message to Master to let it know we are done.
					ShareMsgID.put((U8) MessageClass::idReadytoGrab);
					MsgReady2Send.put(true);
				}				
		
				break;
				
			case GRAB:
												
				//debug("GRAB");
				
				//Raise lifter to grab height
				if(grabStage == 0)
				{
					if(MoveLift(GRAB_HEIGHT)) 
					{
						grabStage = 1;
						
						//Reset check block
						ReadyToCheck(true);
					}
					
				}
				
				//Grab Rings
				if(grabStage == 1)
				{
					moveClaw.put(CLOSECLAW);
					if(ReadyToCheck() && ClawArrived.get() == true) 
					{
						grabStage = 2;
						
						//Reset check block
						ReadyToCheck(true);
					}
					
				}
				
				//Lift off of peg
				if(grabStage == 2)
				{
					if(MoveLift(POST_GRAB_HEIGHT)) 
					{
						grabStage = 0;
						
						//Return to idle state
						state = IDLE;
						
						//Reset check block
						ReadyToCheck(true);
						
						//Send message to Master to let it know we are done.
						ShareMsgID.put((U8) MessageClass::idGrabbedRings);
						MsgReady2Send.put(true);
					}
					
				}				
				
				break;
				
			case PREP2PLACE:

				//debug("PREP2PLACE");				
				
				///Move Lifter to top
				MoveLift(PRE_RELEASE_HEIGHT);
				
				//Wait till action is completed
				if(ReadyToCheck() && LifterArrived.get())
				{
					//Return to idle state
					state = IDLE;
					
					//Reset check block
					ReadyToCheck(true);
					
					//Send message to Master to let it know we are done.
					ShareMsgID.put((U8) MessageClass::idReadytoPlace);
					MsgReady2Send.put(true);
				}
				
				break;
				
			case PLACE:
								
				//debug("PLACE");
				
				//Raise lifter to grab height
				if(placeStage == 0)
				{
					if(MoveLift(RELEASE_HEIGHT)) 
					{
						placeStage = 1;
						
						//Reset check block
						ReadyToCheck(true);
					}
					
				}
				
				//Grab Rings
				if(placeStage == 1)
				{
					moveClaw.put(OPENCLAW);
					if(ReadyToCheck() && ClawArrived.get()) 
					{
						placeStage = 2;
						
						//Reset check block
						ReadyToCheck(true);
					}
					
				}
				
				//Lift off of peg
				if(placeStage == 2)
				{
					if(MoveLift(POST_RELEASE_HEIGHT)) 
					{
						placeStage = 0;
						
						//Return to idle state
						state = IDLE;
						
						//Reset check block
						ReadyToCheck(true);
						
						//Send message to Master to let it know we are done.
						ShareMsgID.put((U8) MessageClass::idPlacedRings);
						MsgReady2Send.put(true);
					}
					
				}	
		
				break;

		}//end switch
		
		//Let other tasks run		
		sleep_from_for(currentTime, 50);
		
	}//end while
}




/**************************************************************************************
 * Task Slavemind
 **************************************************************************************/
/** @brief   SlaveMind task
 *  @details Waits until the start variable is set. Then runs the 
 *			 constructor and the run method (an infinte loop)
 * 		
 */

extern "C"
{


TASK(SlaveMind)
{
	//Wait until permission to start is given
	while(task_SlaveMindStart.get() == false) 
	{
		//Let other tasks run
		NNxt::sleep(100);
	}
	
	SlaveMindConstructor();

	SlaveMindRun();
	
	TerminateTask();
}

}
