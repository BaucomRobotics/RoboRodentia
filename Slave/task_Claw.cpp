//*************************************************************************************
/** @file    task_Claw.cpp
 *  @brief   A task that manages the claw motor
 *  @details This task intializes the claw motor once started
 * 			 and then runs a simple control loop to open
 * 			 and close the claw
 *
 *  Revised:
 *     \li 02-26-2015 ARB Original file
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


/**************************************************************************************
 * Include ECROBOT Files
 **************************************************************************************/
#include <Lcd.h>
#include <Motor.h>
#include <Speaker.h>
#include <TouchSensor.h>

/**************************************************************************************
 * Include Personally Written Files
 **************************************************************************************/
#include "../lib/taskshare.hpp"
#include "shares.hpp"
#include "../lib/ExtraFunctions.hpp"

/**************************************************************************************
 * Include NXTexpanded Lib Files
 **************************************************************************************/
#include "../../nxtOSEK/NXtpandedLib/src/NNxt.hpp"


/**************************************************************************************
 * Constants
 **************************************************************************************/
#define OPENING_SPEED 35
#define CLOSING_SPEED -35
#define OFF 0
#define HOLDING_SPEED -30



/**************************************************************************************
 * Global Variables
 **************************************************************************************/
ecrobot::Motor Claw (ClawPort);
ecrobot::TouchSensor ClawTouch (ClawTouchPort);
TaskShare<S32> moveClaw;
TaskShare<bool> ClawArrived;


/**************************************************************************************
 * Task Claw Constructor
 **************************************************************************************/
/** @brief   Constructor for the claw task
 *  @details Opens the claw until the touch sensor is reached.
 * 			 Then holds in position.
 *  
 */

void ClawConstructor(void)
{
	//Zero the claw position using the touch sensor
	
	//Stop motor and reset position
	Claw.reset();
	
	//Set claw mode to always brake after motion
	Claw.setBrake(true);
	
	//Start claw opening
	Claw.setPWM(OPENING_SPEED);
	
	//Wait for touch sensor
	while (ClawTouch.isPressed() != true)
	{
		//Let other tasks work while waiting
		NNxt::sleep(1);
	}
	
	//Wait for touch sensor (again)
	while (ClawTouch.isPressed() == true)
	{
		//Let other tasks work while waiting
		NNxt::sleep(1);
	}
	
	//Stop motor
	Claw.setPWM(OFF);
	
	//Initialze claw command variable
	moveClaw.put(0);
	
	//Intialize claw arrive variable
	ClawArrived.put(true);
	
	//claw is now ready for operation.	
	//Notify user
	Display.cursor(0,CLAW_LINE);
	Display.putf("s\n", "Claw Ready");
	Display.disp();
}



/**************************************************************************************
 * Task Claw Run Method (infinte loop)
 **************************************************************************************/
/** @brief   Run method for the claw task
 *  @details Runs a control loop for the claw motor
 *			 when a new position is requested. Otherwise does nothing.  
 */


void ClawRun(void)
{
	enum state_t {CLOSED, CLOSING, OPEN, CHECKTOUCH, OPENING} state = OPEN;
	U32 currentTime;
	
	//Go forever!
	while(true)
	{
		
		currentTime = NNxt::getTick();
		
		switch (state)
		{
			/*Claw is in a closed position and holding ring(s)*/
			case CLOSED:
						
				Claw.setPWM(HOLDING_SPEED);
				if(moveClaw.get() == OPENCLAW)
				{
					state = CHECKTOUCH;
					ClawArrived.put(false);
				}
				
				break;
			
			
			case CHECKTOUCH:
			
				if(ClawTouch.isPressed() == false)
				{
					Claw.setPWM(OPENING_SPEED);
				}
				else
				{
					state = OPENING;
				}
				break;
				
			/*Claw is in the process of opening*/
			case OPENING:
				
				Claw.setPWM(OPENING_SPEED);
				if(ClawTouch.isPressed() == false)
				{
					state = OPEN;
					ClawArrived.put(true);
					Claw.setPWM(OFF);
				}				
				
				break;
				
			/*Claw is open and holding position*/
			case OPEN:
				
				Claw.setPWM(OFF);
				if(moveClaw.get() == CLOSECLAW)
				{
					state = CLOSING;
					ClawArrived.put(false);
				}				
				
				break;
			
			/*Claw is in the process of closing*/	
			case CLOSING:
				
				Claw.setPWM(CLOSING_SPEED);
				if(ClawTouch.isPressed() == true)
				{
					state = CLOSED;
					ClawArrived.put(true);
				}	
								
				break;
				
		}//End switch
		
		
		//Let other tasks run
		sleep_from_for(currentTime, 10);
		
	}//End while
}
	



/**************************************************************************************
 * Task Claw
 **************************************************************************************/
/** @brief   Claw task
 *  @details Waits until the start variable is set. Then runs the 
 *			 constructor and the run method. The run method will 
 *			 *never* exit. And if it somehow does the task will just
 *			 exit.  
 */


extern "C"
{


TASK(ClawTask)
{
	//Wait until permission to start is given

	while(task_ClawStart.get() != true) 
	{	
		//Let other tasks run
		NNxt::sleep(100);
	}
	
	//Runs once
	ClawConstructor();

	//This loops forever
	ClawRun();
	
	//shouldn't ever get here
	TerminateTask();
}

}
