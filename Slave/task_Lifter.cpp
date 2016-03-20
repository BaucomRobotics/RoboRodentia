//*************************************************************************************
/** @file    task_Lifter.cpp
 *  @brief   A task that manages the lifter motor
 *  @details This task intializes the lifter motor once started
 * 			 and then runs a simple PI control loop
 * 			 to move the lifter into the desired position.
 *
 *  Revised:
 *     \li 02-24-2015 ARB Original file
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
#include "../../yagarto-old/arm-none-eabi/include/c++/4.6.2/cmath"

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
#define CLOSE_ENOUGH 10  //Number of encoder ticks deemed close enough to desired position
#define Kp 1
#define Ki 0.001
#define LONG_ENOUGH 5  //Number of times the touch sensor needs to read pressed to turn off motor
#define DOWN_MAX  -70
#define UP_MAX  70
#define OFF 0
#define MIN_POWER 20



/**************************************************************************************
 * Global Variables
 **************************************************************************************/
ecrobot::Motor Lifter (LifterPort);
ecrobot::TouchSensor BaseTouch (BaseTouchPort);
TaskShare<S32> moveLifterAbs;
TaskShare<bool> LifterArrived;


/**************************************************************************************
 * Task Lifter Constructor
 **************************************************************************************/
/** @brief   Constructor for the lifter task
 *  @details Lifts the motor off the touch sensor, then moves
 * 			 it back down on the touch sensor, and zeros the 
 * 			 encoder.
 *  
 */

void LifterConstructor(void)
{
	//Zero the lifter position using the touch sensor
	
	//Stop motor and reset position
	Lifter.reset();
	
	//Set lifter mode to always brake after motion
	Lifter.setBrake(true);
	
	//Start lifter moving up
	Lifter.setPWM(UP_MAX);
	
	//Wait for touch sensor
	while (BaseTouch.isPressed() == true)
	{
		//Let other tasks work while waiting
		NNxt::sleep(1);
	}
	
	//Stop the lifter just after touch sensor triggers
	Lifter.setPWM(OFF);
	
	//Move lifter back down until touch triggers
	Lifter.setPWM(DOWN_MAX);
	
	//Wait for touch sensor
	while (BaseTouch.isPressed() == false)
	{
		//Let other tasks work while waiting
		NNxt::sleep(1);		
	}
	
	//Stop the lifter just after touch sensor triggers
	//This also sets the encoder to 0.
	Lifter.reset();
	
	//Initialze lifter command variable
	moveLifterAbs.put(0);
	
	//Intialize lifter arrive variable
	LifterArrived.put(true);
	
	//Lifter is now ready for operation.	
	//Notify user
	Display.cursor(0,LIFTER_LINE);
	Display.putf("s\n", "Lifter Ready");
	Display.disp();
}



/**************************************************************************************
 * Task Lifter Run Method (infinte loop)
 **************************************************************************************/
/** @brief   Run method for the lifter task
 *  @details Runs a control loop for the lifter motor
 *			 when a new position is requested. Otherwise does nothing.  
 */


void LifterRun(void)
{
	S32 cPos = 0; /*Current Position*/
	S32 dPos = 0; /*Desired Position*/
	S32 oPos = 0; /*Old Desired Position*/
	S32 error = 0; /*Position controller error*/
	S32 error_sum = 0; /*Integrated error for I control*/
	S32 power = 0; /*PWM power to send to motor, max value is +/- 100*/
	U8  at_dPos = 0;    /* check if the lifter is held at the right spot*/
	U32 currentTime;
	enum state_t {IDLE, MOVING} state = IDLE;
	
	ecrobot::Speaker mSpeak;
	
	//Go forever!
	while(true)
	{
		currentTime = NNxt::getTick();
		
		//Get current desired position
		dPos = moveLifterAbs.get();
		
		switch (state)
		{
			case IDLE:
						
				//If there is a new position, start the controller
				if (dPos != oPos)
				{
					LifterArrived.put(false);
					state = MOVING;
					
				}
				
					/*reset values*/
					error = 0; 
					error_sum = 0;
					power = 0;
					at_dPos = 0;  
				
				break;
			
			/*If the motion isn't complete (or a new order was just recieved),
			* start the controller*/
			case MOVING:
				
				//Update current encoder position and compute error
				cPos = Lifter.getCount();
				error = dPos - cPos;
				error_sum += error;
				
				//Check if the lifter is sitting in the right spot
				if (std::abs(error) < CLOSE_ENOUGH) {at_dPos++;}
				else {at_dPos = 0;}
				
				
				/*Check if the lifter has been in the right place for long enough
				* to turn off the controller and deem the motion complete*/
				if (at_dPos > LONG_ENOUGH)
				{
					Lifter.setPWM(OFF);
					LifterArrived.put(true);
					oPos = dPos;
					state = IDLE;
					
					mSpeak.playTone(500,50,20);
				}
				
				//Otherwise, continue running the control loop
				else
				{
					power = Kp * error + Ki * error_sum / 1024;
					
					//Cap max power
					if (power > UP_MAX) {power = UP_MAX;}
					if (power < DOWN_MAX) {power = DOWN_MAX;}
					if (power > 0 && power < MIN_POWER) {power = MIN_POWER;}
					
					//Send control signal
					Lifter.setPWM((S8) power);
				}
				
				
				break;
				
		}//End switch
		
		
		//Let other tasks run
		sleep_from_for(currentTime, 20);
		
	}//End while
}
	



/**************************************************************************************
 * Task Lifter
 **************************************************************************************/
/** @brief   Lifter task
 *  @details Waits until the start variable is set. Then runs the 
 *			 constructor and the run method. The run method will 
 *			 *never* exit. And if it somehow does the task will just
 *			 exit.  
 */


extern "C"
{


TASK(LifterTask)
{
	//Wait until permission to start is given
	while(task_LifterStart.get() != true) 
	{
		//Let other tasks run
		NNxt::sleep(100);
	}
	
	//Runs once
	LifterConstructor();

	//This loops forever
	LifterRun();
	
	//shouldn't ever get here
	TerminateTask();
}

}
