//*************************************************************************************
/** @file    task_Navigation.cpp
 *  @brief   Navigation task to handle robot locomotion
 *  @details This task will be responsible for make sure the robot can
 * 			 move around the course as needed and get to the right spots
 * 			 as requested by the MasterMind task
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
#include "../../yagarto-old/arm-none-eabi/include/c++/4.6.2/cmath"

/**************************************************************************************
 * Include ECROBOT Files
 **************************************************************************************/
#include <Motor.h>
#include <Speaker.h>
#include <LightSensor.h>
#include <NxtColorSensor.h>
#include <SonarSensor.h>

/**************************************************************************************
 * Include Personally Written Files
 **************************************************************************************/
#include "../lib/taskshare.hpp"
#include "shares.hpp"
#include "../lib/ExtraFunctions.hpp"
#include "RobotClass.hpp"

/**************************************************************************************
 * Include NXTexpanded Lib Files
 **************************************************************************************/
#include "../../nxtOSEK/NXtpandedLib/src/NNxt.hpp"


/**************************************************************************************
 * Constants
 **************************************************************************************/

#define DIST2CENTER 	  1  //inches

#define BLACKLIMIT        50 //Less than this is considered black
#define CROSS_FIELD_POWER 50  //speed of motors to cross field


#define Kp_STRAIGHT       0.1  //Gain for straight controller in power/tick
#define Ki_STRAIGHT       1  //Ki for straight controller in power/tick*dT
#define MIN_DELTA         -10 //Min delta power for controller
#define MAX_DELTA         10  //Max delta for controller


#define FORWARD_SPEED     25
#define INCH2CM           2.54
#define DIST_FROM_WALL    3  //inches from wall
#define Kp                5  //Scale for wall approach controller
#define Ki                1  //Gain for wall approach controller
#define LONG_ENOUGH       10 //Number of cylces of being close enough to trigger stop
#define MIN_POWER         15 //Minimum power the motors need to move slowly

#define BACK_DIST         9 //Inches to back up from wall
#define BACKUP_SPEED      -20 //Speed to back up with

#define NUM_LOOPS         15


/**************************************************************************************
 * Global Variables
 **************************************************************************************/

//Make motor objects
ecrobot::Motor RightWheel (RightWheelPort);
ecrobot::Motor LeftWheel (LeftWheelPort);

//Create a light sensor object using the nxt color sensor (because that's all I have)
ecrobot::NxtColorSensor::eSensorMode sMode = ecrobot::NxtColorSensor::_LIGHTSENSOR_RED;
ecrobot::NxtColorSensor AuxLight(AuxLightPort, sMode);

//Create a normal light sensor object from the one light sensor I do have
ecrobot::LightSensor MainLight(MainLightPort);

//Create as sonar sensor object to see distance
ecrobot::SonarSensor Sonar(SonarPort);

//Robot class to hold position/velocity data
RobotClass myBot(&LeftWheel, &RightWheel);

TaskShare<U8> task_NavState;



/**************************************************************************************
 * Go Straight Method
 **************************************************************************************/
/** @brief   Simple controller to send the motors straight
 *  @details Keeps the motors moving straight
 */

void goStraight(S8 power, bool reset = false)
{
	//static RobotClass::RectData pos;
	S32 Rpow;
	S32 Lpow;
	S32 Rcount;
	S32 Lcount;
	S32 RLerror;
	S32 deltaP;
	static S32 RLerror_sum = 0;
	
	if (reset) 
	{
		RLerror_sum = 0;
	}
	
	Rcount = RightWheel.getCount();
	Lcount = LeftWheel.getCount();
	
	RLerror = Rcount - (Lcount);
	RLerror_sum += RLerror;
	
	deltaP = Kp_STRAIGHT* RLerror + Ki_STRAIGHT * RLerror_sum / 1024;
	
	if (deltaP > MAX_DELTA) {deltaP = MAX_DELTA;}
	if (deltaP < MIN_DELTA) {deltaP = MIN_DELTA;}
	
	if (power > 0)
	{
		Rpow = power - deltaP;
		Lpow = power + deltaP;
	}
	if (power < 0)
	{
		Rpow = power + deltaP;
		Lpow = power - deltaP;
	}
	
	RightWheel.setPWM((S8) Rpow);
	LeftWheel.setPWM((S8) Lpow);
	
// 	Display.cursor(0,DEBUG);
// 	Display.putf("sd\n", "Rpow: ", Rpow,0);
// 	Display.putf("sd\n", "Lpow: ", Lpow,0);
// 	Display.putf("sd\n", "RLerror: ", RLerror,0);
// 	Display.putf("sd\n", "RLerror_sum: ", RLerror_sum,0);
// 	Display.disp();
	
}


/**************************************************************************************
 * Task Nav Constructor
 **************************************************************************************/
/** @brief   Constructor for the navigation task
 *  @details 
 *  
 */


void NavConstructor(void)
{
	
	task_NavState.put(NAV_IDLE);
	
	Display.cursor(0,NAV_LINE);
	Display.putf("s\n", "Nav Ready");
	Display.disp();
	
}



/**************************************************************************************
 * Task Nav Run Method (infinite loop)
 **************************************************************************************/
/** @brief   Run method for the navigation task
 *  @details 
 * 
 */


void NavRun(void)
{
	U32 currentTime;
	S16 curDist;
	//RobotClass::RectData curPos;
	//bool StopLF = false;
	//bool LineDetected = false;
	//S32 LinePos;
	U8 stage = 0;
	bool firstPass = true;
	S8 power;
	S16 error;
	S16 error_sum;
	U8 at_Wall;
	U8 loop_count=0;
	
	ecrobot::Speaker mSpeak;
	
	//Go forever!
	while(true)
	{		
		currentTime = NNxt::getTick();		
		
		switch (task_NavState.get())
		{			
			case NAV_IDLE:
					
				firstPass = true;
				task_NavDone.put(false);
				
				break;
			
			case NAV_TO_SUPPLY:
			
			
				switch (stage)
				{
					//Follow line until the center line is seen
					case 0:
						
						task_LFStart.put(true);
						
						Display.cursor(0,NAV_LINE);
						Display.putf("sd\n", "Color: ", (U8) AuxLight.getBrightness(),0);
						Display.disp();
						
						if((U8) AuxLight.getBrightness() > 100 ) 
						{
							//Go to next stage
							stage = 1;	
							task_LFStart.put(false);
							
						}
					
						break;
					
					//Move past center line
					case 1:
						goStraight(50);
						
						loop_count++;
						
						if (loop_count > NUM_LOOPS)
						{
							stage = 2;
						}
					
						break;
						
					//Follow line supply line
					case 2:
						task_LFStart.put(true);
						
						if((U8) AuxLight.getBrightness() > 100 ) 
						{
							//Go to next stage
							stage = 0;	
							task_LFStart.put(false);
							task_NavState.put(IDLE);
							
						}
						break;
				}
			
				
				
				//Find line - drive along
				//When getting close to center, switch to manual control
				//Resume LF until getting close to wall
				
				break;
			
				
			case NAV_TO_SCORE:
			
			
			
				break;
				
				
			case NAV_APPROACH_WALL:
				
// 				if(firstPass == true)
// 				{
// 					error = 0;
// 					error_sum = 0;
// 					at_Wall = 0;
// 					RightWheel.reset();
// 					LeftWheel.reset();
// 					goStraight(0, true);
// 					firstPass = false;
// 				}
// 				
// 				//Distance from wall in cm
// 				curDist = Sonar.getDistance(); 
// 				
// 				//Calculate errors in cm
// 				error = (curDist - DIST_FROM_WALL * INCH2CM);				
// 				error_sum += error;
// 				
// 				if (error <= 0) {at_Wall++;}
// 				else {at_Wall = 0;}
// 				
// 				power = (S8) (Kp * error + Ki * error_sum / 1024);
// 				
// 				if (power < MIN_POWER) {power = MIN_POWER;}
// 				
// 				goStraight(power);
// 			
// 				if (at_Wall > LONG_ENOUGH)
// 				{
// 					task_NavState.put(NAV_IDLE);
// 					RightWheel.setPWM(0);
// 					LeftWheel.setPWM(0);
// 
// 					mSpeak.playTone(500,50,20);					
// 				}
// 				
// 				Display.cursor(0,NAV_LINE);
// 				Display.putf("sd\n", "curDist: ", curDist,0);
// 				Display.putf("sd\n", "error: ", error,0);
// 				Display.putf("sd\n", "error_sum: ", error_sum,0);
// 				Display.disp();
				
				//Take over when close to wall
				//Drive forward using US
				//Goal is to stop at proper position underneath rings
			
				if(firstPass == true)
				{
					error = 0;
					error_sum = 0;
					at_Wall = 0;
					RightWheel.reset();
					LeftWheel.reset();
					goStraight(0, true);
					firstPass = false;
				}
				
				//Distance from wall in cm
				curDist = Sonar.getDistance(); 
				
				//Calculate errors in cm
				error =  curDist - DIST_FROM_WALL * INCH2CM;				
				
				goStraight(FORWARD_SPEED);
			
				if (error <= 0)
				{
					task_NavState.put(NAV_IDLE);
					RightWheel.setPWM(0);
					LeftWheel.setPWM(0);

					mSpeak.playTone(500,50,20);					
				}
				
				
				break;
				
			case NAV_BACK_UP:
				
				if(firstPass == true)
				{
					error = 0;
					error_sum = 0;
					at_Wall = 0;
					RightWheel.reset();
					LeftWheel.reset();
					goStraight(0, true);
					firstPass = false;
				}
				
				//Distance from wall in cm
				curDist = Sonar.getDistance(); 
				
				//Calculate errors in cm
				error =  BACK_DIST * INCH2CM - curDist;				
				
				goStraight(BACKUP_SPEED);
			
				if (error <= 0)
				{
					task_NavState.put(NAV_IDLE);
					RightWheel.setPWM(0);
					LeftWheel.setPWM(0);

					mSpeak.playTone(500,50,20);					
				}
				
				
				//Carefully back up away from rings to certain distance
				//Using US 
				
				break;
				
			case NAV_TURN_AROUND:
			
				//Turn in place, (using encoders?), until line is acquired
				
				break;
				
		}//End switch
		
		
		//Let other tasks run
		sleep_from_for(currentTime, 20);
		
	}//End while
}
	



/**************************************************************************************
 * Task Nav
 **************************************************************************************/
/** @brief   Navigation task
 *  @details Waits until the start variable is set. Then runs the 
*	     constructor and the run method. The run method will 
*	     *never* exit. And if it somehow does the task will just
*	      exit.  
 */


extern "C"
{


TASK(NavTask)
{
	//Wait until permission to start is given

	while(task_NavStart.get() != true) 
	{	
		//Let other tasks run
		NNxt::sleep(500);
	}
	
	//Runs once
	NavConstructor();

	//This loops forever
	NavRun();
	
	//shouldn't ever get here
	TerminateTask();
}

}
