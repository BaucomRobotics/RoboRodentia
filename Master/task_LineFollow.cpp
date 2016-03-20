//*************************************************************************************
/** @file    task_LineFollow.cpp
 *  @brief   A task which follows the edge of a line
 *  @details This task will follow a line quickly and ensure
 * 			 ensure that the robot stays on track. It cannot handle sharp turns.
 *
 *  Revised:
 *     \li 03-28-2015 ARB Original file
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
#include <LightSensor.h>

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

#define STD_POWER 50    //Power the motors should spin at if going straight along edge
//#define EDGE_VAL 600    //Brightness reading of the edge the robot needs to follow
#define SCALE    1     //Scale the feedback 


/**************************************************************************************
 * Global Variables
 **************************************************************************************/

//Make motor objects
ecrobot::Motor RightWheel (RightWheelPort);
ecrobot::Motor LeftWheel (LeftWheelPort);

//Create a normal light sensor object from the one light sensor I do have
ecrobot::LightSensor MainLight(MainLightPort);

TaskShare<bool> task_LFStart;

TaskShare<S16> black_limit;

S16 EDGE_VAL;



void Rotate(S8 angle, S8 dir)
{
	
	//dir = +1 or -1
	
	S32 Rcount = 0;;
	
	RightWheel.reset();
	LeftWheel.reset();
	
	RightWheel.setPWM(dir * 30);
	LeftWheel.setPWM(dir * (-30));
	
	while(std::abs(Rcount) < angle)
	{
		Rcount = RightWheel.getCount();
	}
	
	RightWheel.setPWM(0);
	LeftWheel.setPWM(0);
	
}



/**************************************************************************************
 * Task Line Follow Constructor
 **************************************************************************************/
/** @brief   Constructor for the line follow task
 *  @details 
 *  
 */


void LFConstructor(void)
{
	S16 b1,b2;
	
	task_LFStart.put(false);	
	
	Rotate(45,-1);
	
	NNxt::sleep(100);
	b1 = MainLight.get();
	
	Rotate(90,1);
	NNxt::sleep(100);
	b2 = MainLight.get();
	
	EDGE_VAL = (b1+b2)/2;
	black_limit.put(EDGE_VAL);
	
	Rotate(45,-1);
	
}



/**************************************************************************************
 * Task LF Run Method (infinite loop)
 **************************************************************************************/
/** @brief   Run method for the line follow task
 *  @details 
 * 
 */


void LFRun(void)
{
	U32 currentTime;
	enum state_t {IDLE, FOLLOWING} state = IDLE;
	S16 brightness;
	S16 Rpow;
	S16 Lpow;
	
	//Go forever!
	while(true)
	{		
		currentTime = NNxt::getTick();		
		
		switch (state)
		{
			case IDLE:
			
				if(task_LFStart.get()==true)
				{
					state = FOLLOWING;
				}
				
				break;
				
			case FOLLOWING:
				
				brightness = MainLight.get();
				
				Display.cursor(0,DEBUG);
				Display.putf("sd\n", "light: ", brightness,0);
				Display.disp();
				
				//This will follow the right edge of the line - to change sides, switch the signs
				Rpow = STD_POWER + (EDGE_VAL - brightness)/SCALE;
				Lpow = STD_POWER - (EDGE_VAL - brightness)/SCALE;
				
				RightWheel.setPWM((S8)Rpow);
				LeftWheel.setPWM((S8)Lpow);
				
				if(task_LFStart.get()==false)
				{
					state = IDLE;
				}
				
				
				break;		
				
		}//End switch
		
		
		//Let other tasks run
		sleep_from_for(currentTime, 10);
		
	}//End while
}
	



/**************************************************************************************
 * Task LF
 **************************************************************************************/
/** @brief   Line follow task
 *  @details Waits until the start variable is set. Then runs the 
*	     constructor and the run method. The run method will 
*	     *never* exit. And if it somehow does the task will just
*	      exit.  
 */


extern "C"
{


TASK(LFTask)
{
	//Wait until permission to start is given

	while(task_LFStart.get() != true) 
	{	
		//Let other tasks run
		NNxt::sleep(500);
	}
	
	//Runs once
	LFConstructor();

	//This loops forever
	LFRun();
	
	//shouldn't ever get here
	TerminateTask();
}

}
