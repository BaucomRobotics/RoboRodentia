//*************************************************************************************
/** @file    RobotClass.cpp
 *  @brief   Cpp file for robot class
 *  @details Perfroms positonal caluclation and provied formatted data
 * 			 structures useful for dead reckoning systems.
 *
 *  Revised:    
 *	  \li 03-10-2015 ARB Original file
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

//Need header file for the class
#include "RobotClass.hpp"

//Needed for timer function
#include "../../nxtOSEK/NXtpandedLib/src/NNxt.hpp"

//Need for the sin and cos functions
#include "../../yagarto-old/arm-none-eabi/include/c++/4.6.2/cmath"


/**************************************************************************************
 * Constructor
 **************************************************************************************/
/** @brief  Class constructor
 *  @details Initialized class data
 * 	@param   p_LeftMotor Pointer to the motor object controlling the motor on the left side of the robot
 * 	@param   p_RightMotor Pointer to the motor object controlling the motor on the right side of the robot
 */

	RobotClass::RobotClass(ecrobot::Motor* p_LeftMotor, ecrobot::Motor* p_RightMotor)
	{
		//Initialize Motors
		p_Lmotor = p_LeftMotor;
		p_Rmotor = p_RightMotor;
		
		//Clear Rect data
		BotData.x = 0;
		BotData.y = 0;
		BotData.xdot = 0;
		BotData.ydot = 0;
		BotData.theta = 0;
		BotData.thetadot = 0;
		
		//Clear old  Rect data
		BotDataOld = BotData;
	
		//Clear local data (left)
		lData.vel = 0;
		lData.pos_old = 0;
		lData.pos_new = 0;
		
		//Clear local data (right)
		rData = lData;
		
		NewTime = NNxt::getTick();
		OldTime = NewTime;
	}
	
	
/**************************************************************************************
* Get Info
**************************************************************************************/
/** @brief  Gets kinematic info and returns it to the caller
*  	@details Returns a data structure which has all the information
*			 about the robot's position and velocity in rectuangular coordinates 
*   @return   RectData structure object
*/
	
	inline RobotClass::RectData RobotClass::GetInfo(void)
	{
		return BotData;
	}
	
/**************************************************************************************
* Reset Data
**************************************************************************************/
/** @brief  Resets Robot object data
*  	@details Stets all values to 0 to reset the object
*/
	
	void RobotClass::Reset (void)
	{
		//Clear Rect data
		BotData.x = 0;
		BotData.y = 0;
		BotData.xdot = 0;
		BotData.ydot = 0;
		BotData.theta = 0;
		BotData.thetadot = 0;
		
		//Clear old  Rect data
		BotDataOld = BotData;
	
		//Clear local data (left)
		lData.vel = 0;
		lData.pos_old = 0;
		lData.pos_new = 0;
		
		//Clear local data (right)
		rData = lData;
		
		NewTime = NNxt::getTick();
		OldTime = NewTime;
	}	
	
	
	
/**************************************************************************************
*  Update Info
**************************************************************************************/
/** @brief  Update internal data info. This needs to be called often to be accurate!
* 	@details This method performs all the calcuations for this object by reading the 
* 			 encoders for each motor, checking the time that has passed, updating the velocity,
* 			 and then performing Euler integration to find the positions.
*/
	
	void RobotClass::Update(void)
	{
		//Calculate time passed
		NewTime = NNxt::getTick();
		deltaTime = NewTime - OldTime;
		
		//Get encoder counts
		rData.pos_new = p_Rmotor -> getCount();
		lData.pos_new = p_Lmotor -> getCount();
		
		//Find wheel velocity
		rData.vel = (rData.pos_new - rData.pos_old) / deltaTime;
		lData.vel = (lData.pos_new - lData.pos_old) / deltaTime;
		
		//Caluclate rates of change
		BotData.xdot = WHEEL_RAD/2 * (rData.vel + lData.vel) * cos(BotData.theta);
		BotData.ydot = WHEEL_RAD/2 * (rData.vel + lData.vel) * sin(BotData.theta);
		BotData.thetadot = WHEEL_RAD/WHEEL_BASE * (rData.vel - lData.vel);
		
		//Update current positions
		BotData.x = BotDataOld.xdot + BotData.xdot * deltaTime;
		BotData.y = BotDataOld.ydot + BotData.ydot * deltaTime;
		BotData.theta = BotDataOld.thetadot + BotData.thetadot * deltaTime;
		
		//Update old values
		OldTime = NewTime;
		BotDataOld = BotData;
		
	}
	
	

 