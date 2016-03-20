//*************************************************************************************
/** @file    RobotClass.hpp
 *  @brief   A header class for a dual-drive robot
 *  @details Currently this class only applies to robots which have two 
 * 			 independent drive wheels and a caster. All the kinematics
 * 			 follows this model.
 *
 *  Revised:    
 *	  \li 03-09-2015 ARB Original file
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

// This define prevents this .h file from being included more than once in a .cpp file
#ifndef _RBTHEADER_H_
#define _RBTHEADER_H_


#include <Motor.h>

/**************************************************************************************
 * Robot Class Header
 **************************************************************************************/
/** @brief  Defines a robot object to hold position and velocity data
 *  @details This class will hold position and velocity data of a robot	
 *			 (both liniear and angular). This helps ensure that the robot knows
 *			 where it is at all times.
 */

 
class RobotClass
{

protected:

	//Pointers to the motor to use for reading encoders
	ecrobot::Motor* p_Rmotor;
	ecrobot::Motor* p_Lmotor;
	
	//Holds data in a local, motor-specific coordinate frame
	struct LocalData
	{
		S32 vel;
		S32 pos_new;
		S32 pos_old;
	} lData, rData;
	
	//Variables to hold timing information
	U32 OldTime;
	U32 NewTime;
	U32 deltaTime;
	
	//Constants that hold physical proerties of the robot
	//to be used in kinematic calculations
	static const U8 WHEEL_RAD = 0.75; //Inches
	
	static const U8 WHEEL_BASE = 4.4; //Inches

public:

	
	//Holds data in a rectangular coordinate system (inches and radians)
	struct RectData
	{
		S32 x;
		S32 y;
		S32 xdot;
		S32 ydot;
		S32 theta;
		S32 thetadot;
	} BotData, BotDataOld;
	
	//Consructor
	RobotClass(ecrobot::Motor* p_LeftMotor, ecrobot::Motor* p_RightMotor);
	
	//Get a data structure with all the information in 
	//a rectangular coordinate system
	inline RectData GetInfo(void);
	
	//Update the information in this object.
	//Needs to be called often as it is not it's own task.
	void Update(void);
	
	//Reset all values to 0
	void Reset(void);
	
};


//Fixes weird linker issues....
#include "RobotClass.cpp"

#endif
