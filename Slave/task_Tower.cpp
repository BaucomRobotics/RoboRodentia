//*************************************************************************************
/** @file    task_Tower.cpp
 *  @brief   A task that manages the tower motor
 *  @details This task intializes the tower by lifting it
 * 			 it then exits during its current iteration.
 * 			 Later it might actually do some control looping if I
 * 			 decide to do some fun things with it.
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
#include "../../yagarto-old/arm-none-eabi/include/c++/4.6.2/cmath"

/**************************************************************************************
 * Include ECROBOT Files
 **************************************************************************************/
#include <Lcd.h>
#include <Motor.h>
#include <Speaker.h>

/**************************************************************************************
 * Include Personally Written Files
 **************************************************************************************/
#include "../lib/taskshare.hpp"
#include "shares.hpp"

/**************************************************************************************
 * Include NXTexpanded Lib Files
 **************************************************************************************/
#include "../../nxtOSEK/NXtpandedLib/src/NNxt.hpp"


/**************************************************************************************
 * Constants
 **************************************************************************************/
#define INITIAL_SPEED 75
#define SLOWER_SPEED  25
#define OFF 0
#define FINAL_POS 105
#define EASIER_PART 45



/**************************************************************************************
 * Global Variables
 **************************************************************************************/
ecrobot::Motor Tower (TowerPort);
TaskShare<bool> TowerArrived;


/**************************************************************************************
 * Task Tower Constructor
 **************************************************************************************/
/** @brief   Constructor for the Tower task
 *  @details Lifts the tower until stall (reaches limit)
 * 			 Then exits.
 *  
 */

void TowerConstructor(void)
{
	//Set up the tower
	
	//Stop motor and reset position
	Tower.reset();
	
	//Set tower mode to brake after motion
	Tower.setBrake(true);
	
	//Start tower moving up
	Tower.setPWM(INITIAL_SPEED);	
	
	//Get past the initial resistance
	while (Tower.getCount() < EASIER_PART)
	{
		//Let other tasks work while waiting
		NNxt::sleep(50);
	}
	
	//Ease up on the power
	Tower.setPWM(SLOWER_SPEED);	
	
	//Get past the initial resistance
	while (Tower.getCount() < FINAL_POS)
	{
		//Let other tasks work while waiting
		NNxt::sleep(50);
	}
	
	//Turn off motor
	Tower.setPWM(OFF);	
	
	
	//Set tower arrived
	TowerArrived.put(true);
	
	//Tower is now ready for operation.	
	//Notify user
	Display.cursor(0,TOWER_LINE);
	Display.putf("s\n", "Tower Ready");
	Display.disp();
}





/**************************************************************************************
 * Task Tower
 **************************************************************************************/
/** @brief   Tower task
 *  @details Waits until the start variable is set. Then runs the 
 *			 constructor and then exits.
 * 			 Could add run method later if things get fun :P
 */


extern "C"
{


TASK(TowerTask)
{
	//Wait until permission to start is given

	while(task_TowerStart.get() != true) 
	{	
		//Let other tasks run
		NNxt::sleep(100);
	}
	
	//Runs once
	TowerConstructor();
	
	//shouldn't ever get here
	TerminateTask();
}

}
