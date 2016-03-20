//*************************************************************************************
/** @file    task_MasterInit.cpp
 *  @brief   Initialization of Master nxt
 *  @details This is a task which handles initialization of the master
 * 			 nxt. It will initialze the code and then the task will exit.
 *
 *  Revised:
 *     \li 02-17-2015 ARB Original file
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
 * Include Kernel Files
 **************************************************************************************/

extern "C" {
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
}

/**************************************************************************************
 * Include ECROBOT Files
 **************************************************************************************/
#include <Lcd.h>
#include <NXTColorSensor.h>


/**************************************************************************************
 * Include NXTexpanded Lib Files
 **************************************************************************************/
#include "../../nxtOSEK/NXtpandedLib/src/NNxt.hpp"


/**************************************************************************************
 * Include Personally Written Files
 **************************************************************************************/
#include "../lib/taskshare.hpp"
#include "shares.hpp"



/**************************************************************************************
 * PORT DEFINITIONS
 **************************************************************************************/
//These were declared as extern variables in shares.hpp.
//They are simply given definitions here

//Master Sensors
const ePortS AuxLightPort = PORT_1;
const ePortS MainLightPort = PORT_2;
const ePortS SonarPort = PORT_3;
const ePortS CommPort = PORT_4;

//Master Motors
const ePortM RightWheelPort = PORT_B;
const ePortM LeftWheelPort = PORT_C;


/**************************************************************************************
 * Global Delcarations
 **************************************************************************************/

//Declare our counter and resource specified in OIL file
DeclareCounter(SysTimerCnt);

//Create Lcd Resource (This is a definition from the share.h file, the display is actual extern)
ecrobot::Lcd Display;

//Define all of our extern shared variables that handle task startup.
TaskShare<bool> task_MasterMindStart;
TaskShare<bool> task_CommStart;
TaskShare<bool> task_NavStart;


ecrobot::NxtColorSensor AuxLight(AuxLightPort);

/**************************************************************************************
 * Kernal setup
 **************************************************************************************/

extern "C" {

	
//Start up hook - this runs at startup, before any tasks begin.
void StartupHook(void)
{
	//Don't let other tasks start until init is done
	task_MasterMindStart.ISR_put(false);
	task_CommStart.ISR_put(false);
	task_NavStart.ISR_put(false);
}		
	
	
// nxtOSEK hook to be invoked from an ISR in category 2
void user_1ms_isr_type2(void)
{
	SleeperMonitor(); // must be called here to use sleep function.
	StatusType ercd;

	ercd = SignalCounter(SysTimerCnt); /* Increment OSEK Alarm Counter */
	if (ercd != E_OK) 
	{
	    ShutdownOS(ercd);
	}
	
	AuxLight.processBackground();
}




/**************************************************************************************
 * Initialization Task
 **************************************************************************************/
/** @brief   Initializes everything
 *  @details This task doesn't do much by itself, but this file runs the startup hook and global variable setup.
 *           That way everything else is ready to go after this starts the MasterMind and exits.
 * 			 Probably isn't overly necesary, but it is nice to have it all in one place
 *  
 */

TASK(MasterInit)
{
		
	Display.clear();
	Display.putf("s\n", "Init Complete" );
	Display.disp();
	
	task_MasterMindStart.put(true);
	
	TerminateTask();
}



}

