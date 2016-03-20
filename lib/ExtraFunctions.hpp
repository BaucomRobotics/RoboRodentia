//*************************************************************************************
/** @file    ExtraFunctions.hpp
 *  @brief   Some extra functions that can be used for the RoboRodenatia program
 *  @details Included functions:
 * 			 sleep_from_for()
 *
 *  Revised:    
 *	  \li 02-28-2015 ARB Original file
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
#ifndef _EXTRAFUNS_H_
#define _EXTRAFUNS_H_


/**************************************************************************************
 * Include NXTexpanded Lib Files
 **************************************************************************************/
#include "../../nxtOSEK/NXtpandedLib/src/NNxt.hpp"




/**************************************************************************************
 * sleep_from_for
 **************************************************************************************/
/** @brief   Lets a task sleep for an exact amount of time.
 *  @details This funciton takes a time input and lets the task
 * 			 sleep for the specified amount of time since the time provided.
 * 			 This lets tasks have a very high level of timing accuracy.
 *  @param   start_time When to start measuring the time from, in ms. Often this is useful
 * 	         to measure right at the start of a task's exectution and then give to this
 * 			 function so the task has exact timing.
 *  @param   sleep_time The time, in ms, to sleep the task for.
 */

inline void sleep_from_for(U32 start_time, U32 sleep_time)
{
	
	NNxt::sleep(sleep_time - (NNxt::getTick() - start_time));
	
}




#endif
