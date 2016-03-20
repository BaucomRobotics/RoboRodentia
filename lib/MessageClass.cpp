//*************************************************************************************
/** @file    MessageClass.cpp
 *  @brief   A class which handles message-type objects
 *  @details The class allows for the creation of message objects
 * 			 and can package them to be sent via RS485 and decoded
 * 			 on the other end.
 * 			 <b>There are bugs with this code!</b> Be careful when
 * 			 using the encode and decode functions as they contain
 * 			 intermittent and troublesome bugs. However, for simple
 * 			 messages the @c SendMsgSimple and @c GetMsgSimple methods
 * 			 have been tested and found to be reliable.
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

#include "../../yagarto-old/arm-none-eabi/include/c++/4.6.2/cstring"

#include "MessageClass.hpp"

#include <Rs485.h>

/*Create a RS485 object for communications 
 *Doesn't need port because port 4 is the only one that supports it
 */
ecrobot::Rs485 MsgComm;


/**************************************************************************************
 * MessageClass Default Constructor
 **************************************************************************************/
/** @brief  Default Class constructor
 *  @details Allocates the maximum memory for a message. Inefficient, but easy
 */

MessageClass::MessageClass(void)
{
	MsgData = new U8[MAX_MSG_LEN];
	BufferSize = MAX_MSG_LEN;
}

/**************************************************************************************
 * MessageClass Constructor
 **************************************************************************************/
/** @brief   Class constructor
 *  @details If the length of the message is known, this will allocate
 * 			 that amount of memory. Note: it currently does not check to make
 * 			 sure the max is less than the arbiraty max value I chose to set.
 */

MessageClass::MessageClass(U8 length)
{
	MsgData = new U8[length];
	BufferSize = length;
}


/**************************************************************************************
 * Build Message method
 **************************************************************************************/
/** @brief   Builds the message to be sent
 *  @details Gets info about the message and header. Then it creates the header
 * 	     attaches it to the message, and returns it. <b>The method might contain
 * 		 intermittent and difficult to find bugs! You have been warned </b>
 * 
 *  @param   data  A pointer to the data to send.
 *  @param   dType The type of data, specificed by the @c comHeaderDatatype enum
 *  @param   dID   The data identifier, specificed by the @c commHeaderDataID enum
 *  @param   len   The length of the data to be sent. Anything longer than
 * 			       MAX_MSG_LEN - HEADER_LENGTH will be truncated.
 * 
 *  @return  Length of data and header
 *  
 */

U8 MessageClass::BuildMsg( U8* data2send, comDatatype dType, comDataID dID, U8 len)
{
   
	MsgLen = len;
	
	//Build header data
    mHeaderData.datatype = dType;
    mHeaderData.length = len;
    mHeaderData.dataID = dID;
	
	//Build header
	mHeader.HD = mHeaderData;
    
	//Clear old memory
	memset(MsgData, 0 , BufferSize);
	
	//Put header at start of packet
	for (U8 n=0; n<HEADER_LENGTH; n++)
	{
		MsgData[n] = mHeader.asbyte[n];
	}
	
	//Add the data to the packet
	for (U8 i=0; i<len; i++)
	{
		MsgData[(HEADER_LENGTH+i)] = data2send[i];
	}
    
    //Return length of data and header
    return len + HEADER_LENGTH;
}




/**************************************************************************************
 * Decode Message method
 **************************************************************************************/
/** @brief   Decodes the received message currently in MsgData
 *  @details Gets the information stored in the message header and passes
 * 		     it back to the calling function by reference. Then it clears the
 * 			 @c MsgData array and fills it with the received data. Make sure to
 *			 delete what is pointed to by @c data after you use it if space is an issue.
 *			 <b>The method might contain intermittent and difficult to find bugs! You have been warned </b>
 * 
 * @param    data Pointer to the start of the received message data
 * @param    dType The type of data received
 * @param    dID   The ID number of the data received
 * @param    len   The length of the message received
 * 
 * @return   Pointer to the message data
 */

U8* MessageClass::DecodeMsg(comDatatype& dType, comDataID& dID, U8& len)
{
    
	//Get header info
	for (U8 n=0; n<HEADER_LENGTH; n++)
	{
		mHeader.asbyte[n] = MsgData[n];
	}
	
	mHeaderData = mHeader.HD;
	
	//Decode header	
	dType = static_cast<comDatatype> (mHeaderData.datatype);
	dID =   static_cast<comDataID> (mHeaderData.dataID);
	len = mHeaderData.length;
	
	MsgLen = len;
	
	U8* p_data2get = new U8[len];
	
	//Add the data to the packet
	for (U8 i=0; i<len; i++)
	{
		p_data2get[i] = MsgData[(HEADER_LENGTH+i)];
	}	
	
	return p_data2get;
	
}



/**************************************************************************************
 * Get Message from comm port
 **************************************************************************************/
/** @brief  Receive a message from the comm port
 *  @details Use this with the decode message funciton to decode
 * 			 the header and data packaged in the message
 */
 
 U32 MessageClass::GetMsg(void)
 {
	return  MsgComm.receive(MsgData, 0, MsgLen);
 }
 
 

 
 /**************************************************************************************
 * Send Message over comm port
 **************************************************************************************/
/** @brief   Send a message over the comm port
 *  @details Use this with the build message function to send
 * 			 a messeage with a header containing extra information
 */
 
 U32 MessageClass::SendMsg(void)
 {
	return  MsgComm.send(MsgData, 0, MsgLen+HEADER_LENGTH);
 }
 
 
  /**************************************************************************************
 * is Empty
 **************************************************************************************/
/** @brief   Check if the message is empty
 *  @details Check if there is currently any data in the message object.
 * 			 Useful for deciding if one should try to extract data from this object.
 */
 
bool MessageClass::isEmpty(void)
 {
	
	bool empty = true;
	
	if (MsgLen > 0)
	{
		empty = false;
	}
	 
	return  empty;
 }
 
 
  /**************************************************************************************
 * Clear data
 **************************************************************************************/
/** @brief   Remove all data in the object
 *  @details Overwrites all data in the data buffer and sets the length of the message o 0
 */
 
void MessageClass::clearData(void)
 {
	memset(MsgData, 0, MAX_MSG_LEN);
	MsgLen = 0;
 }
 
 
 
 
   /**************************************************************************************
 * SendMsgSimple
 **************************************************************************************/
/** @brief   Sends a simple message
 *  @details Send a single byte which is specified by the data ID byte specified in
 * 			 the public enum @c comDataID. Use this to coordinate quick, simple messages
 * 			 between master and slave.
 */
 
void MessageClass::SendMsgSimple(comDataID dID)
 {
	 U8 msg = (U8) dID;
	 
	 MsgComm.send(&msg, 0, 1);
 }
 
 
 
   /**************************************************************************************
 * GetMsgSimple 
 **************************************************************************************/
/** @brief   Gets a simple message
 *  @details Gets a single byte message and returns the message ID in the form of 
 * 		     an enum specified by @c comDataID. If there is no message to be received,
 * 			 a @c idNoMsg flag is returned.
 */
 
MessageClass::comDataID MessageClass::GetMsgSimple(void)
 {
	U8 msg;
	comDataID dID;
	
	if(MsgComm.receive(&msg, 0, 1))
	{
		dID = static_cast<comDataID> (msg);
	}
	else
	{
		dID = idNoMsg;
	}	
	
	SimpleID = dID;
	
	return dID;
	
 }
 
 
   /**************************************************************************************
 * is Empty Simple
 **************************************************************************************/
/** @brief   Check if the simple message is empty
 *  @details Check if there is currently any data in the message object.
 * 			 Useful for deciding if one should try to extract data from this object.
 */
 
bool MessageClass::isEmptySimple(void)
 {
	
	bool empty = true;
	
	if (SimpleID != idNoMsg)
	{
		empty = false;
	}
	 
	return  empty;
 }
 
 
  /**************************************************************************************
 * Clear data Simple
 **************************************************************************************/
/** @brief   Remove all data in the object
 *  @details Overwrites msg ID
 */
 
inline void MessageClass::clearDataSimple(void)
 {
	SimpleID = idNoMsg;
 }
 
 
  /**************************************************************************************
 * Get Msg Data Simple
 **************************************************************************************/
/** @brief   Gets whatever data is in the simple message
 *  @details 
 * 
 *  @return  Msg ID data 
 */
 
inline MessageClass::comDataID MessageClass::GetMsgDataSimple(void)
 {
	return SimpleID;
 }