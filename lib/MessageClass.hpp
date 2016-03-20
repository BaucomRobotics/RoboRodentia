//*************************************************************************************
/** @file    MessageClass.hpp
 *  @brief   A message class for the RS485 communication port
 *  @details
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
#ifndef _MSGHEADER_H_
#define _MSGHEADER_H_

/**************************************************************************************
 * Message class
 **************************************************************************************/
/** @brief  Defines a message class for use in serial communication.
 *  @details  An instance of this message object can be used to hold message
 * 			 data and automatically build a header to send in front of data
 * 			 so that the receiver knows what to do with it.
 * 			<b> This class still has some bugs!</b> The encoding and decoding 
 * 			 methods need some work still, but one should be able to get away 
 * 			 for the most part with clever use of the @c SendMsgSimple and 
 * 			 @c GetMsgSimple methods.
 */

 
class MessageClass
{

public:

	//Number of bytes needed for the header
	static const U8 HEADER_LENGTH = 3;
	
	//Max message length (an abstract number I made up to keep messages short)
	static const U8 MAX_MSG_LEN = 15;
	
	//Make sure data types are communicated correctly
	enum comDatatype 
	{
		typeUnspec = 0, /**<Unspecified type, may be a error*/
		typeU32 = 	 1, /**<Unsigned 32bit arithmetical type*/
		typeS32 = 	 2, /**<Signed 32bit arithmetical type*/
		typeBool = 	 3, /**<Boolean type*/
		typeFloat =	 4, /**<32bit floating-point number*/
		typeU8 = 	 5, /**<8bit signed char*/
		typeString = 6  /**<C-String type*/
		// free 7 - 255
	};

	
	//This enum will be used so that each message sent will be interpreted
	//by the recieve properly
	enum comDataID
	{
		//General Stuff
		idNoMsg = 0,    /**<Default message*/
		idWakeMsg = 1, /**<Let the other brick know the sender is alive!*/
		idAckMsg   = 2, /**<General acknowlegement of received message*/
		idInitDone = 3,  /**<Initialization has been completed*/
		
		//Stuff Master needs to tell slave
		idPrepForGrabRings = 10,
		idGrabRings = 11,
		idPrepForPlacement = 12,
		idPlaceRings = 13,
		
		//Stuff Slave needs to tell master
		idReadytoGrab = 50,
		idGrabbedRings = 51,
		idReadytoPlace = 52,
		idPlacedRings = 53
		
		//Free values: 4-9, 14-49, 54-255
	};
	
	//Default Constructor
	MessageClass(void);	
	
	//Constructor used when message length is known
	MessageClass(U8 length);
	
	//decodes message that was received
	U8* DecodeMsg(comDatatype& dType, comDataID& dID, U8& len);
	
	//Builds message to be sent
	U8 BuildMsg(U8* data2send, comDatatype dType, comDataID dID, U8 len);
	
	//Send this message
	U32 SendMsg(void);
	
	//Receive a message - overwrites all data currently in the class
	U32 GetMsg(void);
	
	//Check if the message is empty or not
	bool isEmpty(void);
	
	//Clear all data, but don't destruct the message
	void clearData(void);
	
	//Simple message sending
	void SendMsgSimple(comDataID dID);
	
	//Simple Message receiving
	comDataID GetMsgSimple(void);
	
	//Check if Simple Message is empty
	bool isEmptySimple(void);
	
	//Clear Simple Message Data
	inline void clearDataSimple(void);
	
	//Return Simple Message Data
	inline comDataID GetMsgDataSimple(void);
	
protected:

	//Message header data
	struct HeaderData
	{
		U8 datatype;   /**<Type of data the message contains*/
		U8 length;     /**<Length of the data in the message*/
		U8 dataID;     /**<Extra info so the recieve know what the data is*/
	} mHeaderData;


	/*This is a header that will be stuck on each set of data that is put in front of each message*/
	union  RS485_Header
	{
		HeaderData HD;
		U8 asbyte[sizeof(HD)];
		
	} mHeader;
	
	//Pointer to all the message data stored in this object
	U8* MsgData;
	
	//Length of the current message
	U8  MsgLen;	
	
	//Max size of the message buffer
	U8  BufferSize;
	
	//Storage for ID returned from simple message get function
	comDataID SimpleID;
	
	
};

//Fixes weird linker issues....
#include "MessageClass.cpp"

#endif
