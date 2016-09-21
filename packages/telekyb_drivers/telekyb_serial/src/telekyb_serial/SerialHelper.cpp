/*
 * SerialHelper.cpp
 *
 *  Created on: Nov 24, 2011
 *      Author: mriedel
 */

#include <telekyb_serial/SerialHelper.hpp>

#include <string.h>

#include <ros/console.h>

namespace TELEKYB_NAMESPACE {

int SerialHelper::appendCRC ( char* buffer, unsigned int bufferPosition )
{
	unsigned int tmpCRC = 0;
	for ( unsigned int i = 0; i < bufferPosition; i++ )
	{
		tmpCRC += buffer[i];
	}
	tmpCRC %= 4096;
	buffer[bufferPosition] = '=' + tmpCRC / 64;
	buffer[bufferPosition+1] = '=' + tmpCRC % 64;
	return 2;
}

//** Warning Prefix must be encoded so that it does not create binary 11 at the Beginning or a direct \r
int SerialHelper::encodeData ( char* buffer, // buffer to write to must be at least prefixLength + 3 (crc1,2 and \r) + messageLength*4/3
		const char* prefix, unsigned int prefixLength,
		const char* message, unsigned int messageLength )
{
	unsigned int pt = 0;
	unsigned char a,b,c;
	unsigned char ptr = 0;

//	buff[pt++] = '#'; // Startchar
//	buff[pt++] = modul; // Address b, c, d
//	buff[pt++] = cmd; // Command

	// set Prefix
	memcpy(buffer, prefix, prefixLength);
	pt += prefixLength;

	while ( messageLength ){
		if ( messageLength ){
			a = message[ptr++];
			messageLength--;
		}
		else a = 0;
		if ( messageLength ){
			b = message[ptr++];
			messageLength--;
		}
		else b = 0;
		if ( messageLength ){
			c = message[ptr++];
			messageLength--;
		}
		else{
			c = 0;
		}
		buffer[pt++] = '=' + ( a >> 2 );

		buffer[pt++] = '=' + ( ( ( a & 0x03 ) << 4 ) | ( ( b & 0xf0 ) >> 4 ) );
		buffer[pt++] = '=' + ( ( ( b & 0x0f ) << 2 ) | ( ( c & 0xc0 ) >> 6 ) );
		buffer[pt++] = '=' + ( c & 0x3f );
	}

	pt += appendCRC ( buffer, pt );
	// Terminal
	buffer[pt++] = '\r';

	return pt;
}

//** This requires the TargetLength! You have to know what you want to decode!!!
int SerialHelper::decodeData(char* buffer, unsigned int targetLength, const char* message)
{
	unsigned char a,b,c,d;
	unsigned char ptr = 0;
	unsigned char x,y,z;
	// ignore Prefix
	int ptrIn = 0;

	while (targetLength) {
		a = message[ptrIn++] - '=';
		b = message[ptrIn++] - '=';
		c = message[ptrIn++] - '=';
		d = message[ptrIn++] - '=';
		//if (ptrIn > max - 2) break; // nicht mehr Daten verarbeiten, als empfangen wurden

		x = (a << 2) | (b >> 4);
		y = ((b & 0x0f) << 4) | (c >> 2);
		z = ((c & 0x03) << 6) | d;

		if (targetLength--) buffer[ptr++] = x; else break;
		if (targetLength--) buffer[ptr++] = y; else break;
		if (targetLength--) buffer[ptr++] = z; else break;
	}

	// number of decoded bytes
	return ptr;
}

// message must be of structure [msg][crc1][crc2]'\r'
bool SerialHelper::checkCRC(const char* message, unsigned int messageLength)
{
	if (messageLength < 4) {
		ROS_ERROR("CRC Message too short!");
		return false;
	}

	if (message[messageLength-1] != '\r' && message[messageLength-1] != '\n') {
		ROS_WARN("CheckCRC without \\r or \\n terminated message!");
	}

	int tmpCRC = 0;
	for ( unsigned int i = 0; i < messageLength-3; i++ )
	{
		tmpCRC += message[i];
	}
	tmpCRC %= 4096;
	//ROS_INFO("CRC Should be 1: %d 2: %d", (int)('=' + tmpCRC / 64),(int)('=' + tmpCRC % 64));
	return (message[messageLength-3] == '=' + tmpCRC / 64) && (message[messageLength-2] == '=' + tmpCRC % 64);
}


} /* namespace telekyb */
