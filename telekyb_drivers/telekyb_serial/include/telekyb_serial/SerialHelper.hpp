/*
 * SerialHelper.hpp
 *
 *  Created on: Nov 24, 2011
 *      Author: mriedel
 */

#ifndef SERIALHELPER_HPP_
#define SERIALHELPER_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

namespace TELEKYB_NAMESPACE {

class SerialHelper {
public:
	static int appendCRC ( char* buffer, unsigned int bufferPosition );
	//** Warning Prefix must be encoded so that it does not create 11 at the Beginning or a direct \r
	static int encodeData ( char* buffer, // buffer to write to must be at least prefixLength + 3 (crc1,2 and \r) + messageLength*4/3
			const char* prefix, unsigned int prefixLength,
			const char* message, unsigned int messageLength );

	static int decodeData(char* buffer, unsigned int targetLength, const char* message);

	static bool checkCRC (const char* message, unsigned int messageLength);
};

} /* namespace telekyb */
#endif /* SERIALHELPER_HPP_ */
