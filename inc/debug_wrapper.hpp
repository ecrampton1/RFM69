
#ifndef _DEBUG_WRAPPER_HPP
#define _DEBUG_WRAPPER_HPP

#include <stdio.h>


enum class Base : int
{
	BASE_BIN = 2,
	BASE_DEC = 10,
	BASE_HEX = 16

};

class DebugWrapper
{
public:
	static void init() {
		//Do nothing
	}

	template<class T>
	static void send( const T data, const Base base=Base::BASE_DEC )
	{
		//Do nothing
	}

	static void send( float data, int precision=2)
	{
		//TODO implement (look into fixed point) ??

	}

	static void send(uint8_t* const data,const  int numOfBytes)
	{
		//Do nothing
	}

	static void send(const char* data)
	{
		//Do nothing
	}

	static void sendLine(const char* data=nullptr)
	{
		//Do nothing
	}

	static const bool readByte(uint8_t& data)
	{
			return 0;
	}

};

#endif //DEBUG_WRAPPER_HPP
