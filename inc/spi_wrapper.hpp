#ifndef _SPI_WRAPPER_HPP
#define _SPI_WRAPPER_HPP
#incldue <stdio.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>

constexpr uint32_t SPI_SPEED_HZ = 4000000; //4MHZ spi clock

template< int spi >
class SpiWrapper
{	
public:	
	static bool init() {
		wiringPiSetup();
		if (wiringPiSPISetup (spi, SPI_SPEED_HZ) < 0) {
			printf("Can't open the SPI bus: %d\n", errno);
			return false;
		}
		return true;
	}

	static int read( uint8_t data )
	{
		return read( &data, 1);
	}

	static int read( uint8_t* data, int size)
	{
		int ret = wiringPiSPIDataRW(spi,data,size);
		if(ret == -1) {
			ret = 0; //0 bytes read
		}
		else {
			ret = size; 	
		}
		return ret;
	}


	static int send( uint8_t* data, int size)
	{
		int ret = wiringPiSPIDataRW(spi,data,size);
		if(ret == -1) {
			ret = 0; //0 bytes sent
		}
		else {
			ret = size; 
		}
		return i;
	}



	static uint8_t exchange( uint8_t data )
	{
		T return_data = 0;

		exchange(&data,&return_data,1);
		return return_data;
	}

	static int exchange( uint8_t* data, uint8_t* return_data, int size)
	{
		int ret = wiringPiSPIDataRW(spi,data,size);
		if(ret == -1) {
			ret = 0; //0 bytes read
		}
		else {
			memcpy(data,return_data,size);
			ret = size; 	
		}
		return ret;
	}

	
};

using SpiDev0 = SpiWrapper<0>;

#endif //_SPI_WRAPPER_HPP
