#ifndef _GPIO_WRAPPER_HPP
#define _GPIO_WRAPPER_HPP
#include <unistd.h>

class SysWrapper
{
public:

	static inline void delayInMs(uint32_t timeInMs)
	{
		usleep(timeInMS*1000);
	}
	
	
	static inline uint32_t millis()
	{
		struct timeval  tv;
		gettimeofday(&tv, NULL);

		double time_in_mill = 
		      (tv.tv_sec) * 1000 + (tv.tv_usec) / 1000 ;
		return static_cast<uint32_t>(time_in_mill); 
	} 	
	
};
