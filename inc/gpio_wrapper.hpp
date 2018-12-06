#ifndef _GPIO_WRAPPER_HPP
#define _GPIO_WRAPPER_HPP
#include <wiringPi.h>

template<int gpio>
class GpioWrapper
{
public:

	static void output()
	{
		wiringPiSetup ();
		pinMode (gpio, OUTPUT);
	}
	
	static void set()
	{
		digitalWrite (gpio, HIGH);
	}
	
	static void clear()
	{
		digitalWrite (gpio, LOW);
	}
	
}

#endif //_GPIO_WRAPPER_HPP
