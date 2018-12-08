#ifndef _GPIO_WRAPPER_HPP
#define _GPIO_WRAPPER_HPP
#include <wiringPi.h>

typedef void (*callback_t) (void);

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
	
	static void input()
	{
		wiringPiSetup ();
		pinMode (gpio, INPUT);
	}
	
	static void setPinIrqHandler(callback_t cb, void* args)
	{
		mPinHandler = cb;
	}
	
	static void edgeLowToHigh() 
	{
		mEdgeType = INT_EDGE_RISING;
	}
	
	static void intEnable()
	{
		if (wiringPiISR (gpio, mEdgeType, mPinHandler) < 0) {
			printf("Error on gpio interrupt setup");
    	}
	}
	
private:
	static callback_t mPinHandler;
	static int mEdgeType;
	
};

template<int gpio>
callback_t GpioWrapper<gpio>::mPinHandler;

template<int gpio>
int GpioWrapper<gpio>::mEdgeType;
#endif //_GPIO_WRAPPER_HPP
