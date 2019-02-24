#ifndef _GPIO_WRAPPER_HPP
#define _GPIO_WRAPPER_HPP
#include <wiringPi.h>
#include <stdlib.h>

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
		//static enabled = false;
		
		//if(false == enabled) {
		if (wiringPiISR (gpio, mEdgeType, mPinHandler) < 0) {
			printf("Error on gpio interrupt setup");
    	}
    	/***Maybe not needed**}
    	else {
    		//Hacky work around since wiringpi doesnt support disabling the gpio irq
    		system ("/usr/local/bin/gpio edge 17 rising");
    	}*/
	}
	
	static void intDisable()
	{
		system ("/usr/local/bin/gpio edge 17 none");
	}
	
	static void clearIntFlag()
	{
		//Do nothing? RPI should be not need to clear this...
	}
	
	static bool read()
	{
		return digitalRead(gpio);
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
