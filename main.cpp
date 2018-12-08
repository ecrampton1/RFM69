#include "spi_wrapper.hpp"
#include "gpio_wrapper.hpp"
#include "sys_wrapper.hpp"
#include "rfm69/rfm69.hpp"

using DIO0 = GpioWrapper<4>;
using CS = GpioWrapper<1>;
using RF = Rfm69< SpiDev0, SysWrapper,  CS, DIO0, CarrierFrequency::FREQUENCY_915>;

int main()
{
	RF::init();
	printf("Init Complete\n");
	return 1;
}
