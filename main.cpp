#include "spi_wrapper.hpp"
#include "gpio_wrapper.hpp"
#include "sys_wrapper.hpp"
#include "rfm69/rfm69.hpp"

using DIO0 = GpioWrapper<4>;
using CS = GpioWrapper<1>;
using RF = Rfm69< SpiDev0, SysWrapper,  CS, DIO0, CarrierFrequency::FREQUENCY_915>;


static const char* AesKey = "thisIsEncryptKey";

static uint8_t buffer[66];


struct Payload {
  int16_t           nodeId; //store this nodeId
  uint32_t uptime; //uptime in ms
  uint32_t         temp;   //temperature maybe?
}__attribute__((packed));

void setup()
{
	RF::init();
	RF::enableRx();
	RF::printAllRegisters();
	RF::setNodeAddress(99);
	RF::setNetworkAddress(100);
}

void receiver()
{
	PacketHeader* header = reinterpret_cast<PacketHeader*>(buffer);
	if(RF::isPayloadReady()) {
		RF::readPayload(buffer,sizeof(buffer));
		printf("Time: %u - %x - %x - %x - %x\n", SysWrapper::millis(), header->Length,header->Destination, header->Source,header->Control);
		Payload* payload  = reinterpret_cast<Payload*>(&buffer[sizeof(PacketHeader)]);
		printf("%x - %u - %x\n",payload->nodeId,payload->uptime,payload->temp);
		RF::enableRx();
	}
}

void sender()
{
	Payload* payload = reinterpret_cast<Payload*>(&buffer[0]);
	payload->nodeId = 75;
	payload->uptime = SysWrapper::millis();
	payload->temp = 37;

	bool ret = RF::writePayloadWithAck(buffer,sizeof(Payload),75);
	if(ret == true) {
		printf("Sent\n");
	}
	else {
		printf("Not Sent\n");
	}
}

int main()
{
	setup();
	printf("Init Complete\n");
	
	while(1){
		//receiver();
		sender();
		SysWrapper::delayInMs(100);
	}
	return 1;
}
