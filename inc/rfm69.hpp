#ifndef _RFM69_HPP
#define _RFM69_HPP
#include "rfm69/rfm69_comm.hpp"
#include <string.h>
#include "msp430/msp_uart.hpp"

static constexpr uint8_t ACK_RETRIES = 5;
static constexpr uint8_t ACK_TIMEOUT = 10;
//TODO in general better timeout handling and fault handling in this class start with while loop timeouts.

template< class _spi, class _sys,  class _cs, class _irq, CarrierFrequency _freq, class _uart>
class Rfm69
{
	using rfm69_comm = Rfm69Comm< _spi, _cs, _freq, _uart >;

public:
	static void init()
	{
		rfm69_comm::init();
		setNodeAddress(99);
		waitForModeReady();
		configInterrupt();
		mPayloadReady = false;
	}

	static void putToSleep()
	{
		enableSleep();
	}

	static void printAllRegisters()
	{
		rfm69_comm::printAllRegisters();
	}

	static void setNodeAddress(uint8_t address)
	{
		rfm69_comm::writeRegisterNodeAddress(address);
		mNode = address;
	}

	static uint8_t getNodeAddress()
	{
		mNode = rfm69_comm::readRegisterNodeAddress();
		return mNode;
	}

	static void setNetworkAddress(uint8_t address)
	{
		rfm69_comm::writeRegisterSyncValue2(address);
	}

	static uint8_t getNetworkAddress()
	{
		return rfm69_comm::readRegisterSyncValue2();
	}

	static bool isPayloadReady()
	{
		return mPayloadReady;
	}

	static int readPayload(uint8_t* ret_buf, const int max_size)
	{
		//disable interrupts?
		PacketHeader* header = reinterpret_cast<PacketHeader*>(ret_buf);
		mPayloadReady = false;
		enableStandby();
		int length = rfm69_comm::readPacket(ret_buf,max_size);
		if(0 > length) {
			forceRestartRx();
			return -1;
		}

		if(REQUEST_ACK == header->Control){
			writePayload(nullptr,0,header->Source,SEND_ACK);
		}

		return header->Length;

	}

	static bool writePayload(uint8_t* const buf, const int size, uint8_t destination_node)
	{
		return writePayload(buf,size,destination_node,NO_ACK);
	}

	static bool writePayload(uint8_t* const buf, const int size, uint8_t destination_node, uint8_t control)
	{
		PacketHeader header;
		buildPacketHeader(header,size,destination_node,control);
		if(false == waitForReadyToSend()){
			return false;
		}
		return writeAndWaitForSent(header,buf);
	}

	static bool writePayloadWithAck(uint8_t* const buf, const int size, uint8_t destination_node)
	{

		if(false == waitForReadyToSend()){
			return false;
		}
		bool ret = false;
		PacketHeader header;
		for(int i = 0; i < ACK_RETRIES; ++i) {
			writePayload(buf,size,destination_node,REQUEST_ACK);
			enableRx();
			int j = 0;
			while(false == mPayloadReady && ++j < ACK_TIMEOUT) {
				_sys::delayInMs(1);
			}
			if(j > ACK_TIMEOUT) {continue;}
			readPayload(reinterpret_cast<uint8_t*>(&header),sizeof(header));
			if(SEND_ACK == header.Control && destination_node == header.Source){
				ret = true;
				McuPeripheral::McuUart<UartA0, McuPeripheral::BaudRate::BAUD_115200, Speed::SPEED_16MHZ>::send("ACK Received\n");
				break;
			}
		}
		return ret;
	}

	//.5dB steps where RSSI = RssiValue/2 dBm
	static int readRssi(bool forceReading=false)
	{
		int reading = 0;
		if(true == forceReading) {
			rfm69_comm::writeRegisterRssiConfig(RF_RSSI_START);
			while(false == isRssiReady());
		}
		reading = rfm69_comm::readRegisterRssiValue();
		reading = -reading>>1;
		return reading;
	}

	static void setEncryptionKey(const char* key)
	{
		enableStandby();
		memcpy(mEncryptKey,key,sizeof(mEncryptKey));
		rfm69_comm::writeRegisterAesKey(reinterpret_cast<unsigned char *>(mEncryptKey),sizeof(mEncryptKey));
		enableEncryption();
	}

	static void enableEncryption()
	{
		rfm69_comm::writeRegisterPacketConfig2(rfm69_comm::readRegisterPacketConfig2() | RF_PACKET2_AES_ON );
	}

	static void disableEncryption()
	{
		rfm69_comm::writeRegisterPacketConfig2(rfm69_comm::readRegisterPacketConfig2() & ~RF_PACKET2_AES_ON );
	}

	static void enableSleep()
	{
		rfm69_comm::writeRegisterOpMode((rfm69_comm::readRegisterOpMode() & 0xE3) | RF_OPMODE_SLEEP);
	}

	static void enableRx()
	{
		forceRestartRx();
		rfm69_comm::writeRegisterOpMode((rfm69_comm::readRegisterOpMode() & 0xE3) | RF_OPMODE_RECEIVER);
		enablePayloadReadyIrq();
		waitForModeReady();
	}

	static bool isRxEnabled()
	{
		return (rfm69_comm::readRegisterOpMode() & RF_OPMODE_RECEIVER);
	}

	//TODO Listen Mode?
	//TODO add setnetwork id function?

private:

	static void buildPacketHeader(PacketHeader& header, const int size, uint8_t desitnation_node, uint8_t control)
	{
		header.Source = mNode;
		header.Destination = desitnation_node;
		header.Length = size + sizeof(header);
		header.Control = control;
	}

	static void configInterrupt()
	{
		_irq::input();
		_irq::setPinIrqHandler(&gpioIrqTriggered,0);
		_irq::edgeLowToHigh();
		_irq::intEnable();
		enablePayloadReadyIrq();
	}

	static void gpioIrqTriggered(void* args)
	{
		if(isRxEnabled() && isIrqPayloadReady())
		{
			mPayloadReady = true;
			enableStandby();
		}
	}

	static void enablePayloadReadyIrq()
	{
		rfm69_comm::writeRegisterDioMapping1(RF_DIOMAPPING1_DIO0_01);//DIO0 triggers on payload ready in rx mode
	}

	static bool isReady()
	{
		return (rfm69_comm::readRegisterIrqFlags1() & RF_IRQFLAGS1_MODEREADY);
	}

	static bool isIrqPayloadReady()
	{
		return (rfm69_comm::readRegisterIrqFlags2() & RF_IRQFLAGS2_PAYLOADREADY);
	}

	static void forceRestartRx()
	{
		rfm69_comm::writeRegisterPacketConfig2(rfm69_comm::readRegisterPacketConfig2() | RF_PACKET2_RXRESTART);
	}

	static void enableTx()
	{
		rfm69_comm::writeRegisterOpMode((rfm69_comm::readRegisterOpMode() & 0xE3) | RF_OPMODE_TRANSMITTER);
	}

	static void enableStandby()
	{
		rfm69_comm::writeRegisterOpMode((rfm69_comm::readRegisterOpMode() & 0xE3) | RF_OPMODE_STANDBY);
	}

	static bool writeAndWaitForSent(PacketHeader& header,uint8_t* const buf)
	{
		int i = rfm69_comm::writePacket(header,buf);
		enableTx();
		return waitForPacketSent();
	}

	static bool waitForModeReady()
	{
		static const uint32_t timeOut = 500;
		uint32_t currentTime = _sys::millis();
		while( false == isReady() ){
			if(currentTime + timeOut < _sys::millis()) {
				return false; //Something not quite right.
			}
		}
		return true;
	}

	static bool waitForPacketSent()
	{
		int i = 0;

		while(0 == (rfm69_comm::readRegisterIrqFlags2() & RF_IRQFLAGS2_PACKETSENT)) {
			_sys::delayInMs(1);
			if(++i > 500) {
				return false;
			}
		}
		return true;
	}

	static bool isRssiReady()
	{
		return (rfm69_comm::readRegisterRssiConfig() & RF_RSSI_DONE);
	}

	static bool waitForReadyToSend()
	{
		const uint32_t timeOut = 500 + _sys::millis();

		enableRx();
		while(false == checkRxRssiLimit()) {
			forceRestartRx();
			if(timeOut < _sys::millis()) {
				return false; //Something not quite right.
			}
		}

		enableStandby();

		return true;
	}

	static bool checkRxRssiLimit()
	{
		bool ret = false;
		static constexpr int channelRssiLimit = -100;
		if(isRxEnabled() && (readRssi() < channelRssiLimit)) {
			ret = true;
		}
		return ret;
	}


	static char mEncryptKey[16];
	static bool mPayloadReady;
	static uint8_t mNode;
};

template< class _spi, class _sys,  class _cs, class _irq, CarrierFrequency _freq,  class _uart>
bool Rfm69<_spi, _sys, _cs, _irq, _freq,  _uart>::mPayloadReady = false;
template< class _spi, class _sys,  class _cs, class _irq, CarrierFrequency _freq,  class _uart>
char Rfm69<_spi, _sys, _cs, _irq, _freq, _uart>::mEncryptKey[16] = { 0 };
template< class _spi, class _sys,  class _cs, class _irq, CarrierFrequency _freq, class _uart>
uint8_t Rfm69<_spi, _sys, _cs, _irq, _freq, _uart>::mNode = 0xFF;


#endif
