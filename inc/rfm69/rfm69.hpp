#ifndef _RFM69_HPP
#define _RFM69_HPP
#include "rfm69/rfm69_comm.hpp"
#include <string.h>
#include "msp430/msp_uart.hpp"
#include "msp430/msp_gpio.hpp"

static constexpr uint8_t ACK_RETRIES = 5;
static constexpr uint8_t ACK_TIMEOUT = 20;
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
		rfm69_comm::writeRegisterSyncValue4(address);
	}

	static uint8_t getNetworkAddress()
	{
		return rfm69_comm::readRegisterSyncValue4();
	}

	static bool isPayloadReady()
	{
		return mPayloadReady;
	}

	static int readPayload(uint8_t* ret_buf, const int max_size)
	{
		//disable interrupts?
		PacketHeader* header = reinterpret_cast<PacketHeader*>(ret_buf);
		//enableStandby();
		mPayloadReady = false;
		int length = rfm69_comm::readPacket(ret_buf,max_size);
		if(0 > length) {
			forceRestartRx();
			return -1;
		}

		if(REQUEST_ACK == header->Control){
			writePayload(nullptr,0,header->Source,SEND_ACK);
		}

		return length;

	}

	static bool writePayload(uint8_t* const buf, const int size, uint8_t destination_node)
	{
		return writePayload(buf,size,destination_node,NO_ACK);
	}

	static bool writePayload(uint8_t* const buf, const int size, uint8_t destination_node, uint8_t control)
	{
		PacketHeader header;
		buildPacketHeader(header,destination_node,control);
		if(false == waitForReadyToSend()){
			return false;
		}
		return writeAndWaitForSent(header,size + sizeof(PacketHeader), buf);
	}

	static bool writePayloadWithAck(uint8_t* const buf, const int size, uint8_t destination_node)
	{

		//if(false == waitForReadyToSend()){
		//	return false;
		//}
		uint8_t buffer[sizeof(PacketHeader)];
		bool ret = false;
		//PacketHeader header;
		PacketHeader* header = reinterpret_cast<PacketHeader*>(buffer);
		for(int i = 0; i < ACK_RETRIES; ++i) {
			writePayload(buf,size,destination_node,REQUEST_ACK);
			//waitForModeReady();
			enableRx();
			int j = 0;
			while(false == mPayloadReady && ++j < ACK_TIMEOUT) {
				_sys::delayInMs(1);
			}
			if(j >= ACK_TIMEOUT) {_uart::sendLine("TOut"); continue;}
			readPayload(buffer,sizeof(buffer));
			if(SEND_ACK == header->Control && destination_node == header->Source){
				ret = true;
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
		mPayloadReady = false;
		if(isIrqPayloadReady()) {
			forceRestartRx();
		}
		enablePayloadReadyIrq();
		rfm69_comm::writeRegisterOpMode((rfm69_comm::readRegisterOpMode() & 0xE3) | RF_OPMODE_RECEIVER);
		//waitForModeReady();
	}

	static bool isRxEnabled()
	{
		return (rfm69_comm::readRegisterOpMode() & RF_OPMODE_RECEIVER);
	}

	static bool isTxEnabled()
	{
		return (rfm69_comm::readRegisterOpMode() & RF_OPMODE_RECEIVER);
	}

	//TODO Listen Mode?
	//TODO add setnetwork id function?

private:

	static void buildPacketHeader(PacketHeader& header, uint8_t desitnation_node, uint8_t control)
	{
		header.Source = mNode;
		header.Destination = desitnation_node;
		header.Control = control;
	}

	static void configInterrupt()
	{
		_irq::input();
		_irq::setPinIrqHandler(&gpioIrqTriggered,0);
		_irq::edgeLowToHigh();
		_irq::clearIntFlag();
		_irq::intEnable();
	}

	static void gpioIrqTriggered(void* args)
	{
		if(isRxEnabled()) {
			if( isIrqPayloadReady() ) {
				mPayloadReady = true;
				enableStandby();
			}
		}
	}

	static void enablePayloadReadyIrq()
	{
		rfm69_comm::writeRegisterDioMapping1(RF_DIOMAPPING1_DIO0_01);//DIO0 triggers on payload ready in rx mode
		_irq::clearIntFlag();  //remove any flags set previously.
		_irq::intEnable();
	}

	//Note this function is going to enable polling of the pin for a quicker response in the TX mode
	static void enablePacketSentIrq()
	{
		rfm69_comm::writeRegisterDioMapping1(RF_DIOMAPPING1_DIO0_00);//DIO0 triggers on packet sent in tx mode
		_irq::intDisable();
	}

	static bool isReady()
	{
		return (rfm69_comm::readRegisterIrqFlags1() & RF_IRQFLAGS1_MODEREADY);
	}

	static bool isIrqPayloadReady()
	{
		const uint8_t flags = rfm69_comm::readRegisterIrqFlags2();
		return (flags & RF_IRQFLAGS2_PAYLOADREADY);
	}

	static bool packetSent()
	{
		return (rfm69_comm::readRegisterIrqFlags2() & RF_IRQFLAGS2_PACKETSENT);
	}

	static void forceRestartRx()
	{
		rfm69_comm::writeRegisterPacketConfig2(rfm69_comm::readRegisterPacketConfig2() | RF_PACKET2_RXRESTART);
	}

	static void enableTx()
	{
		//reset packet sent status
		//mPacketSent = false;
		rfm69_comm::writeRegisterOpMode((rfm69_comm::readRegisterOpMode() & 0xE3) | RF_OPMODE_TRANSMITTER);
	}

	static void enableStandby()
	{
		rfm69_comm::writeRegisterOpMode((rfm69_comm::readRegisterOpMode() & 0xE3) | RF_OPMODE_STANDBY);
	}

	static bool writeAndWaitForSent(PacketHeader& header,const uint8_t length, uint8_t* const buf)
	{
		waitForModeReady();
		enablePacketSentIrq();
		int i = rfm69_comm::writePacket(header, length, buf);
		enableTx();
		bool ret = waitForPacketSent();
		//enableStandby();  //TODO maybe this isnt necessary?  Go straight to rx mode?
		return ret;
	}

	static bool waitForModeReady()
	{
		static const uint32_t timeOut = 500;
		uint32_t currentTime = _sys::millis();
		while( false == isReady() ){
			if(currentTime + timeOut < _sys::millis()) {
				_uart::sendLine("wFReady");
				return false; //Something not quite right.
			}
		}
		return true;
	}

	static bool waitForPacketSent()
	{
		int i = 0;

		//while(0 == (rfm69_comm::readRegisterIrqFlags2() & RF_IRQFLAGS2_PACKETSENT)) {
		//while(false == mPacketSent) {
		while(false == _irq::read()){
			_sys::delayInUs(10);
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
	static volatile bool mPayloadReady;
	//static volatile bool mPacketSent;
	static uint8_t mNode;
};

template< class _spi, class _sys,  class _cs, class _irq, CarrierFrequency _freq,  class _uart>
volatile bool Rfm69<_spi, _sys, _cs, _irq, _freq,  _uart>::mPayloadReady = false;
/*template< class _spi, class _sys,  class _cs, class _irq, CarrierFrequency _freq,  class _uart>
volatile bool Rfm69<_spi, _sys, _cs, _irq, _freq,  _uart>::mPacketSent = false;*/
template< class _spi, class _sys,  class _cs, class _irq, CarrierFrequency _freq,  class _uart>
char Rfm69<_spi, _sys, _cs, _irq, _freq, _uart>::mEncryptKey[16] = { 0 };
template< class _spi, class _sys,  class _cs, class _irq, CarrierFrequency _freq, class _uart>
uint8_t Rfm69<_spi, _sys, _cs, _irq, _freq, _uart>::mNode = 0xFF;


#endif
