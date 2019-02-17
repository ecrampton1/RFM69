#ifndef _RFM69_COMM_HPP
#define _RFM69_COMM_HPP

#include "rfm69/RFM69registers.h"


//Some default values
constexpr uint8_t Rfm69DefaultNodeId = 100;  //Make sure you change this for each node
constexpr uint8_t Rfm69DefaultRssi = 220;
constexpr uint8_t Rfm69DefaultSync = 0x2D;

constexpr uint8_t WRITE_ACCESS = 0x80;
constexpr uint8_t DUMMY_BYTE = 0x00;

constexpr uint8_t RF69_MAX_DATA_LEN = 61;  //FIFO size is 66 but with AES and CRC limit size

#define READ_8BIT_REGISTER(FUNCNAME,REG) \
		static uint8_t readRegister##FUNCNAME() { \
			return readRegister(REG); \
		}

#define READ_16BIT_REGISTER(FUNCNAME,REG) \
		static uint16_t readRegister##FUNCNAME() { \
			return readWordRegister(REG); \
		}

#define WRITE_8BIT_REGISTER(FUNCNAME,REG) \
		static void writeRegister##FUNCNAME(const uint8_t data) { \
			writeRegister(REG,data); \
		}

/*
#define WRITE_16BIT_REGISTER(FUNCNAME,REG) \
		static void writeRegister##FUNCNAME(const uint8_t data) { \
			writeWordRegister(REG,data); \
		}
*/

#define WRITE_REGISTERS(FUNCNAME,REG) \
		static void writeRegister##FUNCNAME(uint8_t* const data, const int size) { \
			writeRegisters(REG,data,size); \
		}

enum class CarrierFrequency
{
	FREQUENCY_315,
	FREQUENCY_433,
	FREQUENCY_868,
	FREQUENCY_915
};

constexpr uint8_t NO_ACK = 0x00;
constexpr uint8_t REQUEST_ACK = 0x01;
constexpr uint8_t SEND_ACK = 0x80;

struct PacketHeader
{
	//uint8_t Length; Length is not included in packet
	uint8_t Destination;
	uint8_t Source;
	uint8_t Control;
} __attribute__((packed));

template< class _spi, class _cs, CarrierFrequency _freq, class _uart>
class Rfm69Comm
{
public:

	static constexpr uint8_t BROADCAST_ADDRESS = 0xFF;

	static void init()
	{
		_cs::output();
		_cs::set();
		_spi::init();
		_spi::send(static_cast<uint8_t>(0x00));
		uint8_t ret = 0;
		//Is this necessary? Other projects do this not sure the reasoning
		do {
			writeRegisterSyncValue1(0xAA);
			ret = readRegisterSyncValue1();
		}
		while (ret != 0xAA);
		writeInitConfigs();
	}

	static void writeInitConfigs()
	{
		//Currently making this compatible with the arduino library inits.
		writeRegisterOpMode(RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY );
		writeRegisterDataModul( RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00); // no shaping
		writeRegisterBitRateMsb( RF_BITRATEMSB_55555 );
		writeRegisterBitRateLsb( RF_BITRATELSB_55555 ); // default: 4.8 KBPS
		writeRegisterFreqDevMsb(RF_FDEVMSB_50000); // default: 5KHz, (FDEV + BitRate / 2 <= 500KHz)
		writeRegisterFreqDevLsb(RF_FDEVLSB_50000);
		writeRegisterFrfMsb(FRFMSB);
		writeRegisterFrfMid(FRFMID);
		writeRegisterFrfLsb(FRFLSB);

		// looks like PA1 and PA2 are not implemented on RFM69W, hence the max output power is 13dBm
		// +17dBm and +20dBm are possible on RFM69HW
		// +13dBm formula: Pout = -18 + OutputPower (with PA0 or PA1**)
		// +17dBm formula: Pout = -14 + OutputPower (with PA1 and PA2)**
		// +20dBm formula: Pout = -11 + OutputPower (with PA1 and PA2)** and high power PA settings (section 3.3.7 in datasheet)
		///* 0x11 */ { REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | RF_PALEVEL_OUTPUTPOWER_11111},
		///* 0x13 */ { REG_OCP, RF_OCP_ON | RF_OCP_TRIM_95 }, // over current protection (default is 95mA)

		// RXBW defaults are { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_5} (RxBw: 10.4KHz)
		writeRegisterRxBw( RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_2 ); // (BitRate < 2 * RxBw)
		//for BR-19200: /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_3 },
		writeRegisterDioMapping1( RF_DIOMAPPING1_DIO0_01 ); // DIO0 is the only IRQ we're using
		writeRegisterDioMapping2( RF_DIOMAPPING2_CLKOUT_OFF ); // DIO5 ClkOut disable for power saving
		writeRegisterIrqFlags2( RF_IRQFLAGS2_FIFOOVERRUN ); // writing to this bit ensures that the FIFO & status flags are reset
		writeRegisterRssiThreshold( RSSILEVEL ); // must be set to dBm = (-Sensitivity / 2), default is 0xE4 = 228 so -114dBm
		///* 0x2D */ { REG_PREAMBLELSB, RF_PREAMBLESIZE_LSB_VALUE } // default 3 preamble bytes 0xAAAAAA
		writeRegisterSyncConfig( RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_4 | RF_SYNC_TOL_0 );
		writeRegisterSyncValue1( Rfm69DefaultSync );
		writeRegisterSyncValue2( Rfm69DefaultSync );
		writeRegisterSyncValue3( Rfm69DefaultSync );
		writeRegisterSyncValue4( 100 ); // DEFAULT NETWORK ID
		writeRegisterPacketConfig1( RF_PACKET1_FORMAT_VARIABLE | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON | RF_PACKET1_ADRSFILTERING_OFF );
		///* 0x38 */ { REG_PAYLOADLENGTH, 66 }, // in variable length mode: the max frame size, not used in TX DEFAULT is 0x40
		///* 0x39 */ { REG_NODEADRS, nodeID }, // turned off because we're not using address filtering
		writeRegisterFifoThreshold( RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE ); // TX on FIFO not empty
		writeRegisterPacketConfig2( RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF ); // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
		//for BR-19200: /* 0x3D */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_NONE | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
		writeRegisterTestDagc( RF_DAGC_IMPROVED_LOWBETA0 ); // run DAGC continuously in RX mode for Fading Margin Improvement, recommended default for AfcLowBetaOn=0

	}

	static void printAllRegisters()
	{
		uint8_t ret = 0;
		for( uint8_t i = 1; i <= 0x4F; ++i) {
			ret = readRegister(i);
			_uart::send(i);
			_uart::send("  -  ");
			_uart::send(ret);
			_uart::sendLine("\r");
		}
	}


	//This writes a rfm69 specific packet using the FIFO, note you can send empty packets useful for acks
	static int writePacket(PacketHeader& header,const uint8_t length=sizeof(PacketHeader), uint8_t* const buf=nullptr)
	{
		//I need to change REG defines to constexpr for better typing
		uint8_t addr = REG_FIFO | WRITE_ACCESS;
		_cs::clear();
		_spi::send( addr );
		_spi::send( length );
		int i = _spi::send(reinterpret_cast<uint8_t*>(&header),sizeof(PacketHeader));
		if(length > sizeof(PacketHeader)) {
			uint8_t userLength = length - sizeof(PacketHeader);
			i = i + _spi::send(buf,userLength);
		}
		_cs::set();

		return i;
	}

	static void writeRegister(const uint8_t address,const uint8_t data)
	{
		_cs::clear();
		uint8_t addr = address | WRITE_ACCESS;
		_spi::send( ( addr ) );
		_spi::send( data );
		_cs::set();
	}


	static int writeRegisters( const uint8_t address, uint8_t* const buf, const int size )
	{
		_cs::clear();
		uint8_t addr = address | WRITE_ACCESS;
		_spi::send( addr);
		int i = _spi::send(buf,size);
		_cs::set();

		return i;
	}

	//Functions for writing
	WRITE_8BIT_REGISTER( OpMode, REG_OPMODE )
	WRITE_8BIT_REGISTER( DataModul, REG_DATAMODUL )
	WRITE_8BIT_REGISTER( BitRateMsb, REG_BITRATEMSB )
	WRITE_8BIT_REGISTER( BitRateLsb, REG_BITRATELSB )
	WRITE_8BIT_REGISTER( FreqDevMsb, REG_FDEVMSB )
	WRITE_8BIT_REGISTER( FreqDevLsb, REG_FDEVLSB )
	WRITE_8BIT_REGISTER( SyncValue1, REG_SYNCVALUE1 )
	WRITE_8BIT_REGISTER( SyncValue2, REG_SYNCVALUE2 )
	WRITE_8BIT_REGISTER( SyncValue3, REG_SYNCVALUE3 )
	WRITE_8BIT_REGISTER( SyncValue4, REG_SYNCVALUE4 )
	WRITE_8BIT_REGISTER( FrfMsb, REG_FRFMSB )
	WRITE_8BIT_REGISTER( FrfMid, REG_FRFMID )
	WRITE_8BIT_REGISTER( FrfLsb, REG_FRFLSB )
	WRITE_8BIT_REGISTER( RxBw, REG_RXBW )
	WRITE_8BIT_REGISTER( DioMapping1, REG_DIOMAPPING1 )
	WRITE_8BIT_REGISTER( DioMapping2, REG_DIOMAPPING2 )
	WRITE_8BIT_REGISTER( IrqFlags1, REG_IRQFLAGS1 )
	WRITE_8BIT_REGISTER( IrqFlags2, REG_IRQFLAGS2 )
	WRITE_8BIT_REGISTER( RssiThreshold, REG_RSSITHRESH )
	WRITE_8BIT_REGISTER( SyncConfig, REG_SYNCCONFIG )
	WRITE_8BIT_REGISTER( PacketConfig1, REG_PACKETCONFIG1 )
	WRITE_8BIT_REGISTER( PacketConfig2, REG_PACKETCONFIG2 )
	WRITE_8BIT_REGISTER( FifoThreshold, REG_FIFOTHRESH )
	WRITE_8BIT_REGISTER( TestDagc, REG_TESTDAGC )
	WRITE_8BIT_REGISTER( NodeAddress, REG_NODEADRS )
	WRITE_8BIT_REGISTER( RssiConfig, REG_RSSICONFIG )
	WRITE_REGISTERS(AesKey,REG_AESKEY1)


	///Function used for reading from the RFM69
	READ_8BIT_REGISTER( OpMode, REG_OPMODE )
	READ_8BIT_REGISTER( DataModul, REG_DATAMODUL )
	READ_16BIT_REGISTER( BitRate, REG_BITRATEMSB )
	READ_16BIT_REGISTER( FreqDev, REG_FDEVMSB )
	READ_8BIT_REGISTER( SyncValue1, REG_SYNCVALUE1)
	READ_8BIT_REGISTER( SyncValue2, REG_SYNCVALUE2)
	READ_8BIT_REGISTER( IrqFlags1, REG_IRQFLAGS1)
	READ_8BIT_REGISTER( IrqFlags2, REG_IRQFLAGS2)
	READ_8BIT_REGISTER( NodeAddress, REG_NODEADRS )
	READ_8BIT_REGISTER( RssiConfig, REG_RSSICONFIG )
	READ_8BIT_REGISTER( RssiValue, REG_RSSIVALUE )
	READ_8BIT_REGISTER( PacketConfig2, REG_PACKETCONFIG2 )

	static int readPacket(uint8_t* ret_buf,const int max_size)
	{

		_cs::clear();
		_spi::send( static_cast<uint8_t>(REG_FIFO) );
		//This is actually the length of the payload in FIFO
		int length = _spi::exchange( DUMMY_BYTE );
		//sanity check that our buffer size can handle this and its not 0
		if(length >= sizeof(PacketHeader) && length <= max_size) {
			for(int i = 0; i <= length; ++i)//length is not include in the length count
			{
				ret_buf[i] = _spi::exchange( DUMMY_BYTE );
			}
		}
		else{
			for(int i = 1; i <= length; ++i)
			{
				_uart::send( _spi::exchange( DUMMY_BYTE) ,McuPeripheral::Base::BASE_HEX);
			}
			_uart::sendLine();
			length = -1;
		}
		_cs::set();

		while((readRegisterIrqFlags2() & RF_IRQFLAGS2_FIFONOTEMPTY))
		{
			_cs::clear();
			_spi::send( static_cast<uint8_t>(REG_FIFO) );
			_uart::send( _spi::exchange( DUMMY_BYTE) ,McuPeripheral::Base::BASE_HEX);
			_cs::set();
		}



		return length;
	}

	static uint8_t readRegister(uint8_t address)
	{
		_cs::clear();
		_spi::send( address );
		uint8_t ret = _spi::exchange( DUMMY_BYTE );
		_cs::set();

		return ret;
	}


	static uint16_t readWordRegister(uint8_t address)
	{
		_cs::clear();
		_spi::send( address );
		uint16_t ret = _spi::exchange( DUMMY_BYTE ); //read msb first
		ret = (ret << 8) | _spi::exchange( DUMMY_BYTE ); //read lsb
		_cs::set();

		return ret;
	}


	static int readRegisters(const uint8_t address, uint8_t* ret_buf,const int size)
	{
		//possible improvement to create a static buffer to send or  make spi send
		//a byte over and over
		_cs::clear();
		_spi::send( address );
		int i =0;
		while(i < size) {
			ret_buf[i++] = _spi::exchange((uint8_t)DUMMY_BYTE);
		}
		_cs::set();

		return i;
	}


private:
	static const uint8_t FRFMSB = (_freq == CarrierFrequency::FREQUENCY_315 ) ? RF_FRFMSB_315 : (_freq == CarrierFrequency::FREQUENCY_433) ? RF_FRFMSB_433 : (_freq == CarrierFrequency::FREQUENCY_868) ? RF_FRFMSB_868 : RF_FRFMSB_915;
	static const uint8_t FRFMID = (_freq == CarrierFrequency::FREQUENCY_315 ) ? RF_FRFMID_315 : (_freq == CarrierFrequency::FREQUENCY_433) ? RF_FRFMID_433 : (_freq == CarrierFrequency::FREQUENCY_868) ? RF_FRFMID_868 : RF_FRFMID_915;
	static const uint8_t FRFLSB = (_freq == CarrierFrequency::FREQUENCY_315 ) ? RF_FRFLSB_315 : (_freq == CarrierFrequency::FREQUENCY_433) ? RF_FRFLSB_433 : (_freq == CarrierFrequency::FREQUENCY_868) ? RF_FRFLSB_868 : RF_FRFLSB_915;
	static const uint8_t RSSILEVEL = 220;  //Configurable?
};



#endif //_RFM69_COMM_HPP
