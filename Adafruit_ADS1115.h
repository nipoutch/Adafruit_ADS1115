/**************************************************************************/
/*!
@file Adafruit_ADS1115.h
@author K. Townsend (Adafruit Industries)
@license BSD (see license.txt)

This is a library for the Adafruit ADS1115 breakout board
----> https://www.adafruit.com/products/???

Adafruit invests time and resources providing this open source code,
please support Adafruit and open-source hardware by purchasing
products from Adafruit!

@section HISTORY

v1.0 - First release
v1.1 - Add setSPS fonction to select SPS value with good associate delay by Nipoutch
*/
/**************************************************************************/

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>
#include <util/delay.h>

/*=========================================================================
I2C ADDRESS/BITS
-----------------------------------------------------------------------*/
#define ADS1115_ADDRESS	(0x48) // 1001 000 (ADDR = GND)
/*=========================================================================*/

/*=========================================================================
CONVERSION DELAY (in uS)
-----------------------------------------------------------------------*/
#define ADS1115_CONVERSIONDELAY_8 		(128000)
#define ADS1115_CONVERSIONDELAY_16 		(64000)
#define ADS1115_CONVERSIONDELAY_32 		(32000)
#define ADS1115_CONVERSIONDELAY_64 		(16000)
#define ADS1115_CONVERSIONDELAY_128 	(8000)
#define ADS1115_CONVERSIONDELAY_250 	(4000)
#define ADS1115_CONVERSIONDELAY_475 	(2200)
#define ADS1115_CONVERSIONDELAY_860 	(1200)
/*=========================================================================*/

/*=========================================================================
POINTER REGISTER
-----------------------------------------------------------------------*/
#define ADS1115_REG_POINTER_MASK 		(0x03)
#define ADS1115_REG_POINTER_CONVERT 	(0x00)
#define ADS1115_REG_POINTER_CONFIG 		(0x01)
#define ADS1115_REG_POINTER_LOWTHRESH 	(0x02)
#define ADS1115_REG_POINTER_HITHRESH 	(0x03)
/*=========================================================================*/

/*=========================================================================
CONFIG REGISTER
-----------------------------------------------------------------------*/
#define ADS1115_REG_CONFIG_OS_MASK 		(0x8000)
#define ADS1115_REG_CONFIG_OS_SINGLE 	(0x8000) // Write: Set to start a single-conversion
#define ADS1115_REG_CONFIG_OS_BUSY 		(0x0000) // Read: Bit = 0 when conversion is in progress
#define ADS1115_REG_CONFIG_OS_NOTBUSY 	(0x8000) // Read: Bit = 1 when device is not performing a conversion

#define ADS1115_REG_CONFIG_MUX_MASK 	(0x7000)
#define ADS1115_REG_CONFIG_MUX_DIFF_0_1 (0x0000) // Differential P = AIN0, N = AIN1 (default)
#define ADS1115_REG_CONFIG_MUX_DIFF_0_3 (0x1000) // Differential P = AIN0, N = AIN3
#define ADS1115_REG_CONFIG_MUX_DIFF_1_3 (0x2000) // Differential P = AIN1, N = AIN3
#define ADS1115_REG_CONFIG_MUX_DIFF_2_3 (0x3000) // Differential P = AIN2, N = AIN3
#define ADS1115_REG_CONFIG_MUX_SINGLE_0 (0x4000) // Single-ended AIN0
#define ADS1115_REG_CONFIG_MUX_SINGLE_1 (0x5000) // Single-ended AIN1
#define ADS1115_REG_CONFIG_MUX_SINGLE_2 (0x6000) // Single-ended AIN2
#define ADS1115_REG_CONFIG_MUX_SINGLE_3 (0x7000) // Single-ended AIN3

#define ADS1115_REG_CONFIG_PGA_MASK 	(0x0E00)
#define ADS1115_REG_CONFIG_PGA_6_144V 	(0x0000) // +/-6.144V range = Gain 2/3
#define ADS1115_REG_CONFIG_PGA_4_096V 	(0x0200) // +/-4.096V range = Gain 1
#define ADS1115_REG_CONFIG_PGA_2_048V 	(0x0400) // +/-2.048V range = Gain 2 (default)
#define ADS1115_REG_CONFIG_PGA_1_024V 	(0x0600) // +/-1.024V range = Gain 4
#define ADS1115_REG_CONFIG_PGA_0_512V 	(0x0800) // +/-0.512V range = Gain 8
#define ADS1115_REG_CONFIG_PGA_0_256V 	(0x0A00) // +/-0.256V range = Gain 16

#define ADS1115_REG_CONFIG_MODE_MASK 	(0x0100)
#define ADS1115_REG_CONFIG_MODE_CONTIN 	(0x0000) // Continuous conversion mode
#define ADS1115_REG_CONFIG_MODE_SINGLE 	(0x0100) // Power-down single-shot mode (default)

#define ADS1115_REG_CONFIG_DR_MASK 		(0x00E0)
#define ADS1115_REG_CONFIG_DR_8SPS 		(0x0000) // 8 samples per second on ADS1115
#define ADS1115_REG_CONFIG_DR_16SPS 	(0x0020) // 16 samples per second on ADS1115
#define ADS1115_REG_CONFIG_DR_32SPS 	(0x0040) // 32 samples per second on ADS1115
#define ADS1115_REG_CONFIG_DR_64SPS 	(0x0060) // 64 samples per second on ADS1115
#define ADS1115_REG_CONFIG_DR_128SPS 	(0x0080) // 128 samples per second on ADS1115
#define ADS1115_REG_CONFIG_DR_250SPS 	(0x00A0) // 250 samples per second on ADS1115
#define ADS1115_REG_CONFIG_DR_475SPS 	(0x00C0) // 475 samples per second on ADS1115
#define ADS1115_REG_CONFIG_DR_860SPS 	(0x00E0) // 860 samples per second on ADS1115


#define ADS1115_REG_CONFIG_CMODE_MASK 	(0x0010)
#define ADS1115_REG_CONFIG_CMODE_TRAD 	(0x0000) // Traditional comparator with hysteresis (default)
#define ADS1115_REG_CONFIG_CMODE_WINDOW (0x0010) // Window comparator

#define ADS1115_REG_CONFIG_CPOL_MASK 	(0x0008)
#define ADS1115_REG_CONFIG_CPOL_ACTVLOW (0x0000) // ALERT/RDY pin is low when active (default)
#define ADS1115_REG_CONFIG_CPOL_ACTVHI 	(0x0008) // ALERT/RDY pin is high when active

#define ADS1115_REG_CONFIG_CLAT_MASK 	(0x0004) // Determines if ALERT/RDY pin latches once asserted
#define ADS1115_REG_CONFIG_CLAT_NONLAT 	(0x0000) // Non-latching comparator (default)
#define ADS1115_REG_CONFIG_CLAT_LATCH 	(0x0004) // Latching comparator

#define ADS1115_REG_CONFIG_CQUE_MASK	(0x0003)
#define ADS1115_REG_CONFIG_CQUE_1CONV 	(0x0000) // Assert ALERT/RDY after one conversions
#define ADS1115_REG_CONFIG_CQUE_2CONV 	(0x0001) // Assert ALERT/RDY after two conversions
#define ADS1115_REG_CONFIG_CQUE_4CONV 	(0x0002) // Assert ALERT/RDY after four conversions
#define ADS1115_REG_CONFIG_CQUE_NONE 	(0x0003) // Disable the comparator and put ALERT/RDY in high state (default)
/*=========================================================================*/

typedef enum
{
	GAIN_TWOTHIRDS 	= ADS1115_REG_CONFIG_PGA_6_144V,
	GAIN_ONE 		= ADS1115_REG_CONFIG_PGA_4_096V,
	GAIN_TWO 		= ADS1115_REG_CONFIG_PGA_2_048V,
	GAIN_FOUR		= ADS1115_REG_CONFIG_PGA_1_024V,
	GAIN_EIGHT 		= ADS1115_REG_CONFIG_PGA_0_512V,
	GAIN_SIXTEEN 	= ADS1115_REG_CONFIG_PGA_0_256V
} adsGain_t;

typedef enum
{
	SPS_8 			= ADS1115_REG_CONFIG_DR_8SPS,
	SPS_16			= ADS1115_REG_CONFIG_DR_16SPS,
	SPS_32 			= ADS1115_REG_CONFIG_DR_32SPS,
	SPS_64 			= ADS1115_REG_CONFIG_DR_64SPS,
	SPS_128 		= ADS1115_REG_CONFIG_DR_128SPS,
	SPS_250 		= ADS1115_REG_CONFIG_DR_250SPS,
	SPS_475 		= ADS1115_REG_CONFIG_DR_475SPS,
	SPS_860 		= ADS1115_REG_CONFIG_DR_860SPS
} adsSPS_t;


class Adafruit_ADS1115
{
	protected:

	uint8_t 	m_i2cAddress;
	uint16_t 	m_conversionDelay;
	adsGain_t 	m_gain;
	adsSPS_t 	m_sps;

	public:

	Adafruit_ADS1115(uint8_t i2cAddress = ADS1115_ADDRESS);
	
	void 		begin(void);
	uint16_t 	readADC_SingleEnded(uint8_t channel);
	int16_t 	readADC_Differential_0_1(void);
	int16_t 	readADC_Differential_2_3(void);
	void 		startComparator_SingleEnded(uint8_t channel, int16_t threshold);
	int16_t 	getLastConversionResults();
	void 		setGain(adsGain_t gain);
	void 		setSPS(adsSPS_t SPS);
	adsGain_t 	getGain(void);

	private:
};


