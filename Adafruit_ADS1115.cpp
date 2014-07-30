/**************************************************************************/
/*!
@file Adafruit_ADS1115.cpp
@author K.Townsend (Adafruit Industries)
@license BSD (see license.txt)

Driver for the ADS1115/ADS1115 ADC

This is a library for the Adafruit MPL115A2 breakout
----> https://www.adafruit.com/products/???

Adafruit invests time and resources providing this open source code,
please support Adafruit and open-source hardware by purchasing
products from Adafruit!

@section HISTORY

v1.0 - First release
v1.1 - add setSPS(adsSPS_t SPS) method by Nipoutch

*/
/**************************************************************************/

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>
#include "Adafruit_ADS1115.h"

/**************************************************************************/
/*!
@brief Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/

static uint8_t i2cread(void) 
{
	#if ARDUINO >= 100
	return Wire.read();
	#else
	return Wire.receive();
	#endif
}

/**************************************************************************/
/*!
@brief Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/

static void i2cwrite(uint8_t x)
{
	#if ARDUINO >= 100
	Wire.write((uint8_t)x);
	#else
	Wire.send(x);
	#endif
}

/**************************************************************************/
/*!
@brief Writes 16-bits to the specified destination register
*/
/**************************************************************************/

static void writeRegister(uint8_t i2cAddress, uint8_t reg, uint16_t value)
{
	Wire.beginTransmission(i2cAddress);
	i2cwrite((uint8_t)reg);
	i2cwrite((uint8_t)(value>>8));
	i2cwrite((uint8_t)(value & 0xFF));
	Wire.endTransmission();
}

/**************************************************************************/
/*!
@brief Writes 16-bits to the specified destination register
*/
/**************************************************************************/

static uint16_t readRegister(uint8_t i2cAddress, uint8_t reg) 
{
	Wire.beginTransmission(i2cAddress);
	i2cwrite(ADS1115_REG_POINTER_CONVERT);
	Wire.endTransmission();
	Wire.requestFrom(i2cAddress, (uint8_t)2);
	return ((i2cread() << 8) | i2cread());
}

/**************************************************************************/
/*!
@brief Instantiates a new ADS1115 class w/appropriate properties
*/
/**************************************************************************/

Adafruit_ADS1115::Adafruit_ADS1115(uint8_t i2cAddress)
{
	m_i2cAddress = i2cAddress;
	m_gain = GAIN_TWOTHIRDS; /* +/- 6.144V range (limited to VDD +0.3V max!) */
	m_sps = SPS_860;
	m_conversionDelay = ADS1115_CONVERSIONDELAY_860;
}

/**************************************************************************/
/*!
@brief Sets up the HW (reads coefficients values, etc.)
*/
/**************************************************************************/

void Adafruit_ADS1115::begin()
{
	Wire.begin();
}

/**************************************************************************/
/*!
@brief Sets the gain and input voltage range
*/
/**************************************************************************/

void Adafruit_ADS1115::setGain(adsGain_t gain)
{
	m_gain = gain;
}

/**************************************************************************/
/*!
@brief Gets a gain and input voltage range
*/
/**************************************************************************/

adsGain_t Adafruit_ADS1115::getGain()
{
	return m_gain;
}

/**************************************************************************/
/*!
@brief Sets the gain and input voltage range
*/
/**************************************************************************/

void Adafruit_ADS1115::setSPS(adsSPS_t SPS)
{
	m_sps = SPS;
	if(m_sps == ADS1115_REG_CONFIG_DR_8SPS)		m_conversionDelay = ADS1115_CONVERSIONDELAY_8;
	if(m_sps == ADS1115_REG_CONFIG_DR_16SPS)	m_conversionDelay = ADS1115_CONVERSIONDELAY_16;
	if(m_sps == ADS1115_REG_CONFIG_DR_32SPS) 	m_conversionDelay = ADS1115_CONVERSIONDELAY_32;
	if(m_sps == ADS1115_REG_CONFIG_DR_64SPS)	m_conversionDelay = ADS1115_CONVERSIONDELAY_64;
	if(m_sps == ADS1115_REG_CONFIG_DR_128SPS)	m_conversionDelay = ADS1115_CONVERSIONDELAY_128;
	if(m_sps == ADS1115_REG_CONFIG_DR_250SPS)	m_conversionDelay = ADS1115_CONVERSIONDELAY_250;
	if(m_sps == ADS1115_REG_CONFIG_DR_475SPS)	m_conversionDelay = ADS1115_CONVERSIONDELAY_475;
	if(m_sps == ADS1115_REG_CONFIG_DR_860SPS)	m_conversionDelay = ADS1115_CONVERSIONDELAY_860;
}

/**************************************************************************/
/*!
@brief Gets a single-ended ADC reading from the specified channel
*/
/**************************************************************************/

uint16_t Adafruit_ADS1115::readADC_SingleEnded(uint8_t channel)
{
	if (channel > 3)
	{
		return 0;
	}
	// Start with default values
	uint16_t config = 	ADS1115_REG_CONFIG_CQUE_NONE | // Disable the comparator (default val)
						ADS1115_REG_CONFIG_CLAT_NONLAT | // Non-latching (default val)
						ADS1115_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low (default val)
						ADS1115_REG_CONFIG_CMODE_TRAD | // Traditional comparator (default val)
						ADS1115_REG_CONFIG_MODE_SINGLE; // Single-shot mode (default)

	// Set PGA/voltage range
	config |= m_gain;
	// Set SPS value 
	config |= m_sps;
	// Set single-ended input channel
	switch (channel)
	{
		case (0):
		config |= ADS1115_REG_CONFIG_MUX_SINGLE_0;
		break;
		case (1):
		config |= ADS1115_REG_CONFIG_MUX_SINGLE_1;
		break;
		case (2):
		config |= ADS1115_REG_CONFIG_MUX_SINGLE_2;
		break;
		case (3):
		config |= ADS1115_REG_CONFIG_MUX_SINGLE_3;
		break;
	}

	// Set 'start single-conversion' bit
	config |= ADS1115_REG_CONFIG_OS_SINGLE;

	// Write config register to the ADC
	writeRegister(m_i2cAddress, ADS1115_REG_POINTER_CONFIG, config);

	// Wait for the conversion to complete (m_conversionDelay linked with SPS value)
	_delay_us(m_conversionDelay);

	// Read the conversion results
	// Shift 12-bit results right 4 bits for the ADS1115
	return readRegister(m_i2cAddress, ADS1115_REG_POINTER_CONVERT);
}

/**************************************************************************/
/*!
@brief Reads the conversion results, measuring the voltage
difference between the P (AIN0) and N (AIN1) input. Generates
a signed value since the difference can be either
positive or negative.
*/
/**************************************************************************/

int16_t Adafruit_ADS1115::readADC_Differential_0_1() {
	// Start with default values
	uint16_t config = 	ADS1115_REG_CONFIG_CQUE_NONE | // Disable the comparator (default val)
						ADS1115_REG_CONFIG_CLAT_NONLAT | // Non-latching (default val)
						ADS1115_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low (default val)
						ADS1115_REG_CONFIG_CMODE_TRAD | // Traditional comparator (default val)
						ADS1115_REG_CONFIG_MODE_SINGLE; // Single-shot mode (default)

	// Set PGA/voltage range
	config |= m_gain;
	config |= m_sps;
	// Set channels
	config |= ADS1115_REG_CONFIG_MUX_DIFF_0_1; // AIN0 = P, AIN1 = N

	// Set 'start single-conversion' bit
	config |= ADS1115_REG_CONFIG_OS_SINGLE;

	// Write config register to the ADC
	writeRegister(m_i2cAddress, ADS1115_REG_POINTER_CONFIG, config);

	// Wait for the conversion to complete (m_conversionDelay linked with SPS value)
	_delay_us(m_conversionDelay);

	// Read the conversion results
	uint16_t res = readRegister(m_i2cAddress, ADS1115_REG_POINTER_CONVERT);
	return (int16_t)res;
}

/**************************************************************************/
/*!
@brief Reads the conversion results, measuring the voltage
difference between the P (AIN2) and N (AIN3) input. Generates
a signed value since the difference can be either
positive or negative.
*/
/**************************************************************************/

int16_t Adafruit_ADS1115::readADC_Differential_2_3() {
	// Start with default values
	uint16_t config = 	ADS1115_REG_CONFIG_CQUE_NONE | // Disable the comparator (default val)
						ADS1115_REG_CONFIG_CLAT_NONLAT | // Non-latching (default val)
						ADS1115_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low (default val)
						ADS1115_REG_CONFIG_CMODE_TRAD | // Traditional comparator (default val)
						ADS1115_REG_CONFIG_MODE_SINGLE; // Single-shot mode (default)


	// Set PGA/voltage range
	config |= m_gain;
	config |= m_sps;

	// Set channels
	config |= ADS1115_REG_CONFIG_MUX_DIFF_2_3; // AIN2 = P, AIN3 = N

	// Set 'start single-conversion' bit
	config |= ADS1115_REG_CONFIG_OS_SINGLE;

	// Write config register to the ADC
	writeRegister(m_i2cAddress, ADS1115_REG_POINTER_CONFIG, config);

	// Wait for the conversion to complete (m_conversionDelay linked with SPS value)
	_delay_us(m_conversionDelay);

	// Read the conversion results
	uint16_t res = readRegister(m_i2cAddress, ADS1115_REG_POINTER_CONVERT);
	return (int16_t)res;
}

/**************************************************************************/
/*!
@brief Sets up the comparator to operate in basic mode, causing the
ALERT/RDY pin to assert (go from high to low) when the ADC
value exceeds the specified threshold.

This will also set the ADC in continuous conversion mode.
*/
/**************************************************************************/

void Adafruit_ADS1115::startComparator_SingleEnded(uint8_t channel, int16_t threshold)
{
	// Start with default values
	uint16_t config = 	ADS1115_REG_CONFIG_CQUE_1CONV | // Comparator enabled and asserts on 1 match
						ADS1115_REG_CONFIG_CLAT_LATCH | // Latching mode
						ADS1115_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low (default val)
						ADS1115_REG_CONFIG_CMODE_TRAD | // Traditional comparator (default val)
						ADS1115_REG_CONFIG_MODE_CONTIN; // Continuous conversion mode

	// Set PGA/voltage range
	config |= m_gain;
	config |= m_sps;

	// Set single-ended input channel
	switch (channel)
	{
		case (0):
		config |= ADS1115_REG_CONFIG_MUX_SINGLE_0;
		break;
		case (1):
		config |= ADS1115_REG_CONFIG_MUX_SINGLE_1;
		break;
		case (2):
		config |= ADS1115_REG_CONFIG_MUX_SINGLE_2;
		break;
		case (3):
		config |= ADS1115_REG_CONFIG_MUX_SINGLE_3;
		break;
	}

	// Set the high threshold register
	// Shift 12-bit results left 4 bits for the ADS1115
	writeRegister(m_i2cAddress, ADS1115_REG_POINTER_HITHRESH, threshold);

	// Write config register to the ADC
	writeRegister(m_i2cAddress, ADS1115_REG_POINTER_CONFIG, config);
}

/**************************************************************************/
/*!
@brief In order to clear the comparator, we need to read the
conversion results. This function reads the last conversion
results without changing the config value.
*/
/**************************************************************************/

int16_t Adafruit_ADS1115::getLastConversionResults()
{
	// Wait for the conversion to complete
	_delay_us(m_conversionDelay);

	// Read the conversion results
	uint16_t res = readRegister(m_i2cAddress, ADS1115_REG_POINTER_CONVERT);
	return (int16_t)res;
}

