//--------------------------------------------------------------------------------------------------------------------
// Name        : mcp23008.h
// Purpose     : MCP23008 IO-expander driver class
// Description : 
//               This class intended for device driver control of MCP23008 I2C IO-expander chips.
//
//               Interrupts may be setup by applying values to the appropriate registers to desired specification as 
//               described in the datasheet. Improvements to this class might add interrupt methods.
//
// Language    : C++
// Platform    : Portable
// Framework   : Portable
// Copyright   : MIT License 2024, John Greenwell
// Requires    : External : N/A
//               Custom   : hal.h - Custom implementation-defined Hardware Abstraction Layer
//--------------------------------------------------------------------------------------------------------------------
#ifndef _MCP23008_H
#define _MCP23008_H

#include "hal.h"

namespace PeripheralIO
{

// Base address
const uint8_t MCP23008_ADDR    = 0x20;

// Register map
const uint8_t MCP23008_IODIR   = 0x00;
const uint8_t MCP23008_IPOL    = 0x01;
const uint8_t MCP23008_GPINTEN = 0x02;
const uint8_t MCP23008_DEFVAL  = 0x03;
const uint8_t MCP23008_INTCON  = 0x04;
const uint8_t MCP23008_IOCON   = 0x05;
const uint8_t MCP23008_GPPU    = 0x06;
const uint8_t MCP23008_INTF    = 0x07;
const uint8_t MCP23008_INTCAP  = 0x08;
const uint8_t MCP23008_GPIO    = 0x09;
const uint8_t MCP23008_OLAT    = 0x0A;

class MCP23008 
{
    public:

        /**
         * @brief Constructor for MCP23008 object
         * @param address Hardwired address (only the 3 LSB matching device external biasing)
         * @param i2c_channel Identifier for HAL, for when more than one I2C channel is used in parent project
        */
        MCP23008(HAL::I2C& i2c_bus, uint8_t address);

        /**
         * @brief Set IO type on a given pin
         * @param pin Pin 0-7 on which mode is set
         * @param mode Pin mode (GPIO_OUTPUT, GPIO_INPUT, GPIO_INPUT_PULLUP)
         * @return False on invalid arguments or comm failure, true otherwise
        */
        bool pinMode(uint8_t pin, uint8_t mode) const;

        /**
         * @brief Set IO type on entire 8 pins of MCP23008
         * @param mode Pin mode; (GPIO_OUTPUT, GPIO_INPUT, GPIO_INPUT_PULLUP)
         * @return False on invalid arguments or comm failure, true otherwise
        */
        bool portMode(uint8_t mode) const;

        /**
         * @brief Write value to individual pin 0-7. Pin must already be configured as output
         * @param pin Pin 0-7 on which value is set
         * @param val Value to be written; zero for logic low, non-zero for logic high
         * @return False on invalid arguments or comm failure, true otherwise
        */
        bool digitalWrite(uint8_t pin, uint8_t val) const;

        /**
         * @brief Read value from individual pin 0-7; pin must already be configured as input
         * @param pin Pin 0-7 from which value is read
         * @return Value read from pin; zero for logic low, one for logic high
        */
        uint8_t digitalRead(uint8_t pin) const;

        /**
         * @brief Direct write value to entire GPIO port; port must be already configured as output
         * @param val Value to write to GPIO port
         * @return False on invalid arguments or comm failure, true otherwise
        */
        bool write(uint8_t val) const;

        /**
         * @brief Direct write of value to specific MCP23008 internal register; e.g. write(MCP23008_OLAT, 0xFF)
         * @param reg Register number to which value is to be written; e.g. MCP23008_INTCON
         * @param val Value to write to register
         * @return False on invalid arguments or comm failure, true otherwise
        */
        bool write(uint8_t reg, uint8_t val) const;

        /**
         * @brief Direct read value from entire GPIO port; port must already be configured as input
         * @return Value read from GPIO port
        */
        uint8_t read() const;

        /**
         * @brief Direct read of value from specific register; e.g. read(MCP23008_OLAT)
         * @param reg Register number from which value is to be read; e.g. MCP23008_INTCON
         * @return Value read from register
        */
        uint8_t read(uint8_t reg) const;
 
private:
    HAL::I2C& _i2c;
    uint8_t   _i2c_addr;

    uint8_t i2cWrite(uint8_t reg, uint8_t byte) const;
    uint8_t i2cRead(uint8_t reg, uint8_t * data) const;
};

}

#endif // _MCP23008_H

// EOF
