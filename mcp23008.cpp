//--------------------------------------------------------------------------------------------------------------------
// Name        : mcp23008.cpp
// Purpose     : MCP23008 IO-expander driver class
// Description : This source file implements header file mcp23008.h.
// Language    : C++
// Platform    : Portable
// Framework   : Portable
// Copyright   : MIT License 2024, John Greenwell
//--------------------------------------------------------------------------------------------------------------------

#include "mcp23008.h"

namespace PeripheralIO 
{

MCP23008::MCP23008(uint8_t address, uint8_t i2c_channel)
: _i2c((MCP23008_ADDR | (address & 0x07)), i2c_channel)
{ }

void MCP23008::init(uint32_t baudrate) const
{
    _i2c.init(baudrate);
}

bool MCP23008::pinMode(uint8_t pin, uint8_t mode) const 
{
    uint8_t data = 0;

    if (pin > 7) return false;

    data = i2cRead(MCP23008_IODIR);
    
    if (mode == GPIO_INPUT) 
    { 
        data |= (1 << pin);
    } 
    else if (mode == GPIO_OUTPUT) 
    {
        data &= ~(1 << pin);
    } 
    else if (mode == GPIO_INPUT_PULLUP)
    {
        data |= (1 << pin);
        i2cWrite(MCP23008_GPPU, i2cRead(MCP23008_GPPU) | (1 << pin));
    } 
    else 
    {
        return false;
    }

    i2cWrite(MCP23008_IODIR, data);
    
    return true;
}

void MCP23008::portMode(uint8_t mode) const 
{
    if (mode == GPIO_INPUT) 
    { 
        i2cWrite(MCP23008_GPPU, 0x00);
        i2cWrite(MCP23008_IODIR, 0xFF);
    } 
    else if (mode == GPIO_OUTPUT) 
    {
        i2cWrite(MCP23008_GPPU, 0x00);
        i2cWrite(MCP23008_IODIR, 0x00);
    } 
    else if (mode == GPIO_INPUT_PULLUP)
    {
        i2cWrite(MCP23008_GPPU, 0xFF);
        i2cWrite(MCP23008_IODIR, 0xFF);
    }
}

bool MCP23008::digitalWrite(uint8_t pin, uint8_t val) const 
{
    uint8_t data = 0;

    if (pin > 7) return false;

    data = i2cRead(MCP23008_GPIO);
    data = (val == HIGH) ? (data | 1 << pin) : (data & ~(1 << pin));
    i2cWrite(MCP23008_GPIO, data);

    return true;
}

uint8_t MCP23008::digitalRead(uint8_t pin) const 
{
    if (pin > 7) return 0x00;

    return ((i2cRead(MCP23008_GPIO) >> pin) & 0x01);
}

void MCP23008::write(uint8_t val) const
{
    i2cWrite(MCP23008_GPIO, val);
}

void MCP23008::write(uint8_t reg, uint8_t val) const
{
    i2cWrite(reg, val);
}

uint8_t MCP23008::read() const
{
    return i2cRead(MCP23008_GPIO);
}

uint8_t MCP23008::read(uint8_t reg) const
{
    return i2cRead(reg);
}

/**
 * Private: Hardware I2C Write Function
 */
void MCP23008::i2cWrite(uint8_t reg, uint8_t byte) const 
{
    _i2c.write(reg, byte);
}

/**
 * Private: Hardware I2C Read Function
 */
uint8_t MCP23008::i2cRead(uint8_t reg) const 
{
    uint8_t data = 0;

    _i2c.writeRead(reg, &data);

    return data;
}

}

// EOF
