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

MCP23008::MCP23008(HAL::I2C& i2c_bus, uint8_t address)
: _i2c(i2c_bus)
, _i2c_addr((MCP23008_ADDR | (address & 0x07)))
{ }

bool MCP23008::pinMode(uint8_t pin, uint8_t mode) const 
{
    uint8_t data = 0;

    if (pin > 7) return false;

    if (0 != i2cRead(MCP23008_IODIR, &data)) return false;
    
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
        uint8_t tmp;
        if (0 != i2cRead(MCP23008_GPPU, &tmp)) return false;
        tmp |= (1 << pin);
        if (0 != i2cWrite(MCP23008_GPPU, tmp)) return false;
    } 
    else 
    {
        return false;
    }

    if (0 != i2cWrite(MCP23008_IODIR, data)) return false;
    
    return true;
}

bool MCP23008::portMode(uint8_t mode) const
{
    if (mode == GPIO_INPUT) 
    { 
        if (0 != i2cWrite(MCP23008_GPPU, 0x00)) return false;
        if (0 != i2cWrite(MCP23008_IODIR, 0xFF)) return false;
    } 
    else if (mode == GPIO_OUTPUT) 
    {
        if (0 != i2cWrite(MCP23008_GPPU, 0x00)) return false;
        if (0 != i2cWrite(MCP23008_IODIR, 0x00)) return false;
    } 
    else if (mode == GPIO_INPUT_PULLUP)
    {
        if (0 != i2cWrite(MCP23008_GPPU, 0xFF)) return false;
        if (0 != i2cWrite(MCP23008_IODIR, 0xFF)) return false;
    }

    return true;
}

bool MCP23008::digitalWrite(uint8_t pin, uint8_t val) const 
{
    uint8_t data = 0;

    if (pin > 7) return false;

    if (0 != i2cRead(MCP23008_GPIO, &data)) return false;
    data = (val == HIGH) ? (data | 1 << pin) : (data & ~(1 << pin));
    if (0 != i2cWrite(MCP23008_GPIO, data)) return false;

    return true;
}

uint8_t MCP23008::digitalRead(uint8_t pin) const 
{
    uint8_t data;

    if (pin > 7) return 0x00;

    if (0 != i2cRead(MCP23008_GPIO, &data)) return 0xFF;
    data = (data >> pin) & 0x01;

    return data;
}

bool MCP23008::write(uint8_t val) const
{
    return i2cWrite(MCP23008_GPIO, val);
}

bool MCP23008::write(uint8_t reg, uint8_t val) const
{
    return i2cWrite(reg, val);
}

uint8_t MCP23008::read() const
{
    return read(MCP23008_GPIO);
}

uint8_t MCP23008::read(uint8_t reg) const
{
    uint8_t data;

    if (0 != i2cRead(reg, &data)) return 0xFF;

    return data;
}

/**
 * Private: Hardware I2C Write Function
 */
uint8_t MCP23008::i2cWrite(uint8_t reg, uint8_t byte) const
{
    return _i2c.write(_i2c_addr, reg, byte);
}

/**
 * Private: Hardware I2C Read Function
 */
uint8_t MCP23008::i2cRead(uint8_t reg, uint8_t * data) const
{
    return _i2c.writeRead(_i2c_addr, reg, data);
}

}

// EOF
