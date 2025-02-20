# MCP23008 IO-Expander Driver

Portable, multi-instance I2C IO-expander driver.

## Overview

This HAL-mediated custom [MCP23008](https://www.digikey.com/en/products/detail/microchip-technology/MCP23008T-E-SO/739286) IO-expander driver permits ease of use that is designed to be platform-independent.

## Usage

The implementation relies on an external user-defined hardware abstraction layer (HAL) called `hal.h` which defines the necessary calls in the `HAL` namespace. Namely, a I2C bus object with `init()`, `write()`, and `writeRead()` methods.

This IO-expander driver's `pinMode()` and `portMode()` methods require HAL definitions for values `GPIO_OUTPUT`, `GPIO_INPUT`, and `GPIO_INPUT_PULLUP`, which are used as enumerators. The `init()` method of the HAL I2C bus object should perform any necessary initialization. The `write()` should take three uint8_t values: an address, a register value, and a data value be output successively on the bus. The `writeRead()` method takes a uint8_t address, a uint8_t register value to be written and a uint8_t pointer to the buffer into which one byte of data is read. I2C methods should return zero for success and nonzero for error.

### Example

```cpp
#include "mcp23008.h"

...

// Instantiate I2C bus
HAL::I2C i2c_bus;

// Instantiate IO-expander
static const uint8_t   MCP23X08_ADDRESS = 0x20;
PeripheralIO::MCP23008 i2c_io(i2c_bus, MCP23X08_ADDRESS);

...

int main()
{
    uint8_t read_data;

...
    // Init I2C bus
    i2c_bus.init();

    // Init IO-expander, halt if i2c error
    if (!i2c_io.portMode(GPIO_INPUT)) while(true);

...
    // Read IO-expander on condition
    if (someCondition)
    {
        read_data = i2c_io.read();
    }

...

}

...
```

## License

MIT © 2024 John Greenwell