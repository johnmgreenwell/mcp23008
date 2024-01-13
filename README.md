# MCP23008 IO-Expander Driver

Portable, multi-instance I2C IO-expander driver.

## Overview

This HAL-mediated custom [MCP23008](https://www.digikey.com/en/products/detail/microchip-technology/MCP23008T-E-SO/739286) IO-expander driver permits ease of use that is portable across many platforms.

## Usage

The implementation relies on an external user-defined hardware abstraction layer (HAL) called `hal.h` which defines the necessary calls in the `HAL` namespace. Namely, a I2C bus object with `init()`, `write()`, and `writeRead()` methods.

This IO-expander driver's `pinMode()` and `portMode()` methods require HAL definitions for values `GPIO_OUTPUT`, `GPIO_INPUT`, and `GPIO_INPUT_PULLUP`, which are used as enumerators. The `init()` method of the HAL I2C bus object should perform any necessary initialization. The `write()` should take two uint8_t values, a register value and a data value, both to be output successively on the bus. The `writeRead()` method takes a uint8_t register value to be written and a uint8_t pointer to the buffer into which one byte of data is read.

### Example

```cpp
#include <mcp23008.h>

...

// Instantiate IO-expander
static const uint8_t          MCP23X08_ADDRESS = 0x20;
static PeripheralIO::MCP23008 i2c_io(MCP23X08_ADDRESS);

...

int main()
{
    uint8_t read_data;

...
    // Init IO-expander
    i2c_io.init();
    i2c_io.portMode(GPIO_INPUT);

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

MIT © 2023 John Greenwell