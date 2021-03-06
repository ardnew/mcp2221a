[![GoDoc][docimg]][docurl]

[docimg]:https://godoc.org/github.com/ardnew/mcp2221a?status.svg
[docurl]:https://godoc.org/github.com/ardnew/mcp2221a

# mcp2221a
Go **[module](https://blog.golang.org/using-go-modules)** for the MCP2221A USB to I²C/UART Protocol Converter with GPIO (**[datasheet](http://ww1.microchip.com/downloads/en/DeviceDoc/20005565B.pdf)**)

## Features
- [x] **[Fully-documented API](https://godoc.org/github.com/ardnew/mcp2221a)**
- [x] Compliant Go module (see below: **[Installation](#installation)**)
- [x] Supports multiple MCP2221A devices simultaneously
   - Define custom USB device VID/PID
- [x] GPIO input/output
   - All dedicated and alternate functions (see below: **[GP operating modes](#gp-operating-modes)**)
- [x] ADC read (10-bit, 3 channels/pins), configurable reference voltage
- [x] DAC write (5-bit, 2 pins, shared output), configurable reference voltage and default output
- [x] Dedicated UART/I²C activity LED modes with configurable polarity
- [x] Rising/falling/both edge interrupt detection
- [x] Clock output on 1 pin (up to 12 MHz)
- [x] Save default/power-on configuration to flash memory (GPIO mode/value, DAC output, etc.)
   - Password-protected flash support
   - Configure minimum required operating current (510 mA max)
- [x] I²C read/write (configurable bit rate, up to 400 kHz)
   - I²C address scanner discovers all slaves on the bus
   - Convenience routines for reading registers from devices with 8-bit and 16-bit subaddressing

Note that **UART support** is provided natively through the USB interface as a CDC device and is not handled by this module. It should show up in your OS as a regular TTY serial interface (`/dev/tty*` on Linux/macOS, COM on Windows).

#### Dependencies
- [github.com/karalabe/hid](https://github.com/karalabe/hid) - USB HID interface

## Installation
If you are not using Go modules (or are unsure), just use the `go` built-in package manager:
```sh
go get -u -v github.com/ardnew/mcp2221a
```

#### Installation using modules
Either use the built-in package manager as above (and drop the `-u`):
```sh
go get -v github.com/ardnew/mcp2221a
```

Or simply add the import statement to your Go package's source code:
```go
import (
  // ... other imports ...
  mcp "github.com/ardnew/mcp2221a"
)
```
The next time you run `go build`, the appropriate package will be downloaded automatically! Gee whiz!

## Examples
See [examples](examples) for some demo applications:
- [GPIO](examples/gpio/main.go) - classic "Blink" demo, toggles an LED connected to GPIO pin GP0
- [I²C](examples/i2c/main.go) - I²C address scanner, and also reads and prints the "Device ID" register contents from an INA260 power sensor
- [ADC](examples/adc/main.go) - continuously reads and prints an analog value on GPIO pin GP1
- [DAC](examples/dac/main.go) - continuously writes and prints an always-incrementing 5-bit value on GPIO pin GP2
- [Flash](examples/flash/main.go) - prints the USB product descriptors from flash memory, and toggles power-up GPIO output value on pin GP0

## Notes
#### GP operating modes
All of the available operating modes for the general-purpose (GP) pins:

- `GPIO`: Operate as a digital input or a digital output pin.
- `SSPND`: Reflects the USB state (`Suspend`/`Resume`); active-low when `Suspend` has been issued by the USB host, and driven high on `Resume`.
   - This lets the application react (e.g enter a low-power mode) when USB communication has been suspended or resumed.
   - The pin value can be inverted (high on `Suspend`, low on `Resume`) using the `Flash` module.
- `USBCFG`: Starts out low during power-up/reset and goes high after successfully enumerating on the USB host.
   - The pin will also go low when in `Suspend` mode and high on `Resume`.
   - The pin value can be inverted (start high, low after enumeration) using the `Flash` module.
- `LED_URX`,`LED_UTX`: Indicates UART (Rx/Tx) data being received/transmitted by pulsing the pin high for a few milliseconds.
   - The pin value can be inverted (pulse low on receive/transmit) using the `Flash` module.
- `LED_I2C`: Indicates I²C (Rx **and** Tx) data being received **and** transmitted by pulsing the pin high for a few milliseconds.
   - The pin value can be inverted (pulse low on receive **and** transmit) using the `Flash` module.
- `CLKR`: Digital output, providing a clock signal derived from the device’s internal clock.
   - The clock’s nominal frequency is 12 MHz ± 0.25%.
   - Other clock values and duty cycles can be configured using the `Flash` module.
- `IOC`: Digital input ("Interrupt-on-Change") that is sensitive to rising, falling, or both edges.
   - The desired edge can be configured using the `Flash` and `SRAM` modules.

###### Supported modes
However, only certain pins support each of the operating modes listed above — per the following matrix yanked from the datasheet:

|             |**GP0**|**GP1**|**GP2**|**GP3**|
|------------:|:-----:|:-----:|:-----:|:-----:|
|  **Default**| GPIO  | GPIO  | GPIO  | GPIO  |
|**Dedicated**| SSPND | CLKR  |USBCFG |LED_I2C|
|    **Alt 1**|LED_URX| ADC1  | ADC2  | ADC3  |
|    **Alt 2**|  --   |LED_UTX| DAC1  | DAC2  |
|    **Alt 3**|  --   |  IOC  |  --   |  --   |

No idea why the first row of alternate functions is named **Dedicated**, that's how they are identified in the datasheet...

#### Datasheet
Please refer to this before sending me a confusing question:
- **[http://ww1.microchip.com/downloads/en/DeviceDoc/20005565B.pdf](http://ww1.microchip.com/downloads/en/DeviceDoc/20005565B.pdf)**

To get started reading this thing, and easier grokking, note that:
   - *The first half* of the datasheet (.pdf) defines internal registers and inter-component behaviors which are all but completely transparent to us. This info can be ignored, but be sure to read the front-matter and any component descriptions.
   - *The second half* of the datasheet (.pdf) defines the USB HID command and response formats. This is the good stuff that pertains to developers of or using this Go module.

#### Where to get one
Adafruit makes a crazy cheap, snazzy breakout with built-in 3.3V regulator (with VBUS/5V and 3.3V output pins), an I²C Qwiic/Stemma QT connector (as well as the regular SDA/SCL pins), and best of all a USB-C connector as its programming interface:
- https://www.adafruit.com/product/4471
- Only _**$6.50 USD**_ (5 Feb 2020), wow!
