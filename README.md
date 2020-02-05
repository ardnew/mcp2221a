[![GoDoc][docimg]][docurl]

[docimg]:https://godoc.org/github.com/ardnew/mcp2221a?status.svg
[docurl]:https://godoc.org/github.com/ardnew/mcp2221a

# mcp2221a
Go **[module](https://blog.golang.org/using-go-modules)** for the MCP2221A USB to I²C/UART Protocol Converter with GPIO (**[datasheet](http://ww1.microchip.com/downloads/en/DeviceDoc/20005565B.pdf)**)

## Features
- [x] **[Fully-documented API](https://godoc.org/github.com/ardnew/mcp2221a)**
- [x] Compliant Go module (see below: **[Installation](#installation)**)
- [x] Supports multiple MCP2221A devices simultaneously
- [x] GPIO input/output
   - All dedicated and alternate functions (see below: **[GP operating modes](#gp-operating-modes)**)
- [x] I²C read/write (configurable bit rate, up to 400 kHz)
- [x] ADC read (10-bit, 3 channels/pins), configurable reference voltage
- [x] DAC write (5-bit, 2 pins, shared output), configurable reference voltage and default output
- [x] Dedicated UART/I²C activity LED modes with configurable polarity
- [x] Rising/falling/both edge interrupt detection
- [x] Clock output on 1 pin (up to 12 MHz)
- [x] Save default/power-on configuration to flash memory (GPIO mode/value, DAC output, etc.)

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
#### Where to get one
Adafruit makes a crazy cheap, snazzy breakout with built-in 3.3V regulator (with VBUS/5V and 3.3V output pins), an I²C Qwiic/Stemma QT connector (as well as the regular SDA/SCL pins), and best of all a USB-C connector as its programming interface:
- https://www.adafruit.com/product/4471
- Only _**$6.50 USD**_ (5 Feb 2020) ?!!

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

###### GP pin supported modes
However, only certain pins support each of the operating modes listed above — per the following matrix:
<pre>
                  ╔══════════════╤══════════════╤══════════════╤══════════════╗
                  ║<b>     GP0      </b>│<b>     GP1      </b>│<b>     GP2      </b>│<b>     GP3      </b>║
    ╔═════════════╬══════════════╪══════════════╪══════════════╪══════════════╣
    ║<b>       GPIO  </b>║     GPIO     │     GPIO     │     GPIO     │     GPIO     ║
    ║<b>  Dedicated  </b>║     SSPND    │     CLKR     │    USBCFG    │    LED_I2C   ║
    ║<b>      Alt 1  </b>║    LED_URX   │     ADC1     │     ADC2     │     ADC3     ║
    ║<b>      Alt 2  </b>║      --      │    LED_UTX   │     DAC1     │     DAC2     ║
    ║<b>      Alt 3  </b>║      --      │     IOC      │      --      │      --      ║
    ╚═════════════╩══════════════╧══════════════╧══════════════╧══════════════╝
</pre>
