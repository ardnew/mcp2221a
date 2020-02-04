[![GoDoc][docimg]][docurl]

[docimg]:https://godoc.org/github.com/ardnew/mcp2221a?status.svg
[docurl]:https://godoc.org/github.com/ardnew/mcp2221a

# mcp2221a
Go [module](https://blog.golang.org/using-go-modules) for the MCP2221A USB to I²C/UART Protocol Converter with GPIO

Datasheet: [http://ww1.microchip.com/downloads/en/DeviceDoc/20005565B.pdf](http://ww1.microchip.com/downloads/en/DeviceDoc/20005565B.pdf)

### Features
- [x] **[Fully-documented API](https://godoc.org/github.com/ardnew/mcp2221a)**
- [x] Compliant Go module (read more about modules [here](https://blog.golang.org/using-go-modules))
- [x] Supports multiple MCP2221A devices simultaneously
- [x] GPIO input/output
   - All dedicated and alternate functions (see GP operating modes below)
- [x] I²C read/write (configurable bit rate, up to 400 kHz)
- [x] ADC read (10-bit, 3 channels/pins), configurable reference voltage
- [x] DAC write (5-bit, 2 pins, shared output), configurable reference voltage and default output
- [x] Dedicated UART/I²C activity LED modes with configurable polarity
- [x] Rising/falling/both edge interrupt detection
- [x] Clock output on 1 pin (up to 12 MHz)
- [x] Save default/power-on configuration to flash memory (GPIO mode/value, DAC output, etc.)

### Dependencies
- [github.com/karalabe/hid](https://github.com/karalabe/hid) - USB HID interface
   
##### GP operating modes
The available operating modes for each GP pin:
```sh
#  Mode      | GP0       GP1       GP2      GP3
#  --------- + ------- - ------- - ------ - -------
#  GPIO      | GPIO      GPIO      GPIO     GPIO
#  Dedicated | SSPND     CLK OUT   USBCFG   LED_I2C
#  Alt 1     | LED URX   ADC1      ADC2     ADC3
#  Alt 2     |           LED UTX   DAC1     DAC2
#  Alt 3     |           IOC                  
#  --------- + ------- - ------- - ------ - -------
```

Note that UART support is provided natively through the USB interface as a CDC device and is not handled by this module. It should show up in your OS as a regular TTY serial interface (`/dev/tty*` on Linux/macOS, COM on Windows).

### Installation
If you are not using Go modules (or are unsure), just use the `go` built-in package manager:
```sh
go get -u -v github.com/ardnew/mcp2221a
```

###### Module
Alternatively, you are using Go modules, either use the built-in package manager (and drop the `-u`):
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
The next time your run `go build` locally on your module, the appropriate package will be downloaded automatically! Gee whiz!

### Examples
See [examples](examples) for some demo applications:
- [GPIO](examples/gpio/main.go) - classic "Blink" demo, toggles an LED connected to GPIO pin GP0
- [I²C](examples/i2c/main.go) - I²C address scanner, and also reads and prints the "Device ID" register contents from an INA260 power sensor
- [ADC](examples/adc/main.go) - continuously reads and prints an analog value on GPIO pin GP1
- [DAC](examples/dac/main.go) - continuously writes and prints an always-incrementing 5-bit value on GPIO pin GP2
- [Flash](examples/flash/main.go) - prints the USB product descriptors from flash memory, and toggles power-up GPIO output value on pin GP0

#### Where to get one
Adafruit makes a breakout with a fancy USB-C connector: https://www.adafruit.com/product/4471
