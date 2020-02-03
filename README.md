[![GoDoc][docimg]][docurl]

[docimg]:      https://godoc.org/github.com/ardnew/mcp2221a?status.svg
[docurl]:      https://godoc.org/github.com/ardnew/mcp2221a

# mcp2221a
Go library for the MCP2221A USB to I²C/UART Protocol Converter with GPIO

Datasheet: [http://ww1.microchip.com/downloads/en/DeviceDoc/20005565B.pdf](http://ww1.microchip.com/downloads/en/DeviceDoc/20005565B.pdf)

###### Features
- [x] Uses [karalabe/hid](https://github.com/karalabe/hid) for USB HID interface
   - Based on cross-platform [HIDAPI](https://github.com/signal11/hidapi) low-level USB library
- [x] Supports multiple MCP2221A devices simultaneously
- [x] Can set all GP pins to any of their dedicated and alternate function operating modes (see below)
- [x] GPIO input/output
- [x] I²C read/write
- [x] ADC read
- [x] DAC write
- [x] Rising/falling edge interrupt detection

The available operating modes for each pin:
```sh
Mode      | GP0       GP1       GP2      GP3
---------   --------- --------- -------- --------
GPIO      | GPIO      GPIO      GPIO     GPIO
Dedicated | SSPND     CLK OUT   USBCFG   LED_I2C
Alt 1     | LED URX   ADC1      ADC2     ADC3
Alt 2     | --        LED UTX   DAC1     DAC2
Alt 3     | --        IOC       --       --
---------   --------- --------- -------- --------
```

Note that UART support is provided natively through the USB interface as a CDC device and is not handled by this library. It should show up in your OS as a regular TTY serial interface (`/dev/tty*` on Linux/macOS, COM on Windows).

###### Installation
Use the builtin `go` package manager:
```sh
go get -u -v github.com/ardnew/mcp2221a
```

See [examples](examples) for some demo applications:
- [examples/gpio/main.go](examples/gpio/main.go) - classic "Blink" demo, toggles an LED connected to GPIO pin GP0
- [examples/i2c/main.go](examples/i2c/main.go) - I2C address scanner, and also reads and prints the "Device ID" register contents from an INA260 power sensor
- [examples/adc/main.go](examples/adc/main.go) - continuously reads and prints an analog value on GPIO pin GP1
- [examples/dac/main.go](examples/dac/main.go) - continuously writes and prints an always-incrementing 5-bit value on GPIO pin GP2
