# mcp2221a
Go library for the MCP2221A USB to I²C/UART Protocol Converter with GPIO

Datasheet: [http://ww1.microchip.com/downloads/en/DeviceDoc/20005565B.pdf](http://ww1.microchip.com/downloads/en/DeviceDoc/20005565B.pdf)

###### Features
- [x] Uses [karalabe/hid](https://github.com/karalabe/hid) for USB HID interface
   - Based on cross-platform [HIDAPI](https://github.com/signal11/hidapi) low-level USB library
- [x] Supports multiple MCP2221A devices simultaneously
- [x] Can set all pins to any of their dedicated and alternate function operation modes
- [x] GPIO input/output
- [x] I²C read/write
- [ ] ADC read
- [ ] DAC write

Note that UART support is provided natively through the USB interface as a CDC device and is not handled by this library. It should show up in your OS as a regular TTY serial interface (`/dev/tty*` on Linux/macOS, COM on Windows).

###### Installation
Use the builtin `go` package manager:
```sh
go get -u -v github.com/ardnew/mcp2221a
```

See [examples](examples) for some demo applications:
- [examples/gpio/main.go](examples/gpio/main.go) - classic "Blink" demo, toggles LED connected to GPIO pin GP0
- [examples/i2c/main.go](examples/i2c/main.go) - I2C address scanner, and also reads the device ID register from a connected INA260
