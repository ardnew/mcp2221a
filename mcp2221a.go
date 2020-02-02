// Package mcp2221a provides a high-level interface to the Microchip MCP2221A
// USB to GPIO/I²C/UART protocol converter. The physical GPIO and I²C modules
// are implemented as USB HID-class devices, while the UART module is USB CDC.
// This package only supports the USB HID-class devices (GPIO/I²C) and all of
// the functions associated with them (ADC, DAC, SRAM, and flash memory).
//
// Datasheet: http://ww1.microchip.com/downloads/en/devicedoc/20005565b.pdf
//
// USB HID support provided by: https://github.com/karalabe/hid
package mcp2221a

import (
	"fmt"
	"log"
	"math"
	"time"

	usb "github.com/karalabe/hid"
)

// VID and PID are the official vendor and product identifiers assigned by the
// USB-IF.
const (
	VID = 0x04D8 // 16-bit vendor ID for Microchip Technology Inc.
	PID = 0x00DD // 16-bit product ID for the Microchip MCP2221A.
)

// MsgSz is the size (in bytes) of all command and response messages.
const MsgSz = 64

// ClkHz is the internal clock frequency of the MCP2221A.
const ClkHz = 12000000

// WordSet and WordClr are the logical true and false values for a single word
// (byte) in a message.
const (
	WordSet byte = 0xFF // All bits set
	WordClr byte = 0x00 // All bits clear
)

// makeMsg creates a new zero'd slice with required length of command and
// response messages, both of which are always 64 bytes.
func makeMsg() []byte { return make([]byte, MsgSz) }

// logMsg pretty-prints a given byte slice using the default log object. Each
// element is printed on its own line with the following format in columns of
// uniform-width:
//    IDX: DEC {0xHEX} [0bBIN]
func logMsg(buf []byte) {

	if nil == buf || 0 == len(buf) {
		return
	}

	// calculate the number of digits in the final slice index
	n := int(math.Floor(math.Log10(float64(len(buf)-1)))) + 1
	for i, b := range buf {
		log.Printf("%*d: %3d {0x%02X} [0b%08b]", n, i, b, b, b)
	}

}

// Constants for all recognized commands (and responses). These are sent as the
// first word in all command messages, and are echoed back as the first word in
// all response messages.
const (
	cmdStatus    byte = 0x10
	cmdSetParams byte = 0x10

	cmdFlashRead   byte = 0xB0
	cmdFlashWrite  byte = 0xB1
	cmdFlashPasswd byte = 0xB2

	cmdI2CWrite         byte = 0x90
	cmdI2CWriteRepStart byte = 0x92
	cmdI2CWriteNoStop   byte = 0x94
	cmdI2CRead          byte = 0x91
	cmdI2CReadRepStart  byte = 0x93
	cmdI2CReadGetData   byte = 0x40

	cmdGPIOSet byte = 0x50
	cmdGPIOGet byte = 0x51

	cmdSRAMSet byte = 0x60
	cmdSRAMGet byte = 0x61

	cmdReset byte = 0x70
)

// -----------------------------------------------------------------------------
// -- DEVICE -------------------------------------------------------- [start] --
//

// MCP2221A is the primary object used for interacting with the device.
// The struct contains a pointer to an opened HIDAPI device through which all
// USB communication occurs.
// If multiple MCP2221A devices are connected to the host PC, the index of the
// desired target can be determined with AttachedDevices() and passed to New().
// An index of 0 will use the first device found.
// Call Close() on the device when finished to also close the USB connection.
type MCP2221A struct {
	Device *usb.Device
	Index  byte
	VID    uint16
	PID    uint16
}

// AttachedDevices returns a slice of all connected USB HID device descriptors
// matching the given VID and PID.
// Returns an empty slice if no devices were found. See the hid package
// documentation for details on inspecting the returned objects.
func AttachedDevices(vid uint16, pid uint16) []usb.DeviceInfo {

	var info []usb.DeviceInfo

	for _, i := range usb.Enumerate(vid, pid) {
		info = append(info, i)
	}

	return info
}

// New returns a new MCP2221A object with the given VID and PID, enumerated at
// the given index (an index of 0 will use the first device found).
// Returns an error if index is out of range (according to AttachedDevices()) or
// if the USB HID device could not be claimed or opened.
func New(idx byte, vid uint16, pid uint16) (*MCP2221A, error) {

	mcp := &MCP2221A{
		Device: nil,
		Index:  idx,
		VID:    vid,
		PID:    pid,
	}

	info := AttachedDevices(vid, pid)
	if int(idx) >= len(info) {
		return nil, fmt.Errorf("device index %d out of range [0, %d]", idx, len(info)-1)
	}

	var err error
	if mcp.Device, err = info[idx].Open(); nil != err {
		return nil, err
	}

	return mcp, nil
}

// valid verifies the receiver and USB HID device are both not nil, returning
// false and an error indicating which caused the failure.
func (mcp *MCP2221A) valid() (bool, error) {

	if nil == mcp {
		return false, fmt.Errorf("nil MCP2221A")
	}

	if nil == mcp.Device {
		return false, fmt.Errorf("nil USB HID device")
	}

	return true, nil
}

// Close will clean up any resources and close the USB HID connection.
// Returns an error if the USB HID device is invalid or failed to close
// gracefully.
func (mcp *MCP2221A) Close() error {

	if ok, err := mcp.valid(); !ok {
		return err
	}

	if err := mcp.Device.Close(); nil != err {
		return err
	}
	return nil
}

// send transmits an MCP2221A command message and returns the response message.
// The data argument is a byte slice created by makeMsg(), and the cmd argument
// is one of the recognized command byte constants. The cmd byte is inserted
// into the slice at the appropriate position automatically.
//
// A nil slice is returned with an error if the receiver is invalid or if the
// USB HID device could not be written to or read from.
// If any data was successfully read from the USB HID device, then that data
// slice is returned along with an error if fewer than expected bytes were
// received or if the reserved status byte (common to all response messages)
// does not indicate success.
// A nil slice and nil error are returned if the reset command is received and
// successfully transmitted.
func (mcp *MCP2221A) send(cmd byte, data []byte) ([]byte, error) {

	if ok, err := mcp.valid(); !ok {
		return nil, err
	}

	data[0] = cmd
	if _, err := mcp.Device.Write(data); nil != err {
		return nil, fmt.Errorf("Write([cmd=0x%02X]): %v", cmd, err)
	}

	// logMsg(data)

	if cmdReset == cmd {
		// reset is the only command that does not have a response packet
		return nil, nil
	}

	rsp := makeMsg()
	if recv, err := mcp.Device.Read(rsp); nil != err {
		return nil, fmt.Errorf("Read([cmd=0x%02X]): %v", cmd, err)
	} else {
		if recv < MsgSz {
			return rsp, fmt.Errorf("Read([cmd=0x%02X]): short read (%d of %d bytes)", cmd, recv, MsgSz)
		}
		if rsp[0] != cmd || rsp[1] != WordClr {
			return rsp, fmt.Errorf("Read([cmd=0x%02X]): command failed", cmd)
		}
	}

	return rsp, nil
}

// Reset sends a reset command and then attempts to reopen a connection to the
// same USB HID device within a given timeout duration.
// Returns an error if the receiver is invalid, the reset command could not be
// sent, or if the device could not be reopened before the given timeout period.
func (mcp *MCP2221A) Reset(timeout time.Duration) error {

	if ok, err := mcp.valid(); !ok {
		return err
	}

	cmd := makeMsg()
	cmd[1] = 0xAB
	cmd[2] = 0xCD
	cmd[3] = 0xEF

	if _, err := mcp.send(cmdReset, cmd); nil != err {
		return fmt.Errorf("send(): %v", err)
	}

	ch := make(chan *usb.Device)

	go func(c chan *usb.Device) {
		var m *MCP2221A = nil
		for nil == m {
			m, _ = New(mcp.Index, mcp.VID, mcp.PID)
		}
		c <- m.Device
	}(ch)

	select {
	case <-time.After(timeout):
		return fmt.Errorf("New([%d]): timed out opening USB HID device", mcp.Index)
	case dev := <-ch:
		mcp.Device = dev
	}

	return nil
}

// status contains conveniently-typed fields for all data parsed from the
// response message of a status command.
type status struct {
	cmd        byte
	ok         bool
	i2cCancel  byte
	i2cSpeedCh byte
	i2cDivCh   byte
	i2cState   byte
	i2cReqSz   uint16
	i2cSentSz  uint16
	i2cCounter byte
	i2cDiv     byte
	i2cTimeVal byte
	i2cAddr    uint16
	i2cSCL     byte
	i2cSDA     byte
	interrupt  byte
	i2cReadPnd byte
	hwRevMaj   rune
	hwRevMin   rune
	fwRevMaj   rune
	fwRevMin   rune
	adcCh0     uint16
	adcCh1     uint16
	adcCh2     uint16
}

// newStatus parses the response message of a status command.
// Returns a pointer to a newly-created status object on success, or nil if the
// given response message is nil or has inadequate length.
func newStatus(msg []byte) *status {
	if nil == msg || len(msg) < MsgSz {
		return nil
	}
	return &status{
		cmd:        msg[0],
		ok:         (0 == msg[1]),
		i2cCancel:  msg[2],
		i2cSpeedCh: msg[3],
		i2cDivCh:   msg[4],
		// bytes 5-7 reserved
		i2cState:   msg[8],
		i2cReqSz:   (uint16(msg[10]) << 8) | uint16(msg[9]),
		i2cSentSz:  (uint16(msg[12]) << 8) | uint16(msg[11]),
		i2cCounter: msg[13],
		i2cDiv:     msg[14],
		i2cTimeVal: msg[15],
		i2cAddr:    (uint16(msg[17]) << 8) | uint16(msg[16]),
		// bytes 18-21 reserved
		i2cSCL:     msg[22],
		i2cSDA:     msg[23],
		interrupt:  msg[24],
		i2cReadPnd: msg[25],
		// bytes 26-45 reserved
		hwRevMaj: rune(msg[46]),
		hwRevMin: rune(msg[47]),
		fwRevMaj: rune(msg[48]),
		fwRevMin: rune(msg[49]),
		adcCh0:   (uint16(msg[51]) << 8) | uint16(msg[50]),
		adcCh1:   (uint16(msg[53]) << 8) | uint16(msg[52]),
		adcCh2:   (uint16(msg[55]) << 8) | uint16(msg[54]),
	}
}

// status sends a status command request, parsing the response into an object
// referred to by the return value.
// Returns a pointer to the parsed object on success, or nil along with an error
// if the receiver is invalid or the status command could not be sent.
func (mcp *MCP2221A) status() (*status, error) {

	if ok, err := mcp.valid(); !ok {
		return nil, err
	}

	cmd := makeMsg()
	if rsp, err := mcp.send(cmdStatus, cmd); nil != err {
		return nil, fmt.Errorf("send(): %v", err)
	} else {
		return newStatus(rsp), nil
	}

}

// -- DEVICE ---------------------------------------------------------- [end] --
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// -- SRAM ---------------------------------------------------------- [start] --

// config sends a command requesting current SRAM configuration and returns a
// byte slice within the given interval from the response message.
// Returns a nil slice and error if the receiver is invalid, the given range is
// invalid, or if the configuration command could not be sent.
func (mcp *MCP2221A) config(start byte, stop byte) ([]byte, error) {

	if ok, err := mcp.valid(); !ok {
		return nil, err
	}

	if (start > stop) || (stop >= MsgSz) {
		return nil, fmt.Errorf("invalid byte range: [%d, %d]", start, stop)
	}

	cmd := makeMsg()
	if rsp, err := mcp.send(cmdSRAMGet, cmd); nil != err {
		return nil, fmt.Errorf("send(): %v", err)
	} else {
		return rsp[start : stop+1], nil
	}

}

// -- SRAM ------------------------------------------------------------ [end] --
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// -- GPIO ---------------------------------------------------------- [start] --

// GPIOMode and GPIODir represent two of the configuration parameters for all
// of the general purpose pins.
type (
	GPIOMode byte
	GPIODir  byte
)

// Constants associated with the GPIO module.
const (
	// GPPinCount is the number of GPIO pins available.
	GPPinCount = 4

	// GPIO operation modes:         GP0       GP1       GP2      GP3
	ModeGPIO     GPIOMode = 0x00 //  GPIO      GPIO      GPIO     GPIO
	ModeDediFunc GPIOMode = 0x01 //  SSPND     CLK OUT   USBCFG   LED_I2C
	ModeAltFunc0 GPIOMode = 0x02 //  LED URX   ADC1      ADC2     ADC3
	ModeAltFunc1 GPIOMode = 0x03 //  --        LED UTX   DAC1     DAC2
	ModeAltFunc2 GPIOMode = 0x04 //  --        IOC       --       --
	ModeInvalid  GPIOMode = 0xEE // invalid mode is used as error condition

	// GPIO directions
	DirOut     GPIODir = 0x00 // direction OUT is used for writing values to pins
	DirIn      GPIODir = 0x01 // direction IN is used for reading values from pins
	DirInvalid GPIODir = 0xEF // invalid direction is used as error condition
)

// GPIOSetConfig configures a given pin with a default output value, operation
// mode, and direction.
// Returns an error if the receiver is invalid, the pin index is invalid, the
// current configuration could not be read, or if the new configuration could
// not be sent.
func (mcp *MCP2221A) GPIOSetConfig(pin byte, val byte, mode GPIOMode, dir GPIODir) error {

	if ok, err := mcp.valid(); !ok {
		return err
	}

	if pin >= GPPinCount {
		return fmt.Errorf("invalid GPIO pin: %d", pin)
	}

	cmd := makeMsg()

	if cur, err := mcp.config(22, 25); nil != err {
		return fmt.Errorf("config(): %v", err)
	} else {
		// copy the current GPIO settings because they will -all- be set with the
		// command request
		cmd[7] = WordSet // alter GP designation
		cmd[8] = cur[0]
		cmd[9] = cur[1]
		cmd[10] = cur[2]
		cmd[11] = cur[3]
	}

	// and then update our selected pin as desired
	cmd[8+pin] = (val << 4) | (byte(dir) << 3) | byte(mode)

	if _, err := mcp.send(cmdSRAMSet, cmd); nil != err {
		return fmt.Errorf("send(): %v", err)
	}

	return nil
}

// GPIOGetConfig reads the current default output value, operation mode, and
// direction of a given pin.
// Returns an error if the receiver is invalid, the pin index is invalid, or if
// the current configuration could not be read.
func (mcp *MCP2221A) GPIOGetConfig(pin byte) (byte, GPIOMode, GPIODir, error) {

	if ok, err := mcp.valid(); !ok {
		return WordClr, ModeInvalid, DirInvalid, err
	}

	if pin >= GPPinCount {
		return WordClr, ModeInvalid, DirInvalid, fmt.Errorf("invalid GPIO pin: %d", pin)
	}

	if rsp, err := mcp.config(22, 25); nil != err {
		return WordClr, ModeInvalid, DirInvalid, fmt.Errorf("config(): %v", err)
	} else {
		mode := GPIOMode(rsp[pin] & 0x07)
		dir := GPIODir((rsp[pin] >> 3) & 0x01)
		val := (rsp[pin] >> 4) & 0x01
		return val, mode, dir, nil
	}

}

// GPIOSet sets the digital output value for a given pin.
// Returns an error if the receiver is invalid, the pin index is invalid, or if
// the pin value could not be set (e.g. pin not configured for GPIO operation).
func (mcp *MCP2221A) GPIOSet(pin byte, val byte) error {

	if ok, err := mcp.valid(); !ok {
		return err
	}

	if pin >= GPPinCount {
		return fmt.Errorf("invalid GPIO pin: %d", pin)
	}

	cmd := makeMsg()

	i := 2 + 4*pin
	cmd[i+0] = WordSet // alter output value (to val)
	cmd[i+1] = val
	cmd[i+2] = WordSet // alter GPIO direction (to output)
	cmd[i+3] = byte(DirOut)

	if _, err := mcp.send(cmdGPIOSet, cmd); nil != err {
		return fmt.Errorf("send(): %v", err)
	}

	return nil
}

// GPIOGet gets the current digital value of a given pin.
// Returns an error if the receiver is invalid, the pin index is invalid, or if
// the pin value could not be set (e.g. pin not configured for GPIO operation).
func (mcp *MCP2221A) GPIOGet(pin byte) (byte, error) {

	if ok, err := mcp.valid(); !ok {
		return WordClr, err
	}

	if pin >= GPPinCount {
		return WordClr, fmt.Errorf("invalid GPIO pin: %d", pin)
	}

	cmd := makeMsg()
	if rsp, err := mcp.send(cmdGPIOGet, cmd); nil != err {
		return WordClr, fmt.Errorf("send(): %v", err)
	} else {
		i := 2 + 2*pin
		if byte(ModeInvalid) == rsp[i] {
			return WordClr, fmt.Errorf("pin not in GPIO mode: %d", pin)
		} else {
			return rsp[i], nil
		}
	}

}

// -- GPIO ------------------------------------------------------------ [end] --
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// -- I²C ----------------------------------------------------------- [start] --

// Constants associated with the I²C module.
const (
	I2CBaudRate = 100000 // default baud rate
	I2CMinAddr  = 0x08   // minimum possible 7-bit address
	I2CMaxAddr  = 0x77   // maximum possible (unreserved) 7-bit address

	i2cReadMax  = 60 // maximum number of bytes we can read at a time
	i2cWriteMax = 60 // maximum number of bytes we can write at a time

	// these constants were copied from Adafruit_Blinka mcp2221.py package. some
	// aren't listed anywhere in the datasheet, so not sure where they came from.
	// probably with a fancy protocol analyzer, thus my poor ass is going to just
	// bank on Adafruit doing their homework.
	i2cStateStartTimeout    byte = 0x12
	i2cStateRepStartTimeout byte = 0x17
	i2cStateStopTimeout     byte = 0x62

	i2cStateAddrSend    byte = 0x21
	i2cStateAddrTimeout byte = 0x23
	i2cStateAddrNACK    byte = 0x25

	i2cMaskAddrNACK byte = 0x40

	i2cStatePartialData   byte = 0x41
	i2cStateReadMore      byte = 0x43
	i2cStateWriteTimeout  byte = 0x44
	i2cStateWritingNoStop byte = 0x45
	i2cStateReadTimeout   byte = 0x52
	i2cStateReadPartial   byte = 0x54
	i2cStateReadComplete  byte = 0x55

	i2cStateReadError byte = 0x7F

	i2cReadRetry  = 50 // maximum number of retries permitted for a single read
	i2cWriteRetry = 50 // maximum number of retries permitted for a single write
)

// I2CSetConfig configures the I²C bus clock divider calculated from a given
// baud rate (BPS). If in doubt, use global constant I2CBaudRate.
// Returns an error if the receiver is invalid, the given baud rate is invalid,
// the set-parameters command could not be sent, or if an I²C transfer is
// currently in-progress.
func (mcp *MCP2221A) I2CSetConfig(baud uint32) error {

	if ok, err := mcp.valid(); !ok {
		return err
	}

	if baud > ClkHz/3 || baud < ClkHz/258 {
		return fmt.Errorf("invalid baud rate: %d", baud)
	}

	// again, this calculation was shamelessly stolen from Adafruit's Blinka
	// package as I have no idea how they determined these numbers (wtf @ -3).
	cmd := makeMsg()
	cmd[3] = 0x20
	cmd[4] = byte(ClkHz/baud - 3)

	if rsp, err := mcp.send(cmdSetParams, cmd); nil != err {
		return fmt.Errorf("send(): %v", err)
	} else {
		stat := newStatus(rsp)
		if 0x21 == stat.i2cSpeedCh {
			return fmt.Errorf("transfer in progress")
		}
	}

	return nil
}

// I2CCancel sends a set-parameters command to cancel any ongoing I²C transfer
// currently in progress.
// Returns an error if the receiver is invalid, or if the command could not be
// sent.
func (mcp *MCP2221A) I2CCancel() error {

	if ok, err := mcp.valid(); !ok {
		return err
	}

	cmd := makeMsg()
	cmd[2] = 0x10

	if rsp, err := mcp.send(cmdSetParams, cmd); nil != err {
		return fmt.Errorf("send(): %v", err)
	} else {
		stat := newStatus(rsp)
		if 0x10 == stat.i2cCancel {
			time.Sleep(300 * time.Microsecond)
		}
	}

	return nil
}

// I2CWrite is the general-purpose function for writing raw data directly to the
// I²C data bus. If argument stop is true, then an I²C STOP condition is
// generated on the bus once the bytes transmitted equals the number bytes
// specified as parameter cnt (this is the "usual" case). Otherwise, the STOP
// condition is not generated, and the bus remains "active" for subsequent I/O.
//
// Returns an error if any of the following occur: invalid receiver, could not
// read status message, could not cancel an existing I²C connection (if exists),
// could not send command message, the I²C state machine enters an unrecoverable
// state, or too many retries were attempted.
func (mcp *MCP2221A) I2CWrite(stop bool, addr uint8, out []byte, cnt uint16) error {

	if ok, err := mcp.valid(); !ok {
		return err
	}

	if cnt <= 0 {
		return nil
	}

	if int(cnt) > len(out) {
		cnt = uint16(len(out))
	}

	var (
		stat *status
		err  error
	)

	if stat, err = mcp.status(); nil != err {
		return fmt.Errorf("status(): %v", err)
	}

	if WordClr != stat.i2cState {
		if err := mcp.I2CCancel(); nil != err {
			return fmt.Errorf("I2CCancel(): %v", err)
		}
	}

	unrecoverable := func(b byte) bool {
		return b == i2cStateStartTimeout ||
			b == i2cStateStopTimeout ||
			b == i2cStateAddrTimeout ||
			b == i2cStateAddrNACK ||
			b == i2cStateWriteTimeout
	}

	cmdID := cmdI2CWrite
	if !stop {
		cmdID = cmdI2CWriteNoStop
	}

	pos := uint16(0)
	retry := 0
	for pos < cnt {

		sz := cnt - pos
		if sz > i2cWriteMax {
			sz = i2cWriteMax
		}

		cmd := makeMsg()
		cmd[1] = byte(cnt & 0xFF)
		cmd[2] = byte((cnt >> 8) & 0xFF)
		cmd[3] = byte(addr << 1)

		sendCMD := cmdID
		if pos+sz > cnt {
			sendCMD = cmdI2CWriteNoStop
		}

		copy(cmd[4:], out[pos:pos+sz])

		retry := 0
		for retry < i2cWriteRetry {

			if rsp, err := mcp.send(sendCMD, cmd); nil != err {
				if nil != rsp {
					if unrecoverable(rsp[2]) {
						return fmt.Errorf("send(): unrecoverable I²C write error")
					}
				} else {
					return fmt.Errorf("send(): %v", err)
				}
				time.Sleep(300 * time.Microsecond)
			} else {

				partial := true
				for partial {
					if stat, err := mcp.status(); nil != err {
						return fmt.Errorf("status(): %v", err)
					} else {
						partial = i2cStatePartialData == stat.i2cState
					}
				}

				pos += sz
				break
			}

			retry++
		}

		if retry >= i2cWriteRetry {
			return fmt.Errorf("too many retries")
		}

	}

	retry = 0
	for retry < i2cWriteRetry {

		if stat, err := mcp.status(); nil != err {
			return fmt.Errorf("status(): %v", err)
		} else {
			if WordClr == stat.i2cState {
				break
			}
			if cmdI2CWriteNoStop == cmdID && i2cStateWritingNoStop == stat.i2cState {
				break
			}
			if unrecoverable(stat.i2cState) {
				return fmt.Errorf("send(): unrecoverable I²C write error")
			}
			time.Sleep(300 * time.Microsecond)
		}

	}

	return nil
}

// I2CRead is the general purpose function for performing I²C read operations.
// If argument rep is true, a REP-START condition is generated (instead of the
// usual START condition) to indicate we are reading data from an address
// configured before this call to I2CRead().
//
// Returns the data slice of length cnt (bytes) read from the bus if there was
// no error. Otherwise, an error is returned if any of the following occur:
// invalid receiver, could not read status message, could not cancel an existing
// I²C connection (if exists), could not send command message, the I²C state
// machine enters an unrecoverable state.
func (mcp *MCP2221A) I2CRead(rep bool, addr uint8, cnt uint16) ([]byte, error) {

	if ok, err := mcp.valid(); !ok {
		return nil, err
	}

	if cnt <= 0 {
		return []byte{}, nil
	}

	var (
		stat *status
		err  error
	)

	if stat, err = mcp.status(); nil != err {
		return nil, fmt.Errorf("status(): %v", err)
	}

	if WordClr != stat.i2cState && i2cStateWritingNoStop != stat.i2cState {
		if err := mcp.I2CCancel(); nil != err {
			return nil, fmt.Errorf("I2CCancel(): %v", err)
		}
	}

	cmd := makeMsg()
	cmd[1] = byte(cnt & 0xFF)
	cmd[2] = byte((cnt >> 8) & 0xFF)
	cmd[3] = byte((addr << 1) | 0x01)

	cmdID := cmdI2CRead
	if rep {
		cmdID = cmdI2CReadRepStart
	}

	if _, err := mcp.send(cmdID, cmd); nil != err {
		return nil, fmt.Errorf("send(): %v", err)
	}

	in := make([]byte, cnt)

	pos := uint16(0)
	for pos < cnt {

		var (
			rsp []byte
			err error
		)

		retry := 0
		for retry < i2cReadRetry {

			retry++

			cmd := makeMsg()
			if rsp, err = mcp.send(cmdI2CReadGetData, cmd); nil != err {
				return nil, fmt.Errorf("send(): %v", err)
			} else {

				if i2cStatePartialData == rsp[1] || i2cStateReadError == rsp[3] {
					time.Sleep(300 * time.Microsecond)
					continue
				}
				if i2cStateAddrNACK == rsp[2] {
					return nil, fmt.Errorf("send(): NACK from address (0x%02X)", addr)
				}
				if WordClr == rsp[2] && 0 == rsp[3] {
					break
				}
				if i2cStateReadPartial == rsp[2] || i2cStateReadComplete == rsp[2] {
					break
				}

			}

		}

		if retry >= i2cReadRetry {
			return nil, fmt.Errorf("too many retries")
		}

		if len(rsp) > 0 {
			sz := cnt - pos
			if sz > i2cReadMax {
				sz = i2cReadMax
			}
			copy(in[pos:], rsp[4:4+sz])
			pos += sz
		}
	}

	return in, nil
}

// I2CReadReg performs a standard write-then-read I²C operation as a convenience
// for the common case of reading registers. This variant is for target devices
// with 8-bit subaddress widths (i.e. the size of the register pointer).
// Returns the bytes received on success, or return error if either write or
// read failures occurred.
//
// Also see I2CReadReg16() for 16-bit subaddressing devices.
func (mcp *MCP2221A) I2CReadReg(addr uint8, reg uint8, cnt uint16) ([]byte, error) {

	if err := mcp.I2CWrite(false, addr, []byte{reg}, 1); nil != err {
		return nil, fmt.Errorf("I2CWrite([0x%02X]): %v", addr, err)
	}

	if reg, err := mcp.I2CRead(true, addr, cnt); nil != err {
		return nil, fmt.Errorf("I2CRead([0x%02X]): %v", addr, err)
	} else {
		return reg, nil
	}

}

// I2CReadReg16 performs a standard write-then-read I²C operation as a
// convenience for the common case of reading registers. This variant is for
// target devices with 16-bit subaddress widths (i.e. the si ze of the register
// pointer). If argument msb is true, then the buffer containing the register
// pointer is reversed so that the MSByte is at buffer index 0.
// Returns the bytes received on success, or return error if either write or
// read failures occurred.
//
// Also see I2CReadReg() for 16-bit subaddressing devices.
func (mcp *MCP2221A) I2CReadReg16(addr uint8, reg uint16, msb bool, cnt uint16) ([]byte, error) {

	buf := []byte{byte(reg & 0xFF), byte((reg >> 8) & 0xFF)}
	if msb {
		buf = []byte{buf[1], buf[0]}
	}

	if err := mcp.I2CWrite(false, addr, buf, 2); nil != err {
		return nil, fmt.Errorf("I2CWrite([0x%02X]): %v", addr, err)
	}

	if reg, err := mcp.I2CRead(true, addr, cnt); nil != err {
		return nil, fmt.Errorf("I2CRead([0x%02X]): %v", addr, err)
	} else {
		return reg, nil
	}

}

// I2CScan scans a given address range and attempts to communicate with each
// device, ignoring any failures caused by non-existent targets.
// Returns a byte slicie of 7-bit addresses known to be online and able to be
// communicated with.
// Returns a nil slice and error if the receiver is invalid or given address
// range is invalid.
func (mcp *MCP2221A) I2CScan(start uint8, stop uint8) ([]uint8, error) {

	if ok, err := mcp.valid(); !ok {
		return nil, err
	}

	if start > stop {
		return nil, fmt.Errorf("invalid address range [%d, %d]", start, stop)
	}

	found := []byte{}
	for addr := start; addr <= stop; addr++ {
		if err := mcp.I2CWrite(true, addr, []byte{0x00}, 1); nil == err {
			found = append(found, byte(addr))
		}
	}

	return found, nil
}

// -- I²C ------------------------------------------------------------- [end] --
// -----------------------------------------------------------------------------
