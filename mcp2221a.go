// Package mcp2221a provides a high-level interface to the Microchip MCP2221A
// USB to GPIO/I²C/UART protocol converter. The physical GPIO and I²C modules
// are implemented as USB HID-class devices, while the UART module is USB CDC.
// This package only supports the USB HID-class devices (GPIO/I²C) and all of
// the functions associated with them (ADC, DAC, SRAM, and flash memory).
//
// Datasheet: http://ww1.microchip.com/downloads/en/devicedoc/20005565b.pdf
package mcp2221a

import (
	"fmt"
	"log"
	"math"
	"time"
	"unicode/utf16"

	usb "github.com/karalabe/hid"
)

// Constants associated with package version.
const (
	VersionPkg = "mcp2221a"
	VersionMaj = 0
	VersionMin = 3
	VersionPch = 0
)

// Version returns the SemVer-compatible version string of this package.
func Version() string {
	return fmt.Sprintf("%d.%d.%d", VersionMaj, VersionMin, VersionPch)
}

// PackageVersion returns the descriptive version string of this package.
func PackageVersion() string {
	return fmt.Sprintf("%s v%s", VersionPkg, Version())
}

// VID and PID are the official vendor and product identifiers assigned by the
// USB-IF.
const (
	//VID = 0x04D8 // 16-bit vendor ID for Microchip Technology Inc.
	//PID = 0x00DD // 16-bit product ID for the Microchip MCP2221A.
	VID = 0x6f88 // custom-defined 16-bit vendor ID
	PID = 0x04d8 // custom-defined 16-bit product ID
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
func LogMsg(buf []byte) {
	if nil == buf || 0 == len(buf) {
		return
	}
	// calculate the number of digits in the final slice index
	n := int(math.Floor(math.Log10(float64(len(buf)-1)))) + 1
	for i, b := range buf {
		log.Printf("%*d: %3d {0x%02X} [0b%08b]", n, i, b, b, b)
	}
}

// VRef represents one of the enumerated constants that can be used as reference
// voltage for both the ADC and DAC modules.
type VRef byte

// Constants for enumerated reference voltage values used by the settings
// structure read from and written to SRAM and flash.
const (
	VRefDefault VRef = 0x00 // Default (Vdd)
	VRefVdd     VRef = 0x00 // Vdd
	VRef4p096   VRef = 0x07 // 4.096 V
	VRef2p048   VRef = 0x05 // 2.048 V
	VRef1p024   VRef = 0x03 // 1.024 V
	VRefOff     VRef = 0x01 // reference voltage disabled
)

// isVRefValid verifies the given VRef v is one of the recognized enumerated
// reference voltage values.
func isVRefValid(v VRef) bool {
	return (0 == v) || ((0x1 == (v & 0x1)) && (v < 0x08))
}

// chanADC maps GPIO pin (0-3) to its respective ADC channel (0-2). if the pin
// does not have an associated ADC channel, the key is not defined.
var chanADC = map[byte]byte{1: 0, 2: 1, 3: 2}

// outpDAC maps GPIO pin (0-3) to its respective DAC output (0-1). if the pin
// does not have an associated DAC output, the key is not defined.
var outpDAC = map[byte]byte{2: 0, 3: 1}

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

// MCP2221A is the primary object used for interacting with the device and all
// of its modules.
// The struct contains a pointer to an opened HIDAPI device through which all
// USB communication occurs. However, the HIDAPI device should not be used
// directly, as communication should be performed through one of the respective
// on-chip modules.
// If multiple MCP2221A devices are connected to the host PC, the index of the
// desired target can be determined with AttachedDevices() and passed to New().
// An index of 0 will use the first device found.
// Pointers to structs representing each of the chip's modules are exported for
// interacting with that component of the device.
// Call Close() on the device when finished to also close the USB connection.
type MCP2221A struct {
	Device *usb.Device
	Index  byte
	VID    uint16
	PID    uint16

	// Locked indicates if the device is permanently locked, preventing
	// write-access to the flash memory module.
<<<<<<< HEAD
	// It is not possible in any way to unlock the device if this is set. :(
	locked bool

	// configuration storage interfaces are not exported because incorrect usage
	// can leave the device in an unusable state, and exposing their methods
	// confuses or pollutes the API by offering multiple interfaces through which
	// configuration can be performed. therefore, only the configuration methods
	// explicitly exported by the individual components may be used. simple.
	sram  *sram  // volatile active settings, not restored on startup/reset
	flash *flash // non-volatile inactive settings, restored on startup/reset
=======
	// It is not possible in any way to unlock the device if this is set.
	locked bool
>>>>>>> 5cba9e80972c66338ae05cfe80036620cd24db80

	// each of the on-chip modules acting as primary functional interfaces.
	GPIO *GPIO // 4x GPIO pins, each also have unique special functions
	ADC  *ADC  // 3x 10-bit analog-to-digital converter
	DAC  *DAC  // 1x 5-bit digital-to-analog converter (avail on 2 pins)
	Alt  *Alt  // special-purpose GP alternate/dedicated functions
	I2C  *I2C  // dedicated I²C SDA/SCL pins, up to 400 kHz
}

// AttachedDevices returns a slice of all connected USB HID device descriptors
// matching the given VID and PID. The order of devices in the returned slice
// will always be the same, so the index of the devices in this slice can be
// used as index parameter to New() and openUSBDevice().
//
// Returns an empty slice if no devices were found. See the hid package
// documentation for details on inspecting the returned objects.
func AttachedDevices(vid uint16, pid uint16) []usb.DeviceInfo {

	var info []usb.DeviceInfo

	for _, i := range usb.Enumerate(vid, pid) {
		info = append(info, i)
	}

	return info
}

// openUSBDevice returns an opened USB HID device descriptor with the given vid
// and pid and which enumerates itself on the USB host at index idx (an index of
// 0 will use the first device found).
//
// Returns nil and an error if the given index is out of range (including when
// no devices matching vid/pid were found), or if the USB HID device could not
// be claimed or opened.
func openUSBDevice(idx byte, vid uint16, pid uint16) (*usb.Device, error) {

	info := AttachedDevices(vid, pid)
	if int(idx) >= len(info) {
		return nil, fmt.Errorf("device index %d out of range [0, %d]", idx, len(info)-1)
	}

	var (
		dev *usb.Device
		err error
	)

	if dev, err = info[idx].Open(); nil != err {
		return nil, fmt.Errorf("Open(): %v", err)
	}

	return dev, nil
}

// New returns a new MCP2221A object with the given VID and PID, enumerated at
// the given index (an index of 0 will use the first device found).
//
// Returns an error if index is out of range (according to AttachedDevices()) or
// if the USB HID device could not be claimed or opened.
func New(idx byte, vid uint16, pid uint16) (*MCP2221A, error) {

	var (
		dev *usb.Device
		err error
	)

	if dev, err = openUSBDevice(idx, vid, pid); nil != err {
		return nil, fmt.Errorf("openUSBDevice(): %v", err)
	}

	mcp := &MCP2221A{
		Device: dev,
		Index:  idx,
		VID:    vid,
		PID:    pid,
	}

	// each module embeds the common *MCP2221A instance so that the modules can
	// refer to each others' functions.
<<<<<<< HEAD
	mcp.sram, mcp.GPIO, mcp.ADC, mcp.DAC, mcp.I2C =
		&sram{mcp}, &GPIO{mcp}, &ADC{mcp}, &DAC{mcp}, &I2C{mcp}

	// configure the write-access flag as disabled by default until we've read the
	// actual settings from flash memory.
	mcp.flash = &flash{mcp, false}
=======
	mcp.SRAM, mcp.GPIO, mcp.ADC, mcp.DAC, mcp.I2C =
		&SRAM{mcp}, &GPIO{mcp}, &ADC{mcp}, &DAC{mcp}, &I2C{mcp}

	// configure the write-access flag as disabled by default until we've read the
	// actual settings from flash memory.
	mcp.Flash = &Flash{mcp, false}
>>>>>>> 5cba9e80972c66338ae05cfe80036620cd24db80

	// the Alt struct embeds the common *MCP2221A instance, but also has several
	// other fields that need the same reference.
	mcp.Alt = &Alt{
		MCP2221A: mcp,
		// each of the special-purpose modules also embeds the common *MCP2221A.
		SUSPND: &SUSPND{mcp},
		CLKOUT: &CLKOUT{mcp},
		USBCFG: &USBCFG{mcp},
		INTCHG: &INTCHG{mcp},
		LEDI2C: &LEDI2C{mcp},
		LEDURX: &LEDURX{mcp},
		LEDUTX: &LEDUTX{mcp},
	}

	// initialize the device locked flag and flash write-access flag based on the
	// chip security settings stored in flash memory.
<<<<<<< HEAD
	if sec, err := mcp.flash.chipSecurity(); nil != err {
		return nil, fmt.Errorf("flash.chipSecurity(): %v", err)
	} else {
		mcp.locked, mcp.flash.writeable = unlockFlags(sec)
=======
	if sec, err := mcp.Flash.chipSecurity(); nil != err {
		return nil, fmt.Errorf("Flash.chipSecurity(): %v", err)
	} else {
		mcp.locked, mcp.Flash.writeable = unlockFlags(sec)
>>>>>>> 5cba9e80972c66338ae05cfe80036620cd24db80
	}

	return mcp, nil
}

// valid verifies the receiver and USB HID device are both not nil.
//
// Returns false with a descriptive error if any required field is nil.
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
//
// Returns an error if the USB HID device is invalid or failed to close
// gracefully.
func (mcp *MCP2221A) Close() error {

	if ok, err := mcp.valid(); !ok {
		return err
	}

<<<<<<< HEAD
	mcp.locked, mcp.flash.writeable = true, false
=======
	mcp.locked, mcp.Flash.writeable = true, false
>>>>>>> 5cba9e80972c66338ae05cfe80036620cd24db80

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
//
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
		var d *usb.Device = nil
		for nil == d {
			d, _ = openUSBDevice(mcp.Index, mcp.VID, mcp.PID)
		}
		c <- d
	}(ch)

	select {
	case <-time.After(timeout):
		return fmt.Errorf("New([%d]): timed out opening USB HID device", mcp.Index)
	case dev := <-ch:
		mcp.Device = dev
	}

	// initialize the device locked flag and flash write-access flag based on the
	// chip security settings stored in flash memory.
<<<<<<< HEAD
	if sec, err := mcp.flash.chipSecurity(); nil != err {
		return fmt.Errorf("flash.chipSecurity(): %v", err)
	} else {
		mcp.locked, mcp.flash.writeable = unlockFlags(sec)
=======
	if sec, err := mcp.Flash.chipSecurity(); nil != err {
		return fmt.Errorf("Flash.chipSecurity(): %v", err)
	} else {
		mcp.locked, mcp.Flash.writeable = unlockFlags(sec)
>>>>>>> 5cba9e80972c66338ae05cfe80036620cd24db80
	}

	return nil
}

// status contains conveniently-typed fields for all data parsed from the
// response message of a status command.
type status struct {
	cmd        byte
	ok         bool
	i2cCancel  byte
	i2cSpdChg  byte
	i2cClkChg  byte
	i2cState   byte
	i2cReqSz   uint16
	i2cSentSz  uint16
	i2cCounter byte
	i2cClkDiv  byte
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
	adcChan    []uint16
}

// parseStatus parses the response message of the status command.
//
// Returns a pointer to a newly-created status object on success, or nil if the
// given response message is nil or has inadequate length.
func parseStatus(msg []byte) *status {
	if nil == msg || len(msg) < MsgSz {
		return nil
	}
	return &status{
		cmd:       msg[0],
		ok:        (0 == msg[1]),
		i2cCancel: msg[2],
		i2cSpdChg: msg[3],
		i2cClkChg: msg[4],
		// bytes 5-7 reserved
		i2cState:   msg[8],
		i2cReqSz:   (uint16(msg[10]) << 8) | uint16(msg[9]),
		i2cSentSz:  (uint16(msg[12]) << 8) | uint16(msg[11]),
		i2cCounter: msg[13],
		i2cClkDiv:  msg[14],
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
		adcChan: []uint16{
			(uint16(msg[51]) << 8) | uint16(msg[50]), // channel 0 (pin GP1)
			(uint16(msg[53]) << 8) | uint16(msg[52]), // channel 1 (pin GP2)
			(uint16(msg[55]) << 8) | uint16(msg[54]), // channel 2 (pin GP3)
		},
	}
}

// status sends a status command request, parsing the response into an object
// referred to by the return value.
// We don't ever need the actual bytes from this response message to build a
// cmdSetParams command, because these fields have "alter bits", which means it
// can ignore any fields we aren't explicitly modifying.
//
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
		return parseStatus(rsp), nil
	}
}

<<<<<<< HEAD
// USBManufacturer reads the current USB manufacturer description from flash
// memory and returns it as a string.
//
// Returns an empty string and error if the receiver is invalid or if the flash
// configuration could not be read.
func (mcp *MCP2221A) USBManufacturer() (string, error) {

	if ok, err := mcp.valid(); !ok {
		return "", err
	}

	if rsp, err := mcp.flash.read(subcmdUSBMfgDesc); nil != err {
		return "", fmt.Errorf("read(): %v", err)
	} else {
		return parseFlashString(rsp), nil
	}
}

// USBProduct reads the current USB product description from flash memory and
// returns it as a string.
//
// Returns an empty string and error if the receiver is invalid or if the flash
// configuration could not be read.
func (mcp *MCP2221A) USBProduct() (string, error) {

	if ok, err := mcp.valid(); !ok {
		return "", err
	}

	if rsp, err := mcp.flash.read(subcmdUSBProdDesc); nil != err {
		return "", fmt.Errorf("read(): %v", err)
	} else {
		return parseFlashString(rsp), nil
	}
}

// USBSerialNo reads the current USB serial number from flash memory and returns
// it as a string.
//
// Returns an empty string and error if the receiver is invalid or if the flash
// configuration could not be read.
func (mcp *MCP2221A) USBSerialNo() (string, error) {

	if ok, err := mcp.valid(); !ok {
		return "", err
	}

	if rsp, err := mcp.flash.read(subcmdUSBSerialNo); nil != err {
		return "", fmt.Errorf("read(): %v", err)
	} else {
		return parseFlashString(rsp), nil
	}
}

// FactorySerialNo reads the factory serial number (read-only) from flash memory
// and returns it as a string.
//
// Returns an empty string and error if the receiver is invalid or if the flash
// configuration could not be read.
func (mcp *MCP2221A) FactorySerialNo() (string, error) {

	if ok, err := mcp.valid(); !ok {
		return "", err
	}

	if rsp, err := mcp.flash.read(subcmdSerialNo); nil != err {
		return "", fmt.Errorf("read(): %v", err)
	} else {
		cnt := rsp[2] - 2
		if cnt > MsgSz-4 {
			cnt = MsgSz - 4
		}
		return string(rsp[4 : 4+cnt]), nil
	}
}

// ConfigVIDPID changes the device's VID and PID written in flash memory.
// These settings are non-volatile and become the actual VID and PID with which
// the device will enumerate itself on the USB host (i.e., if changed, the
// global VID and PID constants defined in this package cannot be used to open
// the device) on the next reset/startup.
// Therefore, if changed, be sure your system settings are updated to permit
// access to the device, since it will appear to be a new USB HID device.
//
// For instance, on some udev-based Linux systems, you may need to update your
// udev rules to grant read-write access to your user for devices matching the
// new VID/PID (these rules are traditionally kept in /etc/udev/rules.d).
// If the udev rules are not updated, or your user does not otherwise have the
// necessary permissions, New()/Reset() will fail on the eventual call to claim
// the USB HID device (function (*DeviceInfo).Open() in package karalabe/hid).
//
// Returns an error if the receiver is invalid or if chip settings could not be
// read from or written to flash memory.
func (mcp *MCP2221A) ConfigVIDPID(vid uint16, pid uint16) error {

	if ok, err := mcp.valid(); !ok {
		return err
	}

	if cmd, err := mcp.flash.chipSettings(true); nil != err {
		return fmt.Errorf("chipSettings(): %v", err)
	} else {

		cmd[6] = byte(vid & 0xFF)
		cmd[7] = byte((vid >> 8) & 0xFF)
		cmd[8] = byte(pid & 0xFF)
		cmd[9] = byte((pid >> 8) & 0xFF)

		if err := mcp.flash.write(subcmdChipSettings, cmd); nil != err {
			return fmt.Errorf("write(): %v", err)
		}
	}

	return nil
}

// ConfigReqCurrent writes to flash memory the minimum required current (in mA)
// requested from the USB host during enumeration for bus-powered operation.
// The provided current should be a multiple of 2 mA and at maximum 510 mA.
// If the provided current is not a multiple of 2 mA, the requested current is
// incremented by 1 mA.
// If the provided current is greater than 510 mA, the requested current is set
// to 510 mA.
//
// Returns an error if the receiver is invalid or if the settings could not be
// read from or written to flash memory.
func (mcp *MCP2221A) ConfigReqCurrent(ma uint16) error {

	if ok, err := mcp.valid(); !ok {
		return err
	}

	max := uint16(0xFF * 2)
	req := uint16(ma + (ma & 1))
	if req > max {
		req = max
	}

	if cmd, err := mcp.flash.chipSettings(true); nil != err {
		return fmt.Errorf("chipSettings(): %v", err)
	} else {

		cmd[11] = byte(req / 2)

		if err := mcp.flash.write(subcmdChipSettings, cmd); nil != err {
			return fmt.Errorf("write(): %v", err)
		}
	}

	return nil
}

=======
>>>>>>> 5cba9e80972c66338ae05cfe80036620cd24db80
// unlockFlags returns the default device locked flag and flash writeable flag,
// in that order, for the given ChipSecurity sec.
func unlockFlags(sec ChipSecurity) (bool, bool) {

	dev, fla := true, false

	switch sec {
	case SecUnsecured:
		dev = false // device not locked
		fla = true  // flash writeable
	case SecPassword:
		dev = false // device not locked
		fla = false // flash not writeable
	case SecLocked1, SecLocked2:
		dev = true  // device locked
		fla = false // flash not writeable
	}

	return dev, fla
}

<<<<<<< HEAD
// ConfigUnlock sends a flash access command with a given slice of bytes as
// password, returning true if the password was accepted and access granted.
=======
// Unlock sends a flash access command with a given slice of bytes as password,
// returning true if the password was accepted and access granted.
>>>>>>> 5cba9e80972c66338ae05cfe80036620cd24db80
// If more than 8 bytes are provided as password, the first 8 bytes are used and
// the remaining bytes are truncated.
// If the current chip security configuration is set to unsecured (no password),
// then this command has no effect and its return value is not guaranteed.
//
// IMPORTANT-SECURITY-1:
//   Sending too many flash access commands with the incorrect password will
//   **PERMANENTLY** lock the flash memory device from write access. Read access
//   is still permitted, but there is absolutely no way to write any changes to
//   flash memory. Congratulations, you have sorta-ruined your MCP2221A. Trust
//   me, I can empathize, very unfortunately.
//
// Returns false and an error if the receiver is invalid, the password slice is
// nil, the password command could not be sent, the provided password was
// incorrect, or if the flash has been permanently locked.
<<<<<<< HEAD
func (mcp *MCP2221A) ConfigUnlock(pass []byte) (bool, error) {
=======
func (mcp *MCP2221A) Unlock(pass []byte) (bool, error) {
>>>>>>> 5cba9e80972c66338ae05cfe80036620cd24db80

	if ok, err := mcp.valid(); !ok {
		return false, err
	}

	if mcp.locked {
		return false, fmt.Errorf("flash access permanently locked")
	}

	if nil == pass {
		return false, fmt.Errorf("invalid password bytes (nil)")
	}

	// copy the password to a buffer exactly 8 bytes in length, padded with any
	// necessary trailing zeroes.
	buf := make([]byte, PasswordLen)
	copy(buf, pass)

	var (
		rsp []byte
		err error
	)

	cmd := makeMsg()
	copy(cmd[2:], buf)
	if rsp, err = mcp.send(cmdFlashPasswd, cmd); nil != err {
		// err is set when our response status code (byte index 1) is non-zero, but
		// we need to inspect that code to identify rejection reason (below).
		if nil == rsp {
			return false, fmt.Errorf("send(): %v", err)
		}
	}

	switch ChipSecurity(rsp[1]) {

	case SecUnsecured:
<<<<<<< HEAD
		mcp.flash.writeable = true
=======
		mcp.Flash.writeable = true
>>>>>>> 5cba9e80972c66338ae05cfe80036620cd24db80
		return true, nil

	case SecPassword:
		return false, fmt.Errorf("invalid password")

	case SecLocked1, SecLocked2:
		return false, fmt.Errorf("flash access permanently locked")

	default:
		return false, fmt.Errorf("unknown reject reason")
	}
}

// -- DEVICE ---------------------------------------------------------- [end] --
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// -- SRAM ---------------------------------------------------------- [start] --

// sram contains the methods associated with the SRAM component of the
// MCP2221A.
type sram struct {
	*MCP2221A
}

// read sends a command requesting current SRAM configuration and returns the
// entire response message.
//
// Returns a nil slice and error if the receiver is invalid, the given range is
// invalid, or if the configuration command could not be sent.
func (mod *sram) read() ([]byte, error) {

	if ok, err := mod.valid(); !ok {
		return nil, err
	}

	cmd := makeMsg()
	if rsp, err := mod.send(cmdSRAMGet, cmd); nil != err {
		return nil, fmt.Errorf("send(): %v", err)
	} else {
		return rsp, nil
	}
}

// readRange reads the current SRAM configuration and returns a byte slice
// within the given interval (inclusive) from the response message.
//
// Returns a nil slice and error if the receiver is invalid, the given range is
// invalid, or if the configuration command could not be sent.
func (mod *sram) readRange(start byte, stop byte) ([]byte, error) {

	if ok, err := mod.valid(); !ok {
		return nil, err
	}

	if (start > stop) || (stop >= MsgSz) {
		return nil, fmt.Errorf("invalid byte range: [%d, %d]", start, stop)
	}

	if rsp, err := mod.read(); nil != err {
		return nil, fmt.Errorf("read(): %v", err)
	} else {
		return rsp[start : stop+1], nil
	}
}

// -- SRAM ------------------------------------------------------------ [end] --
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// -- FLASH --------------------------------------------------------- [start] --

// flash contains the methods associated with the flash memory component of
// the MCP2221A.
type flash struct {
	*MCP2221A

	// writeable helps prevent permanently locking the flash memory device on
	// accident by acting as a gate that must be cleared before any flash write
	// commands can be performed. Clearing the flag is performed by providing the
<<<<<<< HEAD
	// correct password to ConfigUnlock(). The flag is also automatically cleared
	// when the device is created and the security settings read from flash memory
=======
	// correct password to Unlock(). The flag is also automatically cleared when
	// the device is created and the security settings read from flash memory
>>>>>>> 5cba9e80972c66338ae05cfe80036620cd24db80
	// indicate no password is required. The flag is set again automatically when
	// reset or closed (and cleared again after startup if no password is set).
	writeable bool
}

// Constants related to the flash memory module.
const (
	subcmdChipSettings byte = 0x00
	subcmdGPSettings   byte = 0x01
	subcmdUSBMfgDesc   byte = 0x02
	subcmdUSBProdDesc  byte = 0x03
	subcmdUSBSerialNo  byte = 0x04
	subcmdSerialNo     byte = 0x05
)

// Polarity represents the digital state of certain pins/bits in the flash
// settings struct.
type Polarity bool

// ChipSecurity holds one of the enumerated security configuration constants.
type ChipSecurity byte

// Constants related to ChipSecurity.
const (
	SecUnsecured ChipSecurity = 0x00
	SecPassword  ChipSecurity = 0x01
	SecLocked1   ChipSecurity = 0x02 // equivalent to SecLocked2
	SecLocked2   ChipSecurity = 0x03 // equivalent to SecLocked1
)

// PasswordLen is the maximum number of bytes in a chip security password.
const PasswordLen = 8

// chipSettings contains the "chip settings" (subcommand 0x00) configuration
// stored in flash memory. the members are conveniently typed for general usage.
// Obtain a new instance containing the actual current data stored in flash by
// calling parseChipSettings() with the return from chipSettings(false).
type chipSettings struct {
	cdcSerialNoEnumEnable bool
	ledURXPol             Polarity
	ledUTXPol             Polarity
	ledI2CPol             Polarity
	suspndPol             Polarity
	usbcfgPol             Polarity
	chipSecurity          ChipSecurity
	clkOutDiv             byte
	dacVRef               VRef
	dacOutput             byte
	intEdge               IntEdge
	adcVRef               VRef
	usbVID                uint16
	usbPID                uint16
	usbPowerAttr          byte
	usbReqCurrent         byte
}

// parseChipSettings parses the response message from the chip settings
// subcommand of the flash read command.
//
// Returns a pointer to a newly-created chipSettings object on success, or nil
// if the given response message is nil or has inadequate length.
func parseChipSettings(msg []byte) *chipSettings {
	if nil == msg || len(msg) < MsgSz {
		return nil
	}
	return &chipSettings{
		cdcSerialNoEnumEnable: 0x01 == ((msg[4] >> 7) & 0x01),
		ledURXPol:             Polarity(0x01 == ((msg[4] >> 6) & 0x01)),
		ledUTXPol:             Polarity(0x01 == ((msg[4] >> 5) & 0x01)),
		ledI2CPol:             Polarity(0x01 == ((msg[4] >> 4) & 0x01)),
		suspndPol:             Polarity(0x01 == ((msg[4] >> 3) & 0x01)),
		usbcfgPol:             Polarity(0x01 == ((msg[4] >> 2) & 0x01)),
		chipSecurity:          ChipSecurity(msg[4] & 0x03),
		clkOutDiv:             msg[5] & 0x0F,
		dacVRef:               VRef((msg[6] >> 5) & 0x03),
		dacOutput:             msg[6] & 0x0F,
		intEdge:               IntEdge((msg[7] >> 5) & 0x03),
		adcVRef:               VRef((msg[7] >> 2) & 0x03),
		usbVID:                (uint16(msg[9]) << 8) | uint16(msg[8]),
		usbPID:                (uint16(msg[11]) << 8) | uint16(msg[10]),
		usbPowerAttr:          msg[12],
		usbReqCurrent:         msg[13],
	}
}

// parseFlashString parses a UTF-16-encoded string stored in the response
// messages of flash read commands (0xB0).
func parseFlashString(b []byte) string {

	// 16-bit unicode (2 bytes per rune), starting at byte 4
	const max byte = (MsgSz - 4) / 2

	n := (b[2] - 2) / 2 // length stored at byte 2

	switch {
	case n == 0: // no UTF symbols
		return ""
	case n > max: // buffer overrun
		n = max
	}

	p := []uint16{}
	for i := byte(0); i < n; i++ {
		p = append(p, (uint16(b[4+2*i+1])<<8)|uint16(b[4+2*i]))
	}

	return string(utf16.Decode(p))
}

// read reads the settings associated with the given subcommand sub from flash
// memory and returns the response message.
//
// Returns nil and an error if the receiver is invalid, subcommand is invalid,
// or if the flash read command could not be sent.
func (mod *flash) read(sub byte) ([]byte, error) {

	if ok, err := mod.valid(); !ok {
		return nil, err
	}

	if sub > subcmdSerialNo {
		return nil, fmt.Errorf("invalid subcommand: %d", sub)
	}

	cmd := makeMsg()
	cmd[1] = sub

	if rsp, err := mod.send(cmdFlashRead, cmd); nil != err {
		return nil, fmt.Errorf("send(): %v", err)
	} else {
		return rsp, nil
	}
}

// write writes the given settings data associated with subcommand sub to flash
// memory.
// The flash memory device must be unlocked for write-access prior to calling
// write() by either configuring the chip security as unsecured (default), or by
// calling ConfigUnlock() with the stored 8-byte flash access password.
//
// Returns an error if the receiver is invalid, writeable flag is false,
// subcommand is invalid, or if the flash write command could not be sent.
func (mod *flash) write(sub byte, data []byte) error {

	if ok, err := mod.valid(); !ok {
		return err
	}

	if !mod.writeable {
		return fmt.Errorf("flash access permanently locked")
	}

	if sub >= subcmdSerialNo {
		return fmt.Errorf("invalid subcommand: %d", sub)
	}

	data[1] = sub

	if _, err := mod.send(cmdFlashWrite, data); nil != err {
		return fmt.Errorf("send(): %v", err)
	}

	return nil
}

// chipSettings reads the current chip settings stored in flash memory and
// returns the byte slice from its response message, aligned as either a
// read-response or a write-command of the flash chip-settings subcommand.
// If write is true, the returned slice can then be used for manipulation before
// passing it on directly as the message data to flash.write().
// These settings do not necessarily reflect the current device configuration,
// which are stored in SRAM.
//
// The important/relevant content of the chip-settings flash-write command and
// flash-read response are exactly the same, except that the flash-write content
// starts at byte index 2:
//
//          READ-CHIP-SETTINGS       WRITE-CHIP-SETTINGS
//         --------------------     ---------------------
//  [0]          Command                  Command
//  [1]          Success                 Subcommand
//  [2]          Length              <CHIP-SETTINGS[0]>
//  [3]          Ignored             <CHIP-SETTINGS[1]>
//  [4]     <CHIP-SETTINGS[0]>       <CHIP-SETTINGS[2]>
//  [5]     <CHIP-SETTINGS[1]>       <CHIP-SETTINGS[3]>
//  [N]            ...                      ...
//
// If write is false, the message formatted underneath READ-CHIP-SETTINGS is
// returned.
// If write is true, the message formatted underneath WRITE-CHIP-SETTINGS is
// returned.
//
// Returns nil with an error if the receiver is invalid or could not read from
// flash memory.
func (mod *flash) chipSettings(write bool) ([]byte, error) {

	if ok, err := mod.valid(); !ok {
		return nil, err
	}

	if rsp, err := mod.read(subcmdChipSettings); nil != err {
		return nil, fmt.Errorf("read(): %v", err)
	} else {
		if write {
			cmd := makeMsg()
			copy(cmd[2:], rsp[4:])
			return cmd, nil
		} else {
			return rsp, nil
		}
	}
}

// chipSecurity reads and returns the current security access configuration from
// flash memory.
//
// Returns SecLocked2 and an error if the receiver is invalid or the chip
// settings could not be read from flash memory.
func (mod *flash) chipSecurity() (ChipSecurity, error) {

	if ok, err := mod.valid(); !ok {
		return SecLocked2, err
	}

	// read the current chip settings stored in flash memory
	if chp, err := mod.chipSettings(false); nil != err {
		return SecLocked2, fmt.Errorf("chipSettings(): %v", err)
	} else {
		cs := parseChipSettings(chp)
		return cs.chipSecurity, nil
	}
}

// gpioSettings reads the current GP settings from flash memory and returns only
// the range of bytes containing all of the GPIO pin configurations.
// The returned slice will always be the same length (4), and each pin's
// configuration is stored at its respective index (i.e. pin 2 will be at [2]).
// These settings do not necessarily reflect the current device configuration,
// which are stored in SRAM.
//
// Returns nil with an error if the receiver is invalid or could not read from
// flash memory.
func (mod *flash) gpioSettings() ([]byte, error) {

	if ok, err := mod.valid(); !ok {
		return nil, err
	}

	if rsp, err := mod.read(subcmdGPSettings); nil != err {
		return nil, fmt.Errorf("read(): %v", err)
	} else {
		return rsp[4:8], nil
	}
}

<<<<<<< HEAD
=======
// parseFlashString parses a UTF-16-encoded string stored in the response
// messages of flash read commands (0xB0).
func parseFlashString(b []byte) string {

	// 16-bit unicode (2 bytes per rune), starting at byte 4
	const max byte = (MsgSz - 4) / 2

	n := (b[2] - 2) / 2 // length stored at byte 2

	switch {
	case n == 0: // no UTF symbols
		return ""
	case n > max: // buffer overrun
		n = max
	}

	p := []uint16{}
	for i := byte(0); i < n; i++ {
		p = append(p, (uint16(b[4+2*i+1])<<8)|uint16(b[4+2*i]))
	}

	return string(utf16.Decode(p))
}

// USBManufacturer reads the current USB manufacturer description from flash
// memory and returns it as a string.
//
// Returns an empty string and error if the receiver is invalid or if the flash
// configuration could not be read.
func (mod *Flash) USBManufacturer() (string, error) {

	if ok, err := mod.valid(); !ok {
		return "", err
	}

	if rsp, err := mod.read(subcmdUSBMfgDesc); nil != err {
		return "", fmt.Errorf("read(): %v", err)
	} else {
		return parseFlashString(rsp), nil
	}
}

// USBProduct reads the current USB product description from flash memory and
// returns it as a string.
//
// Returns an empty string and error if the receiver is invalid or if the flash
// configuration could not be read.
func (mod *Flash) USBProduct() (string, error) {

	if ok, err := mod.valid(); !ok {
		return "", err
	}

	if rsp, err := mod.read(subcmdUSBProdDesc); nil != err {
		return "", fmt.Errorf("read(): %v", err)
	} else {
		return parseFlashString(rsp), nil
	}
}

// USBSerialNo reads the current USB serial number from flash memory and returns
// it as a string.
//
// Returns an empty string and error if the receiver is invalid or if the flash
// configuration could not be read.
func (mod *Flash) USBSerialNo() (string, error) {

	if ok, err := mod.valid(); !ok {
		return "", err
	}

	if rsp, err := mod.read(subcmdUSBSerialNo); nil != err {
		return "", fmt.Errorf("read(): %v", err)
	} else {
		return parseFlashString(rsp), nil
	}
}

// FactorySerialNo reads the factory serial number (read-only) from flash memory
// and returns it as a string.
//
// Returns an empty string and error if the receiver is invalid or if the flash
// configuration could not be read.
func (mod *Flash) FactorySerialNo() (string, error) {

	if ok, err := mod.valid(); !ok {
		return "", err
	}

	if rsp, err := mod.read(subcmdSerialNo); nil != err {
		return "", fmt.Errorf("read(): %v", err)
	} else {
		cnt := rsp[2] - 2
		if cnt > MsgSz-4 {
			cnt = MsgSz - 4
		}
		return string(rsp[4 : 4+cnt]), nil
	}
}

// ConfigVIDPID writes the given vendor ID and product ID to flash memory.
// These settings are non-volatile and become the actual VID and PID with which
// the device will enumerate itself on the USB host (i.e., if changed, the
// global VID and PID constants defined in this package cannot be used to open
// the device) on the next reset/startup.
// Therefore, if changed, be sure your system settings are updated to permit
// access to the device, since it will appear to be a new USB HID device.
//
// For instance, on some udev-based Linux systems, you may need to update your
// udev rules to grant read-write access to your user for devices matching the
// new VID/PID (these rules are traditionally kept in /etc/udev/rules.d).
// If the udev rules are not updated, or your user does not otherwise have the
// necessary permissions, New()/Reset() will fail on the eventual call to claim
// the USB HID device (function (*DeviceInfo).Open() in package karalabe/hid).
//
// Returns an error if the receiver is invalid or if chip settings could not be
// read from or written to flash memory.
func (mod *Flash) ConfigVIDPID(vid uint16, pid uint16) error {

	if ok, err := mod.valid(); !ok {
		return err
	}

	if cmd, err := mod.chipSettings(true); nil != err {
		return fmt.Errorf("chipSettings(): %v", err)
	} else {

		cmd[6] = byte(vid & 0xFF)
		cmd[7] = byte((vid >> 8) & 0xFF)
		cmd[8] = byte(pid & 0xFF)
		cmd[9] = byte((pid >> 8) & 0xFF)

		if err := mod.write(subcmdChipSettings, cmd); nil != err {
			return fmt.Errorf("write(): %v", err)
		}
	}

	return nil
}

>>>>>>> 5cba9e80972c66338ae05cfe80036620cd24db80
// -- FLASH ----------------------------------------------------------- [end] --
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// -- GPIO ---------------------------------------------------------- [start] --

// GPIO contains the methods associated with the GPIO module of the MCP2221A.
type GPIO struct {
	*MCP2221A
}

// GPIOMode and GPIODir represent two of the configuration parameters for all
// of the general purpose (GP) pins.
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
	ModeDediFunc GPIOMode = 0x01 //  SSPND     CLKR      USBCFG   LED_I2C
	ModeAltFunc0 GPIOMode = 0x02 //  LED_URX   ADC1      ADC2     ADC3
	ModeAltFunc1 GPIOMode = 0x03 //  --        LED_UTX   DAC1     DAC2
	ModeAltFunc2 GPIOMode = 0x04 //  --        IOC       --       --
	ModeInvalid  GPIOMode = 0xEE // invalid mode is used as error condition

	// General GPIO functions
	ModeADC = ModeAltFunc0 // ADCn - GP1, GP2, GP3 alternate function 0
	ModeDAC = ModeAltFunc1 // DACn -      GP2, GP3 alternate function 1

	// Special functions
	ModeSUSPND = ModeDediFunc // SSPND   - GP0 dedicated function
	ModeCLKOUT = ModeDediFunc // CLKR    - GP1 dedicated function
	ModeUSBCFG = ModeDediFunc // USBCFG  - GP2 dedicated function
	ModeINTCHG = ModeAltFunc2 // IOC     - GP1 alternate function 2

	// LED status functions
	ModeLEDI2C = ModeDediFunc // LED_I2C - GP3 dedicated function
	ModeLEDURX = ModeAltFunc0 // LED_URX - GP0 alternate function 0
	ModeLEDUTX = ModeAltFunc1 // LED_UTX - GP1 alternate function 1

	// GPIO directions
	DirOutput  GPIODir = 0x00 // direction OUT is used for writing values to pins
	DirInput   GPIODir = 0x01 // direction IN is used for reading values from pins
	DirInvalid GPIODir = 0xEF // invalid direction is used as error condition
)

// SetConfig configures a given pin with a default output value, operation mode,
// and direction.
// These settings only affect the current device configuration and are not
// retained after next startup/reset (see: FlashConfig()).
//
// Returns an error if the receiver is invalid, the pin index is invalid, the
// current configuration could not be read, or if the new configuration could
// not be sent.
func (mod *GPIO) SetConfig(pin byte, val byte, mode GPIOMode, dir GPIODir) error {

	if ok, err := mod.valid(); !ok {
		return err
	}

	if pin >= GPPinCount {
		return fmt.Errorf("invalid GPIO pin: %d", pin)
	}

	cmd := makeMsg()

	if cur, err := mod.sram.readRange(22, 25); nil != err {
		return fmt.Errorf("sram.readRange(): %v", err)
	} else {
		// copy the current GPIO settings because they will -all- be set with the
		// command request
		cmd[7] = WordSet // alter GP designation
		copy(cmd[8:], cur)
	}

	// and then update our selected pin as desired
	cmd[8+pin] = (val << 4) | (byte(dir) << 3) | byte(mode)

	if _, err := mod.send(cmdSRAMSet, cmd); nil != err {
		return fmt.Errorf("send(): %v", err)
	}

	return nil
}

// FlashConfig configures a given pin with a default output value, operation
// mode, direction, and then writes that configuration to flash memory.
// These settings become default and are retained after next startup/reset.
//
// Returns an error if the receiver is invalid, the pin index is invalid, the
// current configuration could not be read, or if the new configuration could
// not be sent.
func (mod *GPIO) FlashConfig(pin byte, val byte, mode GPIOMode, dir GPIODir) error {

	if ok, err := mod.valid(); !ok {
		return err
	}

	if pin >= GPPinCount {
		return fmt.Errorf("invalid GPIO pin: %d", pin)
	}

	if err := mod.SetConfig(pin, val, mode, dir); nil != err {
		return fmt.Errorf("SetConfig(): %v", err)
	}

	cmd := makeMsg()

	if cur, err := mod.flash.gpioSettings(); nil != err {
		return fmt.Errorf("flash.gpioSettings(): %v", err)
	} else {
		copy(cmd[2:], cur)
	}

	cmd[2+pin] = (val << 4) | (byte(dir) << 3) | byte(mode)

	if err := mod.flash.write(subcmdGPSettings, cmd); nil != err {
		return fmt.Errorf("flash.write(): %v", err)
	}

	return nil
}

// GetConfig reads the current default output value, operation mode, and
// direction of a given pin.
//
// Returns an error if the receiver is invalid, the pin index is invalid, or if
// the current configuration could not be read.
func (mod *GPIO) GetConfig(pin byte) (byte, GPIOMode, GPIODir, error) {

	if ok, err := mod.valid(); !ok {
		return WordClr, ModeInvalid, DirInvalid, err
	}

	if pin >= GPPinCount {
		return WordClr, ModeInvalid, DirInvalid, fmt.Errorf("invalid GPIO pin: %d", pin)
	}

	if rsp, err := mod.sram.readRange(22, 25); nil != err {
		return WordClr, ModeInvalid, DirInvalid, fmt.Errorf("sram.readRange(): %v", err)
	} else {
		mode := GPIOMode(rsp[pin] & 0x07)
		dir := GPIODir((rsp[pin] >> 3) & 0x01)
		val := (rsp[pin] >> 4) & 0x01
		return val, mode, dir, nil
	}
}

// Set sets the digital output value for a given pin.
//
// Returns an error if the receiver is invalid, the pin index is invalid, or if
// the pin value could not be set (e.g. pin not configured for GPIO operation).
func (mod *GPIO) Set(pin byte, val byte) error {

	if ok, err := mod.valid(); !ok {
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
	cmd[i+3] = byte(DirOutput)

	if _, err := mod.send(cmdGPIOSet, cmd); nil != err {
		return fmt.Errorf("send(): %v", err)
	}

	return nil
}

// Get gets the current digital value of a given pin.
//
// Returns an error if the receiver is invalid, the pin index is invalid, or if
// the pin value could not be set (e.g. pin not configured for GPIO operation).
func (mod *GPIO) Get(pin byte) (byte, error) {

	if ok, err := mod.valid(); !ok {
		return WordClr, err
	}

	if pin >= GPPinCount {
		return WordClr, fmt.Errorf("invalid GPIO pin: %d", pin)
	}

	cmd := makeMsg()
	if rsp, err := mod.send(cmdGPIOGet, cmd); nil != err {
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
// -- ADC ----------------------------------------------------------- [start] --

// ADC contains the methods associated with the ADC module of the MCP2221A.
type ADC struct {
	*MCP2221A
}

// SetConfig configures the analog-to-digital converter by setting the given
// pin's operation mode to ADC and the ADC reference voltage to ref.
// These settings only affect the current device configuration and are not
// retained after next startup/reset (see: FlashConfig()).
//
// Returns an error if the receiver is invalid, pin does not have an associated
// ADC channel, ref is invalid, or if the new configuration could not be sent.
func (mod *ADC) SetConfig(pin byte, ref VRef) error {

	if ok, err := mod.valid(); !ok {
		return err
	}

	if pin >= GPPinCount {
		return fmt.Errorf("invalid GPIO pin: %d", pin)
	}

	if _, ok := chanADC[pin]; !ok {
		return fmt.Errorf("pin does not support ADC: %d", pin)
	}

	if !isVRefValid(ref) {
		return fmt.Errorf("invalid reference voltage: %d", ref)
	}

	if err := mod.GPIO.SetConfig(pin, WordClr, ModeADC, DirInput); nil != err {
		return fmt.Errorf("GPIO.SetConfig(): %v", err)
	}

	cmd := makeMsg()

	// set VREF alter bit (7) and set reference voltage enum
	cmd[5] = (1 << 7) | byte(ref)

	if _, err := mod.send(cmdSRAMSet, cmd); nil != err {
		return fmt.Errorf("send(): %v", err)
	}

	return nil
}

// FlashConfig configures the analog-to-digital converter by setting the given
// pin's operation mode to ADC, the ADC reference voltage to ref, and then
// writes these settings to flash memory.
// These settings become default and are retained after next startup/reset.
//
// Returns an error if the receiver is invalid, pin does not have an associated
// ADC channel, ref is invalid, or if the new configuration could not be sent.
func (mod *ADC) FlashConfig(pin byte, ref VRef) error {

	if ok, err := mod.valid(); !ok {
		return err
	}

	if pin >= GPPinCount {
		return fmt.Errorf("invalid GPIO pin: %d", pin)
	}

	if _, ok := chanADC[pin]; !ok {
		return fmt.Errorf("pin does not support ADC: %d", pin)
	}

	if !isVRefValid(ref) {
		return fmt.Errorf("invalid reference voltage: %d", ref)
	}

	if err := mod.GPIO.FlashConfig(pin, WordClr, ModeADC, DirInput); nil != err {
		return fmt.Errorf("GPIO.FlashConfig(): %v", err)
	}

	if cmd, err := mod.flash.chipSettings(true); nil != err {
		return fmt.Errorf("flash.chipSettings(): %v", err)
	} else {
		cmd[5] &= ^byte(0x07 << 2) // mask off the VRef bits (2-4)
		cmd[5] |= (byte(ref) << 2)

		if err := mod.flash.write(subcmdChipSettings, cmd); nil != err {
			return fmt.Errorf("flash.write(): %v", err)
		}
	}

	return nil
}

// GetConfig reads the current ADC configuration for the given pin, returning
// the ADC reference voltage.
//
// Returns VRefDefault reference voltage and an error if the receiver is
// invalid, pin is invalid, or if the current configuration could not be read.
func (mod *ADC) GetConfig(pin byte) (VRef, error) {

	if ok, err := mod.valid(); !ok {
		return VRefDefault, err
	}

	if pin >= GPPinCount {
		return VRefDefault, fmt.Errorf("invalid GPIO pin: %d", pin)
	}

	if _, ok := chanADC[pin]; !ok {
		return VRefDefault, fmt.Errorf("pin does not support ADC: %d", pin)
	}

	if buf, err := mod.sram.readRange(7, 7); nil != err {
		return VRefDefault, fmt.Errorf("sram.readRange(): %v", err)
	} else {
		ref := VRef((buf[0] >> 2) & 0x07)
		return ref, nil
	}
}

// Read performs an analog-to-digital conversion reading on the given pin and
// returns the value.
//
// Returns the converted analog value as an unsigned 16-bit integer.
// Returns an error if the receiver is invalid, VRef is invalid, or if the new
// configuration could not be sent.
func (mod *ADC) Read(pin byte) (uint16, error) {

	if ok, err := mod.valid(); !ok {
		return 0, err
	}

	if pin >= GPPinCount {
		return 0, fmt.Errorf("invalid GPIO pin: %d", pin)
	}

	var (
		adc byte
		ok  bool
	)

	if adc, ok = chanADC[pin]; !ok {
		return 0, fmt.Errorf("pin does not support ADC: %d", pin)
	}

	if stat, err := mod.status(); nil != err {
		return 0, fmt.Errorf("status(): %v", err)
	} else {
		return stat.adcChan[adc], nil
	}
}

// -- ADC ------------------------------------------------------------- [end] --
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// -- DAC ----------------------------------------------------------- [start] --

// DAC contains the methods associated with the DAC module of the MCP2221A.
type DAC struct {
	*MCP2221A
}

// SetConfig configures the digital-to-analog converter by setting the given
// pin's operation mode to DAC and the DAC reference voltage to ref.
// These settings only affect the current device configuration and are not
// retained after next startup/reset (see: FlashConfig()).
//
// Returns an error if the receiver is invalid, pin does not have an associated
// DAC output, ref is invalid, or if the new configuration could not be sent.
func (mod *DAC) SetConfig(pin byte, ref VRef) error {

	if ok, err := mod.valid(); !ok {
		return err
	}

	if pin >= GPPinCount {
		return fmt.Errorf("invalid GPIO pin: %d", pin)
	}

	if _, ok := outpDAC[pin]; !ok {
		return fmt.Errorf("pin does not support DAC: %d", pin)
	}

	if !isVRefValid(ref) {
		return fmt.Errorf("invalid reference voltage: %d", ref)
	}

	if err := mod.GPIO.SetConfig(pin, WordClr, ModeDAC, DirOutput); nil != err {
		return fmt.Errorf("GPIO.SetConfig(): %v", err)
	}

	cmd := makeMsg()

	// set VREF alter bit (7) and set reference voltage enum
	cmd[3] = (1 << 7) | byte(ref)

	if _, err := mod.send(cmdSRAMSet, cmd); nil != err {
		return fmt.Errorf("send(): %v", err)
	}

	return nil
}

// FlashConfig configures the digital-to-analog converter by setting the given
// pin's operation mode to DAC, the DAC reference voltage to ref, the default
// startup output value to val, and then writes these settings to flash memory.
// These settings become default and are retained after next startup/reset.
//
// Returns an error if the receiver is invalid, pin does not have an associated
// DAC output, ref is invalid, or if the new configuration could not be sent.
func (mod *DAC) FlashConfig(pin byte, ref VRef, val byte) error {

	if ok, err := mod.valid(); !ok {
		return err
	}

	if pin >= GPPinCount {
		return fmt.Errorf("invalid GPIO pin: %d", pin)
	}

	if _, ok := outpDAC[pin]; !ok {
		return fmt.Errorf("pin does not support DAC: %d", pin)
	}

	if !isVRefValid(ref) {
		return fmt.Errorf("invalid reference voltage: %d", ref)
	}

	if err := mod.GPIO.FlashConfig(pin, WordClr, ModeDAC, DirOutput); nil != err {
		return fmt.Errorf("GPIO.FlashConfig(): %v", err)
	}

	if cmd, err := mod.flash.chipSettings(true); nil != err {
		return fmt.Errorf("flash.chipSettings(): %v", err)
	} else {
		cmd[4] = (byte(ref) << 5) | (val & 0x1F)

		if err := mod.flash.write(subcmdChipSettings, cmd); nil != err {
			return fmt.Errorf("flash.write(): %v", err)
		}
	}

	return nil
}

// GetConfig reads the current DAC configuration for the given pin, returning
// the power-up DAC value and reference voltage.
//
// Returns WordClr power-up value, VRefDefault reference voltage, and an error
// if the receiver is invalid, pin is invalid or is not configured for DAC
// operation, or if the current configuration could not be read.
func (mod *DAC) GetConfig(pin byte) (byte, VRef, error) {

	if ok, err := mod.valid(); !ok {
		return WordClr, VRefDefault, err
	}

	if pin >= GPPinCount {
		return WordClr, VRefDefault, fmt.Errorf("invalid GPIO pin: %d", pin)
	}

	if _, ok := outpDAC[pin]; !ok {
		return WordClr, VRefDefault, fmt.Errorf("pin does not support DAC: %d", pin)
	}

	if buf, err := mod.sram.readRange(6, 6); nil != err {
		return WordClr, VRefDefault, fmt.Errorf("sram.readRange(): %v", err)
	} else {
		val := buf[0] & 0x1F // power-up DAC value (bits 0-4)
		ref := VRef((buf[0] >> 5) & 0x07)
		return val, ref, nil
	}
}

// Write performs a digital-to-analog conversion and outputs the given value
// (modulo 32) on all DAC-enabled pins.
// The device only has a single DAC with two pins connected to it, so the value
// output will be present on both pins if they are both configured for DAC
// operation.
//
// Returns an error if the receiver is invalid or if the converted value could
// not be sent.
func (mod *DAC) Write(val byte) error {

	if ok, err := mod.valid(); !ok {
		return err
	}

	cmd := makeMsg()

	// simulate 5-bit integer overflow
	out := val % 0x20

	// set DAC alter bit (7) and set requested 5-bit value
	cmd[4] = (1 << 7) | byte(out)

	if _, err := mod.send(cmdSRAMSet, cmd); nil != err {
		return fmt.Errorf("send(): %v", err)
	}

	return nil
}

// -- DAC ------------------------------------------------------------- [end] --
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// -- Alt ----------------------------------------------------------- [start] --

// Alt contains the various structures for enabling and configuring each of the
// special-purpose GP dedicated and alternate operating modes.
type Alt struct {
	*MCP2221A
	SUSPND *SUSPND
	CLKOUT *CLKOUT
	USBCFG *USBCFG
	INTCHG *INTCHG
	LEDI2C *LEDI2C
	LEDURX *LEDURX
	LEDUTX *LEDUTX
}

// Constants defining the pins capable of the unique GP dedicated/alternate
// functions.
const (
	pinSUSPND byte = 0 // SSPND   - GP0 dedicated function
	pinCLKOUT byte = 1 // CLKR    - GP1 dedicated function
	pinUSBCFG byte = 2 // USBCFG  - GP2 dedicated function
	pinINTCHG byte = 1 // IOC     - GP1 alternate function 2
	pinLEDI2C byte = 3 // LED_I2C - GP3 dedicated function
	pinLEDURX byte = 0 // LED_URX - GP0 alternate function 0
	pinLEDUTX byte = 1 // LED_UTX - GP1 alternate function 1
)

// -- SUSPND -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -

// SUSPND contains the methods associated with the SSPND GP dedicated function.
type SUSPND struct {
	*MCP2221A
}

// FlashConfig configures the sole SSPND-capable GPIO pin's operation mode for
// SSPND, sets the SSPND module's default polarity to the given pol, and writes
// these settings to flash memory.
// A positive (true) polarity means the SSPND-capable pin will be driven high
// when the USB host suspends communications, and low on resume.
// A negative (false) polarity drives the SSPND-capable pin opposite the
// behavior of positive (true) polarity.
// These settings become default but only take effect after next startup/reset.
//
// Returns an error if the receiver is invalid, the pin's operation mode could
// not be set, or the flash settings could not be read from or written to.
func (alt *SUSPND) FlashConfig(pol Polarity) error {

	if ok, err := alt.valid(); !ok {
		return err
	}

	if err := alt.GPIO.FlashConfig(pinSUSPND, WordClr, ModeSUSPND, DirOutput); nil != err {
		return fmt.Errorf("GPIO.FlashConfig(): %v", err)
	}

	if cmd, err := alt.flash.chipSettings(true); nil != err {
		return fmt.Errorf("flash.chipSettings(): %v", err)
	} else {

		cmd[2] &= ^byte(0x01 << 3) // mask off the SSPND bit (3)

		// if pol is false, we set the SSPND bit (the device will use the negated
		// value of this bit upon entering suspend moded). otherwise, we leave the
		// bit clear.
		if !pol {
			cmd[2] |= byte(0x01 << 3)
		}

		if err := alt.flash.write(subcmdChipSettings, cmd); nil != err {
			return fmt.Errorf("flash.write(): %v", err)
		}
	}

	return nil
}

// GetConfig returns the current polarity selection of the SSPND operation mode.
// A positive (true) polarity means the SSPND-capable pin will be driven high
// when the USB host suspends communications, and low on resume.
// A negative (false) polarity drives the SSPND-capable pin opposite the
// behavior of positive (true) polarity.
// If the SSPND-capable pin has not been configured for SSPND operation mode,
// this polarity configuration has no effect.
//
// Returns false and an error if the receiver is invalid or if the current
// configuration could not be read.
func (alt *SUSPND) GetConfig() (Polarity, error) {

	if ok, err := alt.valid(); !ok {
		return false, err
	}

	if buf, err := alt.sram.readRange(4, 4); nil != err {
		return false, fmt.Errorf("sram.readRange(): %v", err)
	} else {
		pol := Polarity(0x00 == ((buf[0] >> 3) & 0x01))
		return pol, nil
	}
}

// -- CLKOUT -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -

// CLKOUT contains the methods associated with the CLKR GP dedicated function.
type CLKOUT struct {
	*MCP2221A
}

// DutyCycle represents one of the enumerated constants that can be used as duty
// cycle for the output clock generated on the GP alternate function CLKR pin.
type DutyCycle byte

// Constants for enumerated DutyCycle type that are used as duty cycle for the
// output clock generated on the GP alternate function CLKR pin.
const (
	Duty0Pct  DutyCycle = 0x00 // 100% of clock period is logic "0"
	Duty25Pct DutyCycle = 0x01 // 75% of clock period is logic "0", 25% logic "1"
	Duty50Pct DutyCycle = 0x02 // 50% of clock period is logic "0", 50% logic "1"
	Duty75Pct DutyCycle = 0x03 // 25% of clock period is logic "0", 75% logic "1"
)

func isDutyCycleValid(c DutyCycle) bool {
	return (c >= Duty0Pct) && (c <= Duty75Pct)
}

// ClkOut represents one of the enumerated constants that can be used as clock
// output for the output clock generated on the GP alternate function CLKR pin.
type ClkOut byte

// Constants for enumerated ClkOut type that are used as clock output for the
// output clock generated on the GP alternate function CLKR pin.
const (
	ClkRes    ClkOut = 0x00 // Reserved
	Clk24MHz  ClkOut = 0x01 //  24 MHz clock output
	Clk12MHz  ClkOut = 0x02 //  12 MHz clock output
	Clk6MHz   ClkOut = 0x03 //   6 MHz clock output
	Clk3MHz   ClkOut = 0x04 //   3 MHz clock output
	Clk1p5MHz ClkOut = 0x05 // 1.5 MHz clock output
	Clk750kHz ClkOut = 0x06 // 750 kHz clock output
	Clk375kHz ClkOut = 0x07 // 375 kHz clock output
)

func isClkOutValid(c ClkOut) bool {
	return (c > ClkRes) && (c <= Clk375kHz)
}

// SetConfig configures the sole CLKR-capable GPIO pin's operation mode for
// CLKR and sets the current clock output and duty cycle to the given clock
// output clk and duty cycle dut.
// These settings only affect the current device configuration and are not
// retained after next startup/reset (see: FlashConfig()).
//
// Returns an error if the receiver is invalid, clock output or duty cycle are
// invalid, the pin's operation mode could not be set, or the SRAM configuration
// could not be set.
func (alt *CLKOUT) SetConfig(clk ClkOut, dut DutyCycle) error {

	if ok, err := alt.valid(); !ok {
		return err
	}

	if !isClkOutValid(clk) {
		return fmt.Errorf("invalid clock output: %d", clk)
	}

	if !isDutyCycleValid(dut) {
		return fmt.Errorf("invalid duty cycle: %d", dut)
	}

	if err := alt.GPIO.SetConfig(pinCLKOUT, WordClr, ModeCLKOUT, DirOutput); nil != err {
		return fmt.Errorf("GPIO.SetConfig(): %v", err)
	}

	// first, configure the interrupt detection module per edge argument
	cmd := makeMsg()

	// set the load new clock divider bit (7), duty cycle, and clock divider.
	cmd[2] = (1 << 7) | (byte(dut) << 3) | byte(clk)

	if _, err := alt.send(cmdSRAMSet, cmd); nil != err {
		return fmt.Errorf("send(): %v", err)
	}

	return nil
}

// FlashConfig configures the sole CLKR-capable GPIO pin's operation mode for
// CLKR, sets the default clock output and duty cycle to the given clock output
// clk and duty cycle dut, and writes these settings to flash memory.
// These settings become default but only take effect after next startup/reset.
//
// Returns an error if the receiver is invalid, clock output or duty cycle are
// invalid, the pin's operation mode could not be set, or the configuration
// could not be read from or written to flash memory.
func (alt *CLKOUT) FlashConfig(clk ClkOut, dut DutyCycle) error {

	if ok, err := alt.valid(); !ok {
		return err
	}

	if !isClkOutValid(clk) {
		return fmt.Errorf("invalid clock output: %d", clk)
	}

	if !isDutyCycleValid(dut) {
		return fmt.Errorf("invalid duty cycle: %d", dut)
	}

	if err := alt.GPIO.FlashConfig(pinCLKOUT, WordClr, ModeCLKOUT, DirOutput); nil != err {
		return fmt.Errorf("GPIO.FlashConfig(): %v", err)
	}

	if cmd, err := alt.flash.chipSettings(true); nil != err {
		return fmt.Errorf("flash.chipSettings(): %v", err)
	} else {
		cmd[3] = (byte(dut) << 3) | byte(clk)

		if err := alt.flash.write(subcmdChipSettings, cmd); nil != err {
			return fmt.Errorf("flash.write(): %v", err)
		}
	}

	return nil
}

// GetConfig returns the currently selected clock output and duty cycle of the
// CLKR operation mode.
// If the CLKR-capable pin has not been configured for CLKR operation mode,
// these clock and duty cycle configurations have no effect.
//
// Returns ClkRes, Duty0Pct, and an error if the receiver is invalid or if the
// current configuration could not be read.
func (alt *CLKOUT) GetConfig() (ClkOut, DutyCycle, error) {

	if ok, err := alt.valid(); !ok {
		return ClkRes, Duty0Pct, err
	}

	if buf, err := alt.sram.readRange(5, 5); nil != err {
		return ClkRes, Duty0Pct, fmt.Errorf("sram.readRange(): %v", err)
	} else {
		clk := ClkOut(buf[0] & 0x07)
		dut := DutyCycle((buf[0] >> 3) & 0x03)
		return clk, dut, nil
	}
}

// -- USBCFG -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -

// USBCFG contains the methods associated with the USBCFG GP dedicated function.
type USBCFG struct {
	*MCP2221A
}

// FlashConfig configures the sole USBCFG-capable GPIO pin's operation mode for
// USBCFG, sets the USBCFG module's default polarity to the given pol, and
// writes these settings to flash memory.
// A positive (true) polarity means the USBCFG-capable pin will be driven high
// when the USB device has been enumerated by the USB host, but low until then.
// A negative (false) polarity drives the USBCFG-capable pin opposite the
// behavior of positive (true) polarity.
// These settings become default but only take effect after next startup/reset.
//
// Returns an error if the receiver is invalid, the pin's operation mode could
// not be set, or the flash settings could not be read from or written to.
func (alt *USBCFG) FlashConfig(pol Polarity) error {

	if ok, err := alt.valid(); !ok {
		return err
	}

	if err := alt.GPIO.FlashConfig(pinUSBCFG, WordClr, ModeUSBCFG, DirOutput); nil != err {
		return fmt.Errorf("GPIO.FlashConfig(): %v", err)
	}

	if cmd, err := alt.flash.chipSettings(true); nil != err {
		return fmt.Errorf("flash.chipSettings(): %v", err)
	} else {

		cmd[2] &= ^byte(0x01 << 2) // mask off the USBCFG bit (2)

		// if pol is true, we set the USBCFG bit (the device will use the negated
		// value of this bit upon successful enumeration). otherwise, we leave the
		// bit clear.
		if pol {
			cmd[2] |= byte(0x01 << 2)
		}

		if err := alt.flash.write(subcmdChipSettings, cmd); nil != err {
			return fmt.Errorf("flash.write(): %v", err)
		}
	}

	return nil
}

// GetConfig returns the current polarity selection of the USBCFG operation
// mode.
// A positive (true) polarity means the USBCFG-capable pin will be driven high
// when the USB device has been enumerated by the USB host, but low until then.
// A negative (false) polarity drives the USBCFG-capable pin opposite the
// behavior of positive (true) polarity.
// If the USBCFG-capable pin has not been configured for USBCFG operation mode,
// this polarity configuration has no effect.
//
// Returns false and an error if the receiver is invalid or if the current
// configuration could not be read.
func (alt *USBCFG) GetConfig() (Polarity, error) {

	if ok, err := alt.valid(); !ok {
		return false, err
	}

	if buf, err := alt.sram.readRange(4, 4); nil != err {
		return false, fmt.Errorf("sram.readRange(): %v", err)
	} else {
		pol := Polarity(0x01 == ((buf[0] >> 2) & 0x01))
		return pol, nil
	}
}

// -- INTCHG -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -

// INTCHG contains the methods associated with the IOC GP alternate function.
type INTCHG struct {
	*MCP2221A
}

// IntEdge represents the edge which triggers an interrupt.
type IntEdge byte

// Constants associated with the interrupt detection module (INTCHG).
const (
	NoInterrupt       IntEdge = 0x00
	RisingEdge        IntEdge = 0x01
	FallingEdge       IntEdge = 0x02
	RisingFallingEdge IntEdge = 0x03
)

// SetConfig configures the sole interrupt-capable GPIO pin's operation mode for
// interrupt detection, sets the edge detection to the given edge, and clears
// the current interrupt flag.
// These settings only affect the current device configuration and are not
// retained after next startup/reset (see: FlashConfig()).
//
// Returns an error if the receiver is invalid, the pin's operation mode could
// not be set, the edge configuration could not be set, or if the interrupt flag
// could not be cleared.
func (alt *INTCHG) SetConfig(edge IntEdge) error {

	if ok, err := alt.valid(); !ok {
		return err
	}

	if edge > RisingFallingEdge {
		return fmt.Errorf("invalid interrupt detection edge: %d", edge)
	}

	if err := alt.GPIO.SetConfig(pinINTCHG, WordClr, ModeINTCHG, DirInput); nil != err {
		return fmt.Errorf("GPIO.SetConfig(): %v", err)
	}

	// first, configure the interrupt detection module per edge argument
	cmd := makeMsg()

	// set detection alter bit (7)
	cmd[6] = (1 << 7)

	if NoInterrupt == edge {
		// set only the "clear interrupt detection" bit
		cmd[6] |= 1
	} else {
		// set the "alter positive edge" and "alter negative edge" bits
		cmd[6] |= (1 << 4)
		cmd[6] |= (1 << 2)
		if (RisingEdge & edge) > 0 {
			cmd[6] |= (1 << 3) // enable positive edge detection
		}
		if (FallingEdge & edge) > 0 {
			cmd[6] |= (1 << 1) // enable negative edge detection
		}
	}

	if _, err := alt.send(cmdSRAMSet, cmd); nil != err {
		return fmt.Errorf("send(): %v", err)
	}

	// next, clear the interrupt detection flag in SRAM
	cmd = makeMsg()

	cmd[24] = WordClr

	if _, err := alt.send(cmdSetParams, cmd); nil != err {
		return fmt.Errorf("send(): %v", err)
	}

	return nil
}

// FlashConfig configures the sole interrupt-capable GPIO pin's operation mode
// for interrupt detection, sets the default edge detection to the given edge,
// and then writes these settings to flash memory.
// These settings become default but will only take effect after next
// startup/reset; thus, the interrupt flag is not cleared as it is with
// SetConfig().
//
// Returns an error if the receiver is invalid, the pin's operation mode could
// not be set, the edge configuration could not be set, or if the interrupt flag
// could not be cleared.
func (alt *INTCHG) FlashConfig(edge IntEdge) error {

	if ok, err := alt.valid(); !ok {
		return err
	}

	if edge > RisingFallingEdge {
		return fmt.Errorf("invalid interrupt detection edge: %d", edge)
	}

	if err := alt.GPIO.FlashConfig(pinINTCHG, WordClr, ModeINTCHG, DirInput); nil != err {
		return fmt.Errorf("GPIO.FlashConfig(): %v", err)
	}

	if cmd, err := alt.flash.chipSettings(true); nil != err {
		return fmt.Errorf("flash.chipSettings(): %v", err)
	} else {
		cmd[5] &= ^byte(0x03 << 5) // mask off the edge bits (5-6)
		cmd[5] |= (byte(edge) << 5)

		if err := alt.flash.write(subcmdChipSettings, cmd); nil != err {
			return fmt.Errorf("flash.write(): %v", err)
		}
	}

	return nil
}

// GetConfig returns the selected detection edge configured for the interrupt
// detection module.
//
// Returns NoInterrupt detection edge and an error if the receiver is invalid or
// if the current configuration could not be read.
func (alt *INTCHG) GetConfig() (IntEdge, error) {

	if ok, err := alt.valid(); !ok {
		return NoInterrupt, err
	}

	if buf, err := alt.sram.readRange(7, 7); nil != err {
		return NoInterrupt, fmt.Errorf("sram.readRange(): %v", err)
	} else {
		edge := IntEdge((buf[0] >> 5) & 0x03)
		return edge, nil
	}
}

// -- LEDI2C -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -

// LEDI2C contains the methods associated with the LED_I2C GP dedicated
// function.
type LEDI2C struct {
	*MCP2221A
}

// FlashConfig configures the sole LED_I2C-capable GPIO pin's operation mode for
// LED_I2C, sets the LEDI2C module's default polarity to the given pol, and then
// writes these settings to flash memory.
// A positive (true) polarity means the LED_I2C-capable pin will be driven high
// when I2C traffic is active, and negative (false) with no traffic.
// A negative (false) polarity drives the LED_I2C-capable pin opposite the
// behavior of positive (true) polarity.
// These settings become default but only take effect after next startup/reset.
//
// Returns an error if the receiver is invalid, the pin's operation mode could
// not be set, or the flash settings could not be read from or written to.
func (alt *LEDI2C) FlashConfig(pol Polarity) error {

	if ok, err := alt.valid(); !ok {
		return err
	}

	if err := alt.GPIO.FlashConfig(pinLEDI2C, WordClr, ModeLEDI2C, DirOutput); nil != err {
		return fmt.Errorf("GPIO.FlashConfig(): %v", err)
	}

	if cmd, err := alt.flash.chipSettings(true); nil != err {
		return fmt.Errorf("flash.chipSettings(): %v", err)
	} else {

		cmd[2] &= ^byte(0x01 << 4) // mask off the LED_I2C bit (4)

		// if pol is false, we set the LED_I2C bit (the device will use the negated
		// value of this bit during active traffic). otherwise, we leave the bit
		// clear.
		if !pol {
			cmd[2] |= byte(0x01 << 4)
		}

		if err := alt.flash.write(subcmdChipSettings, cmd); nil != err {
			return fmt.Errorf("flash.write(): %v", err)
		}
	}

	return nil
}

// GetConfig returns the current polarity selection of the LED_I2C operation
// mode.
// A positive (true) polarity means the LED_I2C-capable pin will be driven high
// when I2C traffic is active, and negative (false) with no traffic.
// A negative (false) polarity drives the LED_I2C-capable pin opposite the
// behavior of positive (true) polarity.
// If the LED_I2C-capable pin has not been configured for LED_I2C operation
// mode, this polarity configuration has no effect.
//
// Returns false and an error if the receiver is invalid or if the current
// configuration could not be read.
func (alt *LEDI2C) GetConfig() (Polarity, error) {

	if ok, err := alt.valid(); !ok {
		return false, err
	}

	if buf, err := alt.sram.readRange(4, 4); nil != err {
		return false, fmt.Errorf("sram.readRange(): %v", err)
	} else {
		pol := Polarity(0x00 == ((buf[0] >> 4) & 0x01))
		return pol, nil
	}
}

// -- LEDURX -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -

// LEDURX contains the methods associated with the LED_URX GP alternate
// function.
type LEDURX struct {
	*MCP2221A
}

// FlashConfig configures the sole LED_URX-capable GPIO pin's operation mode for
// LED_URX, sets the LEDURX module's default polarity to the given pol, and then
// writes these settings to flash memory.
// A positive (true) polarity means the LED_URX-capable pin will be driven high
// when UART Rx traffic is active, and negative (false) with no traffic.
// A negative (false) polarity drives the LED_URX-capable pin opposite the
// behavior of positive (true) polarity.
// These settings become default but only take effect after next startup/reset.
//
// Returns an error if the receiver is invalid, the pin's operation mode could
// not be set, or the flash settings could not be read from or written to.
func (alt *LEDURX) FlashConfig(pol Polarity) error {

	if ok, err := alt.valid(); !ok {
		return err
	}

	if err := alt.GPIO.FlashConfig(pinLEDURX, WordClr, ModeLEDURX, DirOutput); nil != err {
		return fmt.Errorf("GPIO.FlashConfig(): %v", err)
	}

	if cmd, err := alt.flash.chipSettings(true); nil != err {
		return fmt.Errorf("flash.chipSettings(): %v", err)
	} else {

		cmd[2] &= ^byte(0x01 << 6) // mask off the LED_URX bit (6)

		// if pol is false, we set the LED_URX bit (the device will use the negated
		// value of this bit during active UART Rx traffic). otherwise, we leave the
		// bit clear.
		if !pol {
			cmd[2] |= byte(0x01 << 6)
		}

		if err := alt.flash.write(subcmdChipSettings, cmd); nil != err {
			return fmt.Errorf("flash.write(): %v", err)
		}
	}

	return nil
}

// GetConfig returns the current polarity selection of the LED_URX operation
// mode.
// A positive (true) polarity means the LED_URX-capable pin will be driven high
// when UART Rx traffic is active, and negative (false) with no traffic.
// A negative (false) polarity drives the LED_URX-capable pin opposite the
// behavior of positive (true) polarity.
// If the LED_URX-capable pin has not been configured for LED_URX operation
// mode, this polarity configuration has no effect.
//
// Returns false and an error if the receiver is invalid or if the current
// configuration could not be read.
func (alt *LEDURX) GetConfig() (Polarity, error) {

	if ok, err := alt.valid(); !ok {
		return false, err
	}

	if buf, err := alt.sram.readRange(4, 4); nil != err {
		return false, fmt.Errorf("sram.readRange(): %v", err)
	} else {
		pol := Polarity(0x00 == ((buf[0] >> 6) & 0x01))
		return pol, nil
	}
}

// -- LEDUTX -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -

// LEDUTX contains the methods associated with the LED_UTX GP alternate
// function.
type LEDUTX struct {
	*MCP2221A
}

// FlashConfig configures the sole LED_UTX-capable GPIO pin's operation mode for
// LED_UTX, sets the LEDUTX module's default polarity to the given pol, and then
// writes these settings to flash memory.
// A positive (true) polarity means the LED_UTX-capable pin will be driven high
// when UART Tx traffic is active, and negative (false) with no traffic.
// A negative (false) polarity drives the LED_UTX-capable pin opposite the
// behavior of positive (true) polarity.
// These settings become default but only take effect after next startup/reset.
//
// Returns an error if the receiver is invalid, the pin's operation mode could
// not be set, or the flash settings could not be read from or written to.
func (alt *LEDUTX) FlashConfig(pol Polarity) error {

	if ok, err := alt.valid(); !ok {
		return err
	}

	if err := alt.GPIO.FlashConfig(pinLEDUTX, WordClr, ModeLEDUTX, DirOutput); nil != err {
		return fmt.Errorf("GPIO.FlashConfig(): %v", err)
	}

	if cmd, err := alt.flash.chipSettings(true); nil != err {
		return fmt.Errorf("flash.chipSettings(): %v", err)
	} else {

		cmd[2] &= ^byte(0x01 << 5) // mask off the LED_UTX bit (5)

		// if pol is false, we set the LED_UTX bit (the device will use the negated
		// value of this bit during active UART Tx traffic). otherwise, we leave the
		// bit clear.
		if !pol {
			cmd[2] |= byte(0x01 << 5)
		}

		if err := alt.flash.write(subcmdChipSettings, cmd); nil != err {
			return fmt.Errorf("flash.write(): %v", err)
		}
	}

	return nil
}

// GetConfig returns the current polarity selection of the LED_UTX operation
// mode.
// A positive (true) polarity means the LED_UTX-capable pin will be driven high
// when UART Tx traffic is active, and negative (false) with no traffic.
// A negative (false) polarity drives the LED_UTX-capable pin opposite the
// behavior of positive (true) polarity.
// If the LED_UTX-capable pin has not been configured for LED_UTX operation
// mode, this polarity configuration has no effect.
//
// Returns false and an error if the receiver is invalid or if the current
// configuration could not be read.
func (alt *LEDUTX) GetConfig() (Polarity, error) {

	if ok, err := alt.valid(); !ok {
		return false, err
	}

	if buf, err := alt.sram.readRange(4, 4); nil != err {
		return false, fmt.Errorf("sram.readRange(): %v", err)
	} else {
		pol := Polarity(0x00 == ((buf[0] >> 5) & 0x01))
		return pol, nil
	}
}

// -- Alt ------------------------------------------------------------- [end] --
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// -- I²C ----------------------------------------------------------- [start] --

// I2C contains the methods associated with the I²C module of the MCP2221A.
type I2C struct {
	*MCP2221A
}

// Constants associated with the I²C module.
const (
	I2CBaudRate = 100000 // default baud rate
	I2CMinAddr  = 0x08   // minimum possible 7-bit address
	I2CMaxAddr  = 0x77   // maximum possible (unreserved) 7-bit address
)

// Private constants associated with the I²C module.
const (
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

// i2cStateNACK tests if the given internal I²C state machine status indicates
// an I²C NACK from a requested slave address.
// This is considered an I²C fatal error.
func i2cStateNACK(state byte) bool {
	return (i2cStateAddrNACK == state)
}

// i2cStateTimeout tests if the given internal I²C state machine status
// indicates any type of I²C communication timeout.
// This is considered an I²C fatal error.
func i2cStateTimeout(state byte) bool {
	return (i2cStateStartTimeout == state) ||
		(i2cStateRepStartTimeout == state) ||
		(i2cStateStopTimeout == state) ||
		(i2cStateReadTimeout == state) ||
		(i2cStateWriteTimeout == state) ||
		(i2cStateAddrTimeout == state)
}

// SetConfig configures the I²C bus clock divider calculated from a given baud
// rate (BPS). If in doubt, use global constant I2CBaudRate.
// These settings only affect the current device configuration and are not
// retained after next startup/reset.
//
// Returns an error if the receiver is invalid, the given baud rate is invalid,
// the set-parameters command could not be sent, or if an I²C transfer is
// currently in-progress.
func (mod *I2C) SetConfig(baud uint32) error {

	if ok, err := mod.valid(); !ok {
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

	if rsp, err := mod.send(cmdSetParams, cmd); nil != err {
		return fmt.Errorf("send(): %v", err)
	} else {
		stat := parseStatus(rsp)
		if 0x21 == stat.i2cSpdChg {
			return fmt.Errorf("transfer in progress")
		}
	}

	return nil
}

// Cancel sends a set-parameters command to cancel any ongoing I²C transfer
// currently in progress.
//
// Returns an error if the receiver is invalid, or if the command could not be
// sent.
func (mod *I2C) Cancel() error {

	if ok, err := mod.valid(); !ok {
		return err
	}

	cmd := makeMsg()
	cmd[2] = 0x10

	if rsp, err := mod.send(cmdSetParams, cmd); nil != err {
		return fmt.Errorf("send(): %v", err)
	} else {
		stat := parseStatus(rsp)
		if 0x10 == stat.i2cCancel {
			time.Sleep(300 * time.Microsecond)
		}
	}

	return nil
}

// Write is the general-purpose function for writing raw data directly to the
// I²C data bus.
// If argument stop is true, then an I²C STOP condition is generated on the bus
// once the bytes transmitted equals the number bytes specified as parameter cnt
// (this is the "usual" case). Otherwise, the STOP condition is not generated,
// and the bus remains "active" for subsequent I/O.
//
// Returns an error if any of the following occur: invalid receiver, could not
// read status message, could not cancel an existing I²C connection (if exists),
// could not send command message, the I²C state machine enters an unrecoverable
// state, or too many retries were attempted.
func (mod *I2C) Write(stop bool, addr uint8, out []byte, cnt uint16) error {

	if ok, err := mod.valid(); !ok {
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

	if stat, err = mod.status(); nil != err {
		return fmt.Errorf("status(): %v", err)
	}

	if WordClr != stat.i2cState {
		if err := mod.I2C.Cancel(); nil != err {
			return fmt.Errorf("I2C.Cancel(): %v", err)
		}
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
			retry++
			if rsp, err := mod.send(sendCMD, cmd); nil != err {
				if nil != rsp {
					if i2cStateNACK(rsp[2]) {
						return fmt.Errorf("send(): I²C NACK from address (0x%02X)", addr)
					}
					if i2cStateTimeout(rsp[2]) {
						return fmt.Errorf("send(): I²C write timed out")
					}
				} else {
					return fmt.Errorf("send(): %v", err)
				}
				time.Sleep(300 * time.Microsecond)
			} else {
				partialData := func(im *I2C) bool {
					stat, err := im.status()
					// the error isn't handled here, we just break out of the loops. if it
					// persists, the I²C state loop below will return it to the caller.
					return (nil == err) && (i2cStatePartialData == stat.i2cState)
				}
				for partialData(mod) {
					time.Sleep(300 * time.Microsecond)
				}
				pos += sz
				break
			}
		}
		if retry >= i2cWriteRetry {
			return fmt.Errorf("too many retries")
		}
	}

	retry = 0
	for retry < i2cWriteRetry {
		retry++
		if stat, err := mod.status(); nil != err {
			return fmt.Errorf("status(): %v", err)
		} else {
			if WordClr == stat.i2cState {
				break
			}
			if (cmdI2CWriteNoStop == cmdID) && (i2cStateWritingNoStop == stat.i2cState) {
				break
			}
			if i2cStateNACK(stat.i2cState) {
				return fmt.Errorf("status(): I²C NACK from address (0x%02X)", addr)
			}
			if i2cStateTimeout(stat.i2cState) {
				return fmt.Errorf("status(): I²C write timed out")
			}
			time.Sleep(300 * time.Microsecond)
		}
	}

	return nil
}

// Read is the general purpose function for reading raw data directly from the
// I²C data bus.
// If argument rep is true, a REP-START condition is generated (instead of the
// usual START condition) to indicate we are reading data from a slave
// subaddress configured before this call to Read().
//
// Returns the data slice of length cnt (bytes) read from the bus if there was
// no error. Otherwise, an error is returned if any of the following occur:
// invalid receiver, could not read status message, could not cancel an existing
// I²C connection (if exists), could not send command message, the I²C state
// machine enters an unrecoverable state.
func (mod *I2C) Read(rep bool, addr uint8, cnt uint16) ([]byte, error) {

	if ok, err := mod.valid(); !ok {
		return nil, err
	}

	if cnt <= 0 {
		return []byte{}, nil
	}

	var (
		stat *status
		err  error
	)

	if stat, err = mod.status(); nil != err {
		return nil, fmt.Errorf("status(): %v", err)
	}

	if WordClr != stat.i2cState && i2cStateWritingNoStop != stat.i2cState {
		if err := mod.I2C.Cancel(); nil != err {
			return nil, fmt.Errorf("I2C.Cancel(): %v", err)
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

	if _, err := mod.send(cmdID, cmd); nil != err {
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
			if rsp, err = mod.send(cmdI2CReadGetData, cmd); nil != err {
				return nil, fmt.Errorf("send(): %v", err)
			} else {
				if i2cStatePartialData == rsp[1] || i2cStateReadError == rsp[3] {
					time.Sleep(300 * time.Microsecond)
					continue
				}
				if i2cStateNACK(rsp[2]) {
					return nil, fmt.Errorf("send(): I²C NACK from address (0x%02X)", addr)
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

// ReadReg performs a standard write-then-read I²C operation as a convenience
// for the common case of reading registers.
// This variant is for target devices with 8-bit subaddress widths (i.e. the
// size of the register pointer).
//
// Returns the bytes received on success, or return error if either write or
// read failures occurred.
//
// See also I2CReadReg16() for 16-bit subaddressing devices.
func (mod *I2C) ReadReg(addr uint8, reg uint8, cnt uint16) ([]byte, error) {

	if err := mod.I2C.Write(false, addr, []byte{reg}, 1); nil != err {
		return nil, fmt.Errorf("I2C.Write(): %v", err)
	}

	if reg, err := mod.I2C.Read(true, addr, cnt); nil != err {
		return nil, fmt.Errorf("I2C.Read(): %v", err)
	} else {
		return reg, nil
	}
}

// ReadReg16 performs a standard write-then-read I²C operation as a convenience
// for the common case of reading registers.
// This variant is for target devices with 16-bit subaddress widths (i.e. the
// size of the register pointer). If argument msb is true, then the buffer
// containing the register pointer is reversed so that the MSByte is at buffer
// index 0.
//
// Returns the bytes received on success, or returns an error if either write or
// read failures occurred.
//
// See also I2CReadReg() for 8-bit subaddressing devices.
func (mod *I2C) ReadReg16(addr uint8, reg uint16, msb bool, cnt uint16) ([]byte, error) {

	buf := []byte{byte(reg & 0xFF), byte((reg >> 8) & 0xFF)}
	if msb {
		buf = []byte{buf[1], buf[0]}
	}

	if err := mod.I2C.Write(false, addr, buf, 2); nil != err {
		return nil, fmt.Errorf("I2C.Write(): %v", err)
	}

	if reg, err := mod.I2C.Read(true, addr, cnt); nil != err {
		return nil, fmt.Errorf("I2C.Read(): %v", err)
	} else {
		return reg, nil
	}
}

// ReadReady tests if the device needs to perform a read from a requested
// slave device.
//
// Returns false and an error if the receiver is invalid or if the I²C state
// machine status could not be read.
func (mod *I2C) ReadReady() (bool, error) {

	if ok, err := mod.valid(); !ok {
		return false, err
	}

	if stat, err := mod.status(); nil != err {
		return false, fmt.Errorf("status(): %v", err)
	} else {
		return stat.i2cReadPnd > 0, nil
	}
}

// Scan scans a given address range and attempts to communicate with each
// device, ignoring any failures caused by non-existent targets.
//
// Returns a byte slice of 7-bit addresses known to be online and able to be
// communicated with.
// Returns a nil slice and error if the receiver is invalid or given address
// range is invalid.
func (mod *I2C) Scan(start uint8, stop uint8) ([]uint8, error) {

	if ok, err := mod.valid(); !ok {
		return nil, err
	}

	if start > stop {
		return nil, fmt.Errorf("invalid address range [%d, %d]", start, stop)
	}

	found := []byte{}
	for addr := start; addr <= stop; addr++ {
		if err := mod.I2C.Write(true, addr, []byte{0x00}, 1); nil == err {
			found = append(found, byte(addr))
		}
	}

	return found, nil
}

// -- I²C ------------------------------------------------------------- [end] --
// -----------------------------------------------------------------------------
