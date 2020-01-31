package mcp2221a

import (
	"fmt"

	usb "github.com/karalabe/hid"
)

// factory default identification and configuration of USB descriptors
const (
	VID = 0x04D8 // Microchip Technology Inc.
	PID = 0x00DD

	PktSz = 64 // size of command/response buffers (bytes)
)

func cmdBuf() []byte { return make([]byte, PktSz) }
func rspBuf() []byte { return make([]byte, PktSz) }

const (
	CmdStatus    byte = 0x10
	CmdSetParams byte = 0x10

	CmdFlashRead   byte = 0xB0
	CmdFlashWrite  byte = 0xB1
	CmdFlashPasswd byte = 0xB2

	CmdI2CWrite         byte = 0x90
	CmdI2CWriteRepStart byte = 0x92
	CmdI2CWriteNoStop   byte = 0x94
	CmdI2CRead          byte = 0x91
	CmdI2CReadRepStart  byte = 0x93
	CmdI2CReadGetData   byte = 0x40

	CmdGPIOSet byte = 0x50
	CmdGPIOGet byte = 0x51

	CmdSRAMSet byte = 0x60
	CmdSRAMGet byte = 0x61

	CmdReset byte = 0x70
)

// -- DEVICE -------------------------------------------------------- [start] --

type MCP2221A struct {
	Device *usb.Device
	VID    uint16
	PID    uint16
}

func AttachedDevices(vid uint16, pid uint16) []usb.DeviceInfo {

	var info []usb.DeviceInfo

	for _, i := range usb.Enumerate(vid, pid) {
		info = append(info, i)
	}

	return info
}

func NewMCP2221A(idx uint8, vid uint16, pid uint16) (*MCP2221A, error) {

	mcp := &MCP2221A{
		Device: nil,
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

func (mcp *MCP2221A) isValid() (bool, error) {

	if nil == mcp {
		return false, fmt.Errorf("nil MCP2221A")
	}

	if nil == mcp.Device {
		return false, fmt.Errorf("nil MCP2221A HID device")
	}

	return true, nil
}

func (mcp *MCP2221A) Close() error {

	if ok, err := mcp.isValid(); !ok {
		return err
	}

	if err := mcp.Device.Close(); nil != err {
		return err
	}
	return nil
}

func (mcp *MCP2221A) SendCmd(cmd byte, data []byte) ([]byte, error) {

	if ok, err := mcp.isValid(); !ok {
		return nil, err
	}

	data[0] = cmd
	if _, err := mcp.Device.Write(data); nil != err {
		return nil, fmt.Errorf("Write([cmd=0x%02X]): %v", cmd, err)
	}

	rsp := rspBuf()
	if recv, err := mcp.Device.Read(rsp); nil != err {
		return nil, fmt.Errorf("Read([cmd=0x%02X]): %v", cmd, err)
	} else {
		if recv < PktSz {
			return nil, fmt.Errorf("Read([cmd=0x%02X]): short read (%d of %d bytes)", cmd, recv, PktSz)
		}
	}

	return rsp, nil
}

func (mcp *MCP2221A) Reset() error {

	if ok, err := mcp.isValid(); !ok {
		return err
	}

	cmd := cmdBuf()
	cmd[1] = 0xAB
	cmd[2] = 0xCD
	cmd[3] = 0xEF

	if _, err := mcp.SendCmd(CmdReset, cmd); nil != err {
		return fmt.Errorf("SendCmd(): %v", err)
	}

	return nil
}

type Status struct {
	Cmd        byte
	OK         bool
	I2CCancel  byte
	I2CSpeedCh byte
	I2CDivCh   byte
	I2CState   byte
	I2CReqSz   uint16
	I2CSentSz  uint16
	I2CCounter byte
	I2CDiv     byte
	I2CTimeVal byte
	I2CAddr    uint16
	I2CSCL     byte
	I2CSDA     byte
	Interrupt  byte
	I2CReadPnd byte
	HWRevMaj   rune
	HWRevMin   rune
	FWRevMaj   rune
	FWRevMin   rune
	ADCCh0     uint16
	ADCCh1     uint16
	ADCCh2     uint16
}

var (
	I2CCancelDesc = map[byte]string{
		0x00: "idle",
		0x10: "marked for cancellation",
		0x11: "no transfer (ignored)",
	}
	I2CSpeedChDesc = map[byte]string{
		0x00: "idle",
		0x20: "new speed accepted",
		0x21: "cannot set speed",
	}
)

func (mcp *MCP2221A) Status() (*Status, error) {

	if ok, err := mcp.isValid(); !ok {
		return nil, err
	}

	cmd := cmdBuf()

	if rsp, err := mcp.SendCmd(CmdStatus, cmd); nil != err {

		return nil, fmt.Errorf("SendCmd(): %v", err)

	} else {

		return &Status{
			Cmd:        rsp[0],
			OK:         (0 == rsp[1]),
			I2CCancel:  rsp[2],
			I2CSpeedCh: rsp[3],
			I2CDivCh:   rsp[4],
			// bytes 5-7 reserved
			I2CState:   rsp[8],
			I2CReqSz:   (uint16(rsp[10]) << 8) | uint16(rsp[9]),
			I2CSentSz:  (uint16(rsp[12]) << 8) | uint16(rsp[11]),
			I2CCounter: rsp[13],
			I2CDiv:     rsp[14],
			I2CTimeVal: rsp[15],
			I2CAddr:    (uint16(rsp[17]) << 8) | uint16(rsp[16]),
			// bytes 18-21 reserved
			I2CSCL:     rsp[22],
			I2CSDA:     rsp[23],
			Interrupt:  rsp[24],
			I2CReadPnd: rsp[25],
			// bytes 26-45 reserved
			HWRevMaj: rune(rsp[46]),
			HWRevMin: rune(rsp[47]),
			FWRevMaj: rune(rsp[48]),
			FWRevMin: rune(rsp[49]),
			ADCCh0:   (uint16(rsp[51]) << 8) | uint16(rsp[50]),
			ADCCh1:   (uint16(rsp[53]) << 8) | uint16(rsp[52]),
			ADCCh2:   (uint16(rsp[55]) << 8) | uint16(rsp[54]),
		}, nil
	}
}

// -- DEVICE ---------------------------------------------------------- [end] --

// -- GPIO ---------------------------------------------------------- [start] --

type (
	GPIOMode byte
	GPIODir  bool
)

const (
	GPIOPinCount = 4
	// GPIO operation modes
	ModeGPIO     GPIOMode = 0x00
	ModeDediFunc GPIOMode = 0x01
	ModeAltFunc0 GPIOMode = 0x02
	ModeAltFunc1 GPIOMode = 0x03
	ModeAltFunc2 GPIOMode = 0x04
	ModeAltFunc3 GPIOMode = 0x05
	// GPIO directions
	DirOut GPIODir = false
	DirIn  GPIODir = true
)

var (
	GPIODirValue = map[GPIODir]byte{false: 0, true: 1}
)

func (mcp *MCP2221A) GPIOSet(pin int, val byte) error {

	if ok, err := mcp.isValid(); !ok {
		return err
	}

	if pin < 0 || pin >= GPIOPinCount {
		return fmt.Errorf("invalid GPIO pin: %d", pin)
	}

	cmd := cmdBuf()

	i := 2 + 4*pin
	cmd[i+0] = 0xFF
	cmd[i+1] = val
	cmd[i+2] = 0xFF
	cmd[i+3] = 0

	if _, err := mcp.SendCmd(CmdGPIOSet, cmd); nil != err {
		return fmt.Errorf("SendCmd(): %v", err)
	}

	return nil
}

// -- GPIO ------------------------------------------------------------ [end] --

// -- SRAM ---------------------------------------------------------- [start] --

func (mcp *MCP2221A) SRAMSetGPIO(pin uint8, mode GPIOMode, dir GPIODir, val byte) error {

	if ok, err := mcp.isValid(); !ok {
		return err
	}

	if pin < 0 || pin >= GPIOPinCount {
		return fmt.Errorf("invalid GPIO pin: %d", pin)
	}

	cmd := cmdBuf()

	if cur, err := mcp.SRAMGet(22, 25); nil != err {
		return fmt.Errorf("SRAMGet(): %v", err)
	} else {
		// copy the current GPIO settings because they will -all- be set with the
		// command request
		cmd[7] = 0xFF // alter GP designation
		cmd[8] = cur[0]
		cmd[9] = cur[1]
		cmd[10] = cur[2]
		cmd[11] = cur[3]
	}

	// and then update our selected pin as desired
	cmd[8+pin] = (val << 4) | (GPIODirValue[dir] << 3) | byte(mode)

	if _, err := mcp.SendCmd(CmdSRAMSet, cmd); nil != err {
		return fmt.Errorf("SendCmd(): %v", err)
	}

	return nil
}

func (mcp *MCP2221A) SRAMGet(start uint8, stop uint8) ([]byte, error) {

	if ok, err := mcp.isValid(); !ok {
		return nil, err
	}

	if (start > stop) || (stop >= PktSz) {
		return nil, fmt.Errorf("invalid byte range: [%d, %d]", start, stop)
	}

	cmd := cmdBuf()
	if rsp, err := mcp.SendCmd(CmdSRAMGet, cmd); nil != err {
		return nil, fmt.Errorf("SendCmd(): %v", err)
	} else {
		return rsp[start : stop+1], nil
	}
}

// -- SRAM ------------------------------------------------------------ [end] --
