package mcp2221a

import (
	"fmt"
	"io"
	"log"

	usb "github.com/google/gousb"
)

// factory default identification and configuration of USB descriptors
const (
	VID          = 0x04D8 // Microchip Technology Inc.
	PID          = 0x00DD
	DeviceConfig = 1
	CDCInterface = 1
	CDCAlternate = 0
	CDCEndpoint  = 2
	HIDInterface = 2
	HIDAlternate = 0
	HIDEndpoint  = 3
)

type Interface struct {
	iface *usb.Interface
	in    *usb.InEndpoint
	out   *usb.OutEndpoint
}

type MCP2221A struct {
	ctx *usb.Context
	dev *usb.Device
	cfg *usb.Config
	cdc *Interface
	hid *Interface
	vid usb.ID
	pid usb.ID
}

func newInterface(cfg *usb.Config, inf int, alt int, ep int) (*Interface, error) {

	var err error

	i := &Interface{}

	if i.iface, err = cfg.Interface(inf, alt); nil != err {
		return nil, err
	}
	defer i.iface.Close()

	if i.in, err = i.iface.InEndpoint(ep); nil != err {
		return nil, err
	}
	if i.out, err = i.iface.OutEndpoint(ep); nil != err {
		return nil, err
	}

	return i, nil
}

func NewMCP2221A(vid uint16, pid uint16) (*MCP2221A, error) {

	var err error

	mcp := &MCP2221A{
		ctx: usb.NewContext(),
	}

	defer mcp.ctx.Close()

	// Iterate through available Devices, finding all that match a known VID/PID.
	v, p := usb.ID(0x04f2), usb.ID(0xb531)
	devs, err := mcp.ctx.OpenDevices(func(desc *usb.DeviceDesc) bool {
		// this function is called for every device present.
		// Returning true means the device should be opened.
		log.Printf("Vendor(%d)==(%d), Product(%d)==(%d)", desc.Vendor, v, desc.Product, p)
		return desc.Vendor == v && desc.Product == p
	})
	// All returned devices are now open and will need to be closed.
	for _, d := range devs {
		defer d.Close()
	}
	if err != nil {
		log.Fatalf("OpenDevices(): %v", err)
	}
	if len(devs) == 0 {
		log.Fatalf("no devices found matching VID %s and PID %s", vid, pid)
	}

	// Pick the first device found.
	mcp.dev = devs[0]

	// attempt to use only the provided VID/PID
	//mcp.dev, err = mcp.ctx.OpenDeviceWithVIDPID(usb.ID(vid), usb.ID(pid))
	//if nil != err {
	//	return nil, err
	//}
	//if nil == mcp.dev {
	//	return nil, fmt.Errorf("no device found with VID=%d, PID=%d", usb.ID(vid), usb.ID(pid))
	//}

	mcp.cfg, err = mcp.dev.Config(DeviceConfig)
	if nil != err {
		return nil, err
	}

	// initialize the CDC interface (UART) with In/OutEndpoints
	mcp.cdc, err = newInterface(mcp.cfg, CDCInterface, CDCAlternate, CDCEndpoint)
	if nil != err {
		return nil, err
	}

	// initialize the HID interface (I2C/GPIO/INT) with In/OutEndpoints
	mcp.hid, err = newInterface(mcp.cfg, HIDInterface, HIDAlternate, HIDEndpoint)
	if nil != err {
		return nil, err
	}

	return mcp, nil
}

func (mcp *MCP2221A) Close() (err error) {
	ioClose := func(c io.Closer) {
		if nil != c {
			if e := c.Close(); nil != e {
				if nil == err {
					err = e
				}
			}
		}
	}
	if nil != mcp {
		defer ioClose(mcp.cfg)
		defer ioClose(mcp.dev)
		defer ioClose(mcp.ctx)
	} else {
		err = fmt.Errorf("nil MCP2221A")
	}
	return
}
