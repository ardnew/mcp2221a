package mcp2221a

import (
	"fmt"
	//"log"

	usb "github.com/google/gousb"
)

const (
	VID = usb.ID(0x04D8) // Microchip Technology Inc.
	PID = usb.ID(0x00DD)
)

type USBModule struct {
	in  *usb.InEndpoint
	out *usb.OutEndpoint
}

type MCP2221A struct {
	ctx *usb.Context
	dev *usb.Device
	cdc *USBModule
	hid *USBModule
	vid usb.ID
	pid usb.ID
}

func NewMCP2221A(vid usb.ID, pid usb.ID) (*MCP2221A, error) {

	m := &MCP2221A{}

	m.ctx = usb.NewContext()

	if dev, err := m.ctx.OpenDeviceWithVIDPID(usb.ID(vid), usb.ID(pid)); nil != err {
		return nil, err
	} else {
		m.dev = dev
	}

	m.ctx.Debug(2)

	m.dev.SetAutoDetach(true)

	m.cdc = &USBModule{}
	m.hid = &USBModule{}
	m.vid = usb.ID(vid)
	m.pid = usb.ID(pid)

	for _, cfgDesc := range m.dev.Desc.Configs {

		cfg, e := m.dev.Config(cfgDesc.Number)
		if nil != e {
			return nil, e
		}

		for _, ifaceDesc := range cfgDesc.Interfaces {
			for _, altDesc := range ifaceDesc.AltSettings {

				var (
					mod *USBModule = nil
				)

				switch altDesc.Class {
				case usb.ClassHID:
					mod = m.hid
				case usb.ClassData:
					mod = m.cdc
				}

				if nil != mod {

					iface, err := cfg.Interface(ifaceDesc.Number, altDesc.Alternate)
					if nil != err {
						return nil, err
					}

					for _, endDesc := range altDesc.Endpoints {

						switch endDesc.Direction {
						case usb.EndpointDirectionIn:

							mod.in, err = iface.InEndpoint(endDesc.Number)
							if nil != err {
								return nil, err
							}

						case usb.EndpointDirectionOut:

							mod.out, err = iface.OutEndpoint(endDesc.Number)
							if nil != err {
								return nil, err
							}

						}
					}
				}
			}
		}
	}

	return m, nil
}

func (m *MCP2221A) Close() error {
	if nil != m {
		var err error = nil

		// close the device
		if nil != m.dev {
			if e := m.dev.Close(); nil != e {
				if nil == err {
					err = e
				}
			}
			m.dev = nil
		} else {
			if nil == err {
				err = fmt.Errorf("libusb device already closed")
			}
		}

		// close the context
		if nil != m.ctx {
			if e := m.ctx.Close(); nil != e {
				if nil == err {
					err = e
				}
			}
			m.ctx = nil
		} else {
			if nil == err {
				err = fmt.Errorf("libusb context already closed")
			}
		}
		return err
	}

	return fmt.Errorf("invalid receiver")
}
