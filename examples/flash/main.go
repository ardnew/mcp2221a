package main

import (
	"log"
	"time"

	mcp "github.com/ardnew/mcp2221a"
)

func main() {

	m, err := mcp.New(0, mcp.VID, mcp.PID)
	if nil != err {
		log.Fatalf("Open(): %v", err)
	}
	defer m.Close()

	log.Print(mcp.PackageVersion())

	// reset device to default settings stored in flash memory
	if err := m.Reset(5 * time.Second); nil != err {
		log.Fatalf("Reset(): %v", err)
	}

	// print the description strings stored in flash memory
	if s, err := m.Flash.USBManufacturer(); nil != err {
		log.Fatalf("Flash.USBManufacturer(): %s", err)
	} else {
		log.Printf("USB Manufacturer  = \"%s\"", s)
	}
	if s, err := m.Flash.USBProduct(); nil != err {
		log.Fatalf("Flash.USBProduct(): %s", err)
	} else {
		log.Printf("USB Product       = \"%s\"", s)
	}
	if s, err := m.Flash.USBSerialNo(); nil != err {
		log.Fatalf("Flash.USBSerialNo(): %s", err)
	} else {
		log.Printf("USB Serial No     = \"%s\"", s)
	}
	if s, err := m.Flash.FactorySerialNo(); nil != err {
		log.Fatalf("Flash.FactorySerialNo(): %s", err)
	} else {
		log.Printf("Factory Serial No = \"%s\"", s)
	}

	pin := byte(0)

	log.Printf("setting pin %d = 0 (Flash, non-volatile)", pin)
	if err := m.GPIO.FlashConfig(pin, 0, mcp.ModeGPIO, mcp.DirOutput); nil != err {
		log.Fatalf("GPIO.FlashConfig(): %v", err)
	}
	if get, err := m.GPIO.Get(pin); nil != err {
		log.Fatalf("GPIO.Get(): %v", err)
	} else {
		log.Printf("Pin[%d] = %d", pin, get)
	}
	time.Sleep(2 * time.Second)

	// --

	log.Printf("setting pin %d = 1 (SRAM, volatile)", pin)
	if err := m.GPIO.SetConfig(pin, 1, mcp.ModeGPIO, mcp.DirOutput); nil != err {
		log.Fatalf("GPIO.SetConfig(): %v", err)
	}
	if get, err := m.GPIO.Get(pin); nil != err {
		log.Fatalf("GPIO.Get(): %v", err)
	} else {
		log.Printf("Pin[%d] = %d", pin, get)
	}
	time.Sleep(2 * time.Second)

	// --

	log.Printf("resetting device ...")
	if err := m.Reset(5 * time.Second); nil != err {
		log.Fatalf("Reset(): %v", err)
	}
	if get, err := m.GPIO.Get(pin); nil != err {
		log.Fatalf("GPIO.Get(): %v", err)
	} else {
		log.Printf("Pin[%d] = %d", pin, get)
	}
	time.Sleep(2 * time.Second)

	// --

	log.Printf("setting pin %d = 1 (Flash, non-volatile)", pin)
	if err := m.GPIO.FlashConfig(pin, 1, mcp.ModeGPIO, mcp.DirOutput); nil != err {
		log.Fatalf("GPIO.FlashConfig(): %v", err)
	}
	if get, err := m.GPIO.Get(pin); nil != err {
		log.Fatalf("GPIO.Get(): %v", err)
	} else {
		log.Printf("Pin[%d] = %d", pin, get)
	}
	time.Sleep(2 * time.Second)

	// --

	log.Printf("setting pin %d = 0 (SRAM, volatile)", pin)
	if err := m.GPIO.SetConfig(pin, 0, mcp.ModeGPIO, mcp.DirOutput); nil != err {
		log.Fatalf("GPIO.SetConfig(): %v", err)
	}
	if get, err := m.GPIO.Get(pin); nil != err {
		log.Fatalf("GPIO.Get(): %v", err)
	} else {
		log.Printf("Pin[%d] = %d", pin, get)
	}
	time.Sleep(2 * time.Second)

	// --

	log.Printf("resetting device ...")
	if err := m.Reset(5 * time.Second); nil != err {
		log.Fatalf("Reset(): %v", err)
	}
	if get, err := m.GPIO.Get(pin); nil != err {
		log.Fatalf("GPIO.Get(): %v", err)
	} else {
		log.Printf("Pin[%d] = %d", pin, get)
	}
}
