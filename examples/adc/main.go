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

	pin := byte(1)

	// set pin 1 to ADC mode
	if err := m.ADC.SetConfig(pin, mcp.VRefDefault); nil != err {
		log.Fatalf("ADC.SetConfig(): %v", err)
	}

	// repeatedly read and print the ADC converted value every 100 ms
	for {
		if val, err := m.ADC.Read(pin); nil != err {
			log.Fatalf("ADC.Read(): %v", err)
		} else {
			log.Printf("Pin[%d] = %d", pin, val)
		}
		time.Sleep(100 * time.Millisecond)
	}
}
