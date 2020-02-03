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

	// reset device to default settings stored in flash memory
	if err := m.Reset(5 * time.Second); nil != err {
		log.Fatalf("Reset(): %v", err)
	}

	pin := byte(2)

	// set pin 2 to DAC mode
	if err := m.DACSetConfig(pin, mcp.VRefDefault); nil != err {
		log.Fatalf("DACSetConfig(): %v", err)
	}

	// repeatedly write and print an incrementing 5-bit value every 100 ms
	val := uint16(0)
	for {
		val = (val + 1) % 0x20 // 5-bit maximum
		if err := m.DACWrite(val); nil != err {
			log.Fatalf("DACWrite(): %v", err)
		} else {
			log.Printf("Pin[%d] = %d", pin, val)
		}
		time.Sleep(100 * time.Millisecond)
	}
}
