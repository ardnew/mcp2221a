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

	if err := m.Reset(5 * time.Second); nil != err {
		log.Fatalf("Reset(): %v", err)
	}

	pin := uint8(0)

	// set pin 0 to GPIO output mode
	if err := m.GPIOSetConfig(pin, 0, mcp.ModeGPIO, mcp.DirOut); nil != err {
		log.Fatalf("GPIOSetConfig(): %v", err)
	}

	var val byte = 1
	for {
		if err := m.GPIOSet(pin, val); nil != err {
			log.Fatalf("GPIOSet(): %v", err)
		}
		if get, err := m.GPIOGet(pin); nil != err {
			log.Fatalf("GPIOGet(): %v", err)
		} else {
			log.Printf("Pin[%d] = %d", pin, get)
		}
		val = 1 - val
		time.Sleep(500 * time.Millisecond)
	}
}
