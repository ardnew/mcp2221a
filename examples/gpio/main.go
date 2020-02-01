package main

import (
	"log"
	"time"

	mcp "github.com/ardnew/mcp2221a"
)

func main() {
	m, err := mcp.NewMCP2221A(0, mcp.VID, mcp.PID)
	if nil != err {
		log.Fatalf("Open(): %v", err)
	}
	defer m.Close()

	// set pin 0 to GPIO output mode
	if err := m.ConfigSetGPIO(0, mcp.ModeGPIO, mcp.DirOut, 0); nil != err {
		log.Fatalf("ConfigSetGPIO(): %v", err)
	}

	var val byte = 1
	for {
		if err := m.GPIOSet(0, val); nil != err {
			log.Fatalf("GPIOSet(): %v", err)
		}
		val = 1 - val
		time.Sleep(500 * time.Millisecond)
	}
}
