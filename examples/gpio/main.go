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

	pin := byte(0)

	// set pin 0 to GPIO output mode
	if err := m.GPIO.SetConfig(pin, 0, mcp.ModeGPIO, mcp.DirOutput); nil != err {
		log.Fatalf("GPIO.SetConfig(): %v", err)
	}

	// repeatedly toggle the pin, and then read and print its value, every 500 ms
	var val byte = 1
	for {
		// set output value
		if err := m.GPIO.Set(pin, val); nil != err {
			log.Fatalf("GPIO..Set(): %v", err)
		}
		// get current value
		if get, err := m.GPIO.Get(pin); nil != err {
			log.Fatalf("GPIO.Get(): %v", err)
		} else {
			log.Printf("Pin[%d] = %d", pin, get)
		}
		val = 1 - val
		time.Sleep(500 * time.Millisecond)
	}
}
