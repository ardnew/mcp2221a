package main

import (
	"log"

	mcp "github.com/ardnew/mcp2221a"
)

func main() {
	m, e := mcp.NewMCP2221A(mcp.VID, mcp.PID)
	if nil != e {
		log.Fatalf("failed to open MCP2221A: %v", e)
	}
	m.Close()
}
