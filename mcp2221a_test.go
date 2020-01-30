package mcp2221a

import (
	"fmt"
	"testing"
)

func TestNewMCP2221A(t *testing.T) {

	type TC struct {
		vid uint16
		pid uint16
		err error
	}

	tc := []TC{
		{vid: VID, pid: PID, err: nil},
		{vid: 0, pid: 0, err: fmt.Errorf("libusb: not found [code -5]")},
	}

	for _, c := range tc {

		m, e := NewMCP2221A(c.vid, c.pid)
		d := fmt.Sprintf("NewMCP2221A(%d, %d) == (%+v, %+v)", c.vid, c.pid, m, e)

		if nil == c.err {
			// no error expected
			if (nil != m) && (nil == e) {
				// success
				t.Logf("[ ] PASS: %s", d)
			} else {
				// fail
				t.Errorf("[ ] FAIL: %s | {%+v}, {%+v} != {%+v}", d, m, e, c.err)
			}
		} else {
			// error expected
			if (nil == m) && (c.err.Error() == e.Error()) {
				// success
				t.Logf("[ ] PASS: %s", d)
			} else {
				// fail
				t.Errorf("[ ] FAIL: %s | {%+v}, {%+v} != {%+v}", d, m, e, c.err)
			}
		}

		if nil != m {
			m.Close()
		}
	}
}

func TestClose(t *testing.T) {

	type TC struct {
		vid  uint16
		pid  uint16
		err1 error
		err2 error
	}

	tc := []TC{
		{
			vid:  VID,
			pid:  PID,
			err1: nil,
			err2: fmt.Errorf("libusb context already closed"),
		},
		{
			vid:  0,
			pid:  0,
			err1: fmt.Errorf("nil MCP2221A"),
			err2: fmt.Errorf("nil MCP2221A"),
		},
	}

	for _, c := range tc {

		m, _ := NewMCP2221A(c.vid, c.pid)
		e1 := m.Close()
		e2 := m.Close()
		d1 := fmt.Sprintf("(%+v).Close() == %+v", m, e1)
		d2 := fmt.Sprintf("(%+v).Close() [x2] == %+v", m, e2)

		if nil == c.err1 {
			// no error expected
			if nil == e1 {
				// success
				t.Logf("[ ] PASS: %s", d1)
			} else {
				// fail
				t.Errorf("[ ] FAIL: %s | {%+v}, {%+v} != {%+v}", d1, m, e1, c.err1)
			}
		} else {
			// error expected
			if c.err1.Error() == e1.Error() {
				// success
				t.Logf("[ ] PASS: %s", d1)
			} else {
				// fail
				t.Errorf("[ ] FAIL: %s | {%+v}, {%+v} != {%+v}", d1, m, e1, c.err1)
			}
		}

		if nil == c.err2 {
			// no error expected
			if nil == e2 {
				// success
				t.Logf("[ ] PASS: %s", d2)
			} else {
				// fail
				t.Errorf("[ ] FAIL: %s | {%+v}, {%+v} != {%+v}", d2, m, e2, c.err2)
			}
		} else {
			// error expected
			if c.err2.Error() == e2.Error() {
				// success
				t.Logf("[ ] PASS: %s", d2)
			} else {
				// fail
				t.Errorf("[ ] FAIL: %s | {%+v}, {%+v} != {%+v}", d2, m, e2, c.err2)
			}
		}
	}
}
