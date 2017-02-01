package cc2538

import (
	"io"
	"fmt"
	"errors"
	"log"
	"time"
)

const (
	FLASH_BASE_ADDR uint32 = 0x00200000
	FLASH_PAGE_SIZE uint32 = 2048
	NUM_FLASH_PAGES uint32 = 256
	FLASH_CCA_PAGE uint32 = NUM_FLASH_PAGES - 1
	CCA_BASE_ADDR  uint32 = FLASH_BASE_ADDR + FLASH_CCA_PAGE * FLASH_PAGE_SIZE
	FLASH_CTRL_DIECFG0 uint32 = 0x400D3014
        FLASH_CTRL_DIECFG2 uint32 = 0x400D301C
        IEEE_ADDR uint32 = 0x00280028
	ACKWAIT = 3 // seconds to wait for an ack
)

type commandCode int

const (
	COMMAND_PING commandCode = 0x20
	COMMAND_GET_CHIP_ID commandCode = 0x28
	COMMAND_ERASE commandCode = 0x26
	COMMAND_CRC32 commandCode = 0x27
	COMMAND_DOWNLOAD commandCode = 0x21
	COMMAND_SEND_DATA commandCode = 0x24
	COMMAND_MEMORY_READ = 0x2a
	COMMAND_GET_STATUS commandCode = 0x23

	COMMAND_RET_SUCCESS commandCode = 0x40
	COMMAND_RET_UNKNOWN_CMD commandCode = 0x41
	COMMAND_RET_INVALID_CMD commandCode = 0x42
	COMMAND_RET_INVALID_ADR commandCode = 0x43
	COMMAND_RET_FLASH_FAIL commandCode = 0x44

)

var debug *log.Logger

func SetDebugLogger(logger *log.Logger) {
	debug = logger
}

type Serilzable interface {
	Serialize() []byte
}

type Packet struct {
	command commandCode
	payload []byte
}

// creates the packet format for a bootloader command
func (p Packet) Serialize() []byte {
	len := byte(3 + len(p.payload))

	sum := int(p.command)
	for _, val := range p.payload {
		sum += int(val)
	}
	chksum := byte(sum % 256)

	ret := []byte{len, chksum, byte(p.command)}
	ret = append(ret, p.payload...)
	debug.Println("packet serialize:", ret)
	return ret
}

type Frame []byte

// reads the port for incomming bytes and frames out responses
func ScanPort (port io.Reader, out chan Frame, kill chan bool) {
	debug.Println("port scanner started")

	// bring in a frame
	len := 0
	first := true
	var frame Frame
	for {
		select {
		case <- kill:
			debug.Println("port scanner killed")
			break
		default:
		}
		
		resp := make([]byte, 1)
		n, _ := port.Read(resp)
		if n == 0 { continue }

		if first {
			len = int(resp[0])
			// acks and naks send zero length
			// thanks TI
			if len == 0 {
				len = 1
			} else {
				len -= 1
			}
			first = false
		} else {
			len -= 1
		}

		frame = append(frame, resp[0])

		if len == 0 && !first {
			out <- frame
			frame = nil
			len = 0
			first = true
		}
	}
}

func (f Frame) isAck() bool {
	if f[1] == 0xcc {
		return true
	} else {
		return false
	}
}

func (f Frame) isNak() bool {
	if f[1] == 0x33 {
		return true
	} else {
		return false
	}
}

func (f Frame) chipID() int {
	r := int(f[4]) << 8 | int(f[5])
	return r
}

func needACK(frames chan Frame) error {
	select {
	case f := <- frames:
		if f.isAck() {
			debug.Println("got ACK [0xcc]")
			return nil
		} else if f.isNak() {
			debug.Println("got NAK [0x33]")
			return errors.New("got NAK needed ACK")
		} else {
			return errors.New(fmt.Sprintf("unexpected data in needACK: %#v", f))
		}
	case <- time.After(ACKWAIT * time.Second):
		return errors.New("timed out waiting for ACK")
	}
}

type Bootloader struct {
	Port io.ReadWriteCloser // serial port
	Frames chan Frame       // channel to recieve frames from
}

func (c Bootloader) Sync() error {
	b := []byte{0x55, 0x55}
	n, err := c.Port.Write(b)
	if err != nil {
		log.Fatalf("port.Write: %v", err)
		return err
	}
	_ = n
	err = needACK(c.Frames)
	return err
}

func (c Bootloader) Ping() bool {
	com := Packet{COMMAND_PING, nil}
	c.Port.Write(com.Serialize())
	needACK(c.Frames)
	return true
}


func (c Bootloader) GetChipID() int {
	com := Packet{COMMAND_GET_CHIP_ID, nil}
	c.Port.Write(com.Serialize())

	needACK(c.Frames)

	// get the ID
	// cc2538: 0x00 00  b9  64
	//            0  0 185 100
	frame := <- c.Frames
	id := frame.chipID()
	// need to send an ACK. Why! (grrr..ti)
	c.ack()

	return id
}


func pack32(n uint32, b []byte) {
	b[3] = byte(n)
	b[2] = byte(n >> 8)
	b[1] = byte(n >> 16)
	b[0] = byte(n >> 24)
}

func (c Bootloader) ack() {
	c.Port.Write([]byte{0xcc})
}

func (c Bootloader) Erase(addr uint32, num uint32) {
	p := make([]byte, 8)
	pack32(addr, p[0:4])
	pack32(num, p[4:8])
	com := Packet{COMMAND_ERASE, p}
	c.Port.Write(com.Serialize())
	needACK(c.Frames)
}

func (c Bootloader) Crc32(addr uint32, num uint32) uint32 {
	p := make([]byte, 8)
	pack32(addr, p[0:4])
	pack32(num, p[4:8])
	com := Packet{COMMAND_CRC32, p}
	c.Port.Write(com.Serialize())
	needACK(c.Frames)
	crc := <- c.Frames
	crc = crc[2:] // first two bytes are len and checksum
	c.ack()
	return uint32(crc[3]) << 24 | uint32(crc[2]) << 16 | uint32(crc[1]) << 8 | uint32(crc[0])
}

// width can be 1 for byte read or 4 for 32bit word read
// regardless of width, uint32 with the data is always returned.
func (c Bootloader) Read(addr uint32, width uint) uint32 {
	p := make([]byte, 5)
	pack32(addr, p[0:4])
	p[4] = byte(width)
	com := Packet{COMMAND_MEMORY_READ, p}
	c.Port.Write(com.Serialize())
	needACK(c.Frames)
	data := <- c.Frames
	data = data[2:] // first two bytes are len and checksum
	c.ack()
	return uint32(data[3]) << 24 | uint32(data[2]) << 16 | uint32(data[1]) << 8 | uint32(data[0])
}

func (c Bootloader) Download(addr uint32, len uint32) {
	p := make([]byte, 8)
	pack32(addr, p[0:4])
	pack32(len, p[4:8])
	com := Packet{COMMAND_DOWNLOAD, p}
	c.Port.Write(com.Serialize())
	needACK(c.Frames)
}

// send up to 252 bytes in a COMMAND_SEND_DATA
func (c Bootloader) SendData(data []byte) {
	com := Packet{COMMAND_SEND_DATA, data}
	c.Port.Write(com.Serialize())
	needACK(c.Frames)
}

func (c Bootloader) GetStatus() commandCode {
	com := Packet{COMMAND_GET_STATUS, nil}
	c.Port.Write(com.Serialize())
	needACK(c.Frames)
	data := <- c.Frames
	fmt.Println("status frame:", data)
	data = data[2:] // first two bytes are len and checksum
	c.ack()
	return commandCode(data[0])
}

// flash does a "download" followed by "send_data" to write a payload
func (c Bootloader) Flash(addr uint32, payload []byte) {
	len := len(payload)
	c.Download(addr, uint32(len))
	i := 0
	for i < len {
		amt := len - i
		if amt >= 248 {
			amt = 248
		}
		c.SendData(payload[i:i+amt])
		status := c.GetStatus()
		switch status {
		case COMMAND_RET_FLASH_FAIL:
			fmt.Printf("flashing failed with status 0x%x: addr %x start %x len %x; will retry\n", status, addr, i, amt)
		case COMMAND_RET_SUCCESS:
			i += amt
		default:
			fmt.Printf("flashing failed with status 0x%x: addr %x start %x len %x; stopping\n", status, addr, i, amt)
			goto out
		}
	}
out:
}
