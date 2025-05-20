package main

import (
	"flag"
	"fmt"
	"log"
	"net"
	"os"
	"os/signal"
	"syscall"
	"time"
	"bytes" 

	"github.com/kpeu3i/gods4"
)

var cameraTCPConn net.Conn 
var cameraHost string
var cameraPort string

var (
	viscaPreambleCommand = []byte{0x81, 0x01}
	viscaPreambleQuery   = []byte{0x81, 0x09}
	viscaTerminator      = byte(0xFF)

	cmdIFClear      = "00 01"
	cmdPanTiltDrive = "06 01" 
	cmdZoomBase     = "04 07" 
	cmdZoomStop     = "04 07 00"
)

const (
	viscaMaxPanSpeed  byte = 0x18 
	viscaMaxTiltSpeed byte = 0x14 
	viscaMaxZoomSpeed byte = 0x07 
	triggerDeadzone   byte = 20   
	stickDeadzone     int  = 20 
)

var currentLeftStickX byte = 128
var currentLeftStickY byte = 128
var currentRightStickX byte = 128
var currentRightStickY byte = 128

var lastSentPanSpeed byte = 0x00
var lastSentTiltSpeed byte = 0x00
var lastSentPanDirection byte = 0x03 
var lastSentTiltDirection byte = 0x03 
var lastZoomCommand string 

func sendRawViscaCommand(conn net.Conn, commandPayloadHex string, isQuery bool) ([]byte, error) {
	if conn == nil {
		return nil, fmt.Errorf("camera connection is not established")
	}
	preamble := viscaPreambleCommand
	if isQuery { preamble = viscaPreambleQuery }
	payloadBytes, err := hexStringToBytes(commandPayloadHex)
	if err != nil { return nil, fmt.Errorf("error converting command hex string: %v", err) }
	message := append(append(preamble, payloadBytes...), viscaTerminator)
	_, err = conn.Write(message)
	if err != nil { return nil, fmt.Errorf("error sending VISCA command: %v", err) }

	var fullResponse []byte
	buffer := make([]byte, 64) 
	startTime := time.Now()
	for {
		conn.SetReadDeadline(time.Now().Add(250 * time.Millisecond)) 
		n, err := conn.Read(buffer)
		if err != nil {
			if netErr, ok := err.(net.Error); ok && netErr.Timeout() {
				if len(fullResponse) > 0 { break }
				return nil, fmt.Errorf("visca response timeout") 
			}
			return nil, fmt.Errorf("error reading VISCA response: %v", err)
		}
		fullResponse = append(fullResponse, buffer[:n]...)
		processedSomething := false
		for {
			ffIndex := bytes.IndexByte(fullResponse, viscaTerminator)
			if ffIndex == -1 { break }
			singleViscaMessage := fullResponse[:ffIndex+1]
			fullResponse = fullResponse[ffIndex+1:] 
			processedSomething = true
			if len(singleViscaMessage) < 2 { 
				log.Printf("VISCA: Received too short message: %x\n", singleViscaMessage)
				continue 
			}
			if singleViscaMessage[0] == 0x90 && (singleViscaMessage[1]>>4) == 0x06 { 
				return nil, fmt.Errorf("visca error response: %x (status byte: %#02x)", singleViscaMessage, singleViscaMessage[1])
			}
			if singleViscaMessage[0] == 0x90 && (singleViscaMessage[1]>>4) == 0x05 {
				if len(singleViscaMessage) > 2 { return singleViscaMessage[2 : len(singleViscaMessage)-1], nil }
				return []byte{}, nil 
			}
			if singleViscaMessage[0] == 0x90 && (singleViscaMessage[1]>>4) == 0x04 {
				if len(fullResponse) == 0 && time.Since(startTime) > 500*time.Millisecond { return []byte{}, nil }
				continue
			}
			if isQuery && singleViscaMessage[0] == 0x90 { 
			    if len(singleViscaMessage) > 2 { return singleViscaMessage[1 : len(singleViscaMessage)-1], nil }
			}
			log.Printf("VISCA: Received unhandled message type: %x\n", singleViscaMessage)
		}
		if !processedSomething && len(fullResponse) > 0 && time.Since(startTime) > 500*time.Millisecond {
			return nil, fmt.Errorf("incomplete VISCA response (no terminator): %x", fullResponse)
		}
		if len(fullResponse) == 0 && time.Since(startTime) > 500*time.Millisecond {
			return nil, fmt.Errorf("visca response processing timeout")
		}
		if len(fullResponse) > 32 { return nil, fmt.Errorf("visca response buffer too large") }
	}
	if len(fullResponse) > 0 { return nil, fmt.Errorf("exited response loop with unprocessed data: %x", fullResponse) }
	return nil, fmt.Errorf("exited response loop without valid completion") 
}

func hexStringToBytes(hexStr string) ([]byte, error) {
	s := ""
	for _, part := range bytes.Fields([]byte(hexStr)) { s += string(part) }
	if len(s)%2 != 0 { return nil, fmt.Errorf("hex string must have an even number of characters: %s", hexStr) }
	var result []byte
	for i := 0; i < len(s); i += 2 {
		byteStr := s[i : i+2]
		var b byte
		_, err := fmt.Sscanf(byteStr, "%02x", &b)
		if err != nil { return nil, fmt.Errorf("error parsing hex byte string '%s': %v", byteStr, err) }
		result = append(result, b)
	}
	return result, nil
}

func mapAxisToVisca(axisValue int, maxSpeed byte, deadZone int) (speed byte, direction byte) {
	center := 128
	value := int(axisValue)
	if value < center-deadZone { 
		direction = 0x01 
		rawSpeed := float64(center-deadZone-1-value) / float64(center-deadZone-1) * float64(maxSpeed-1)
		speed = byte(rawSpeed) + 1
		if speed == 0 && direction != 0x03 { speed = 1 } 
		if speed > maxSpeed { speed = maxSpeed }
	} else if value > center+deadZone { 
		direction = 0x02 
		rawSpeed := float64(value-(center+deadZone+1)) / float64(255-(center+deadZone+1)) * float64(maxSpeed-1)
		speed = byte(rawSpeed) + 1
		if speed == 0 && direction != 0x03 { speed = 1 } 
		if speed > maxSpeed { speed = maxSpeed }
	} else { 
		speed = 0x00 
		direction = 0x03 
	}
	return
}

func mapPressureToViscaZoomSpeed(pressureValue byte) byte {
	if pressureValue < triggerDeadzone { return 0 }
	effectivePressure := float64(pressureValue - triggerDeadzone)
	maxEffectivePressure := float64(255 - triggerDeadzone)
	speed := byte((effectivePressure / maxEffectivePressure) * float64(viscaMaxZoomSpeed-1)) + 1
	if speed > viscaMaxZoomSpeed { return viscaMaxZoomSpeed }
	return speed
}

func processPanTilt() {
	if cameraTCPConn == nil { return }

	var finalPanX, finalPanY byte
	
	// Cast stick positions to int for comparison with stickDeadzone (int)
	leftStickActive := int(currentLeftStickX) < 128-stickDeadzone || int(currentLeftStickX) > 128+stickDeadzone ||
					   int(currentLeftStickY) < 128-stickDeadzone || int(currentLeftStickY) > 128+stickDeadzone
	rightStickActive := int(currentRightStickX) < 128-stickDeadzone || int(currentRightStickX) > 128+stickDeadzone ||
						int(currentRightStickY) < 128-stickDeadzone || int(currentRightStickY) > 128+stickDeadzone

	if leftStickActive {
		finalPanX = currentLeftStickX
		finalPanY = currentLeftStickY
	} else if rightStickActive {
		finalPanX = currentRightStickX
		finalPanY = currentRightStickY
	} else { 
		finalPanX = 128
		finalPanY = 128
	}

	currentPanSpeed, currentPanDirection := mapAxisToVisca(int(finalPanX), viscaMaxPanSpeed, stickDeadzone) 
	currentTiltSpeed, currentTiltDirection := mapAxisToVisca(int(finalPanY), viscaMaxTiltSpeed, stickDeadzone) 
	
	if currentPanDirection == 0x03 { currentPanSpeed = 0 }
	if currentTiltDirection == 0x03 { currentTiltSpeed = 0 }

	if currentPanSpeed != lastSentPanSpeed || currentTiltSpeed != lastSentTiltSpeed || currentPanDirection != lastSentPanDirection || currentTiltDirection != lastSentTiltDirection {
		cmdHex := fmt.Sprintf("%s %02x %02x %02x %02x", cmdPanTiltDrive, currentPanSpeed, currentTiltSpeed, currentPanDirection, currentTiltDirection)
		_, err := sendRawViscaCommand(cameraTCPConn, cmdHex, false)
		if err != nil {
			log.Printf("VISCA PanTiltDrive error: %v", err)
		} else {
			lastSentPanSpeed = currentPanSpeed
			lastSentTiltSpeed = currentTiltSpeed
			lastSentPanDirection = currentPanDirection
			lastSentTiltDirection = currentTiltDirection
		}
	}
}

func main() {
	flag.StringVar(&cameraHost, "host", "192.168.1.100", "VISCA camera IP address or hostname")
	flag.StringVar(&cameraPort, "port", "52381", "VISCA camera TCP port")
	flag.Parse()

	cameraAddress := fmt.Sprintf("%s:%s", cameraHost, cameraPort)
	log.Printf("Attempting to connect to VISCA camera via TCP at %s...\n", cameraAddress)
	
	var err error
	cameraTCPConn, err = net.DialTimeout("tcp", cameraAddress, 5*time.Second)
	if err != nil { log.Panicf("Failed to establish TCP connection to VISCA camera: %v", err) }
	log.Println("Successfully established TCP connection to VISCA camera.")
	defer cameraTCPConn.Close()

	_, err = sendRawViscaCommand(cameraTCPConn, cmdIFClear, false)
	if err != nil { log.Printf("Error sending IF_Clear: %v", err) } else { log.Println("IF_Clear sent to camera.") }

	controllers := gods4.Find()
	if len(controllers) == 0 { log.Panic("No connected DS4 controllers found") }
	controller := controllers[0]
	err = controller.Connect() 
	if err != nil { log.Panicf("Failed to connect to controller: %v", err) }
	log.Printf("* Controller #1 | %-10s | name: %s, connection: %s\n", "Connect", controller, controller.ConnectionType())

	signals := make(chan os.Signal, 1)
	signal.Notify(signals, syscall.SIGINT, syscall.SIGTERM)
	go func() {
		s := <-signals
		log.Printf("* Controller #1 | %-10s | received signal: %v\n", "Signal", s)
		if cameraTCPConn != nil {
			log.Println("Stopping camera movement on exit...")
			if lastSentPanDirection != 0x03 || lastSentTiltDirection != 0x03 {
				stopCmd := fmt.Sprintf("%s %02x %02x %02x %02x", cmdPanTiltDrive, 0x00, 0x00, 0x03, 0x03)
				_, err := sendRawViscaCommand(cameraTCPConn, stopCmd, false)
				if err != nil { log.Printf("Error stopping pan/tilt on exit: %v", err) }
			}
			if lastZoomCommand != cmdZoomStop {
				_, err := sendRawViscaCommand(cameraTCPConn, cmdZoomStop, false)
				if err != nil { log.Printf("Error stopping zoom on exit: %v", err) }
			}
		}
		controller.Disconnect()
		log.Printf("* Controller #1 | %-10s | bye!\n", "Disconnect")
		os.Exit(0)
	}()

	controller.On(gods4.EventLeftStickMove, func(data interface{}) error {
		stick, ok := data.(gods4.Stick)
		if !ok { return fmt.Errorf("invalid data type for LeftStickMove: %T", data) }
		currentLeftStickX = stick.X
		currentLeftStickY = stick.Y
		processPanTilt()
		return nil
	})

	controller.On(gods4.EventRightStickMove, func(data interface{}) error {
		stick, ok := data.(gods4.Stick)
		if !ok { return fmt.Errorf("invalid data type for RightStickMove: %T", data) }
		currentRightStickX = stick.X
		currentRightStickY = stick.Y
		processPanTilt()
		return nil
	})

	controller.On(gods4.EventL2Press, func(data interface{}) error { 
		if cameraTCPConn == nil { return nil }
		pressure, ok := data.(byte)
		if !ok { log.Printf("Error: L2Press data not byte: %T", data); return nil }
		zoomSpeed := mapPressureToViscaZoomSpeed(pressure)
		var cmdHex string
		if zoomSpeed > 0 { cmdHex = fmt.Sprintf("%s 3%01x", cmdZoomBase, zoomSpeed) } else { cmdHex = cmdZoomStop }
		if cmdHex != lastZoomCommand { 
			_, err := sendRawViscaCommand(cameraTCPConn, cmdHex, false)
			if err != nil { log.Printf("VISCA Zoom Out (L2) error: %v", err) }
			lastZoomCommand = cmdHex
		}
		return nil
	})
	controller.On(gods4.EventR2Press, func(data interface{}) error { 
		if cameraTCPConn == nil { return nil }
		pressure, ok := data.(byte)
		if !ok { log.Printf("Error: R2Press data not byte: %T", data); return nil }
		zoomSpeed := mapPressureToViscaZoomSpeed(pressure)
		var cmdHex string
		if zoomSpeed > 0 { cmdHex = fmt.Sprintf("%s 2%01x", cmdZoomBase, zoomSpeed) } else { cmdHex = cmdZoomStop }
		if cmdHex != lastZoomCommand { 
			_, err := sendRawViscaCommand(cameraTCPConn, cmdHex, false)
			if err != nil { log.Printf("VISCA Zoom In (R2) error: %v", err) }
			lastZoomCommand = cmdHex
		}
		return nil
	})

	controller.On(gods4.EventL1Press, func(data interface{}) error { log.Printf("* Controller #1 | %-10s | state: press\n", "L1"); return nil })
	controller.On(gods4.EventR1Press, func(data interface{}) error { log.Printf("* Controller #1 | %-10s | state: press\n", "R1"); return nil })
	controller.On(gods4.EventCrossPress, func(data interface{}) error { log.Printf("* Controller #1 | %-10s | state: press\n", "Cross"); return nil })
	controller.On(gods4.EventCirclePress, func(data interface{}) error { log.Printf("* Controller #1 | %-10s | state: press\n", "Circle"); return nil })
	controller.On(gods4.EventSquarePress, func(data interface{}) error { log.Printf("* Controller #1 | %-10s | state: press\n", "Square"); return nil })
	controller.On(gods4.EventTrianglePress, func(data interface{}) error { log.Printf("* Controller #1 | %-10s | state: press\n", "Triangle"); return nil })
	controller.On(gods4.EventDPadUpPress, func(data interface{}) error { log.Printf("* Controller #1 | %-10s | state: press\n", "DpadUp"); return nil })
	controller.On(gods4.EventDPadDownPress, func(data interface{}) error { log.Printf("* Controller #1 | %-10s | state: press\n", "DpadDown"); return nil })
	controller.On(gods4.EventDPadLeftPress, func(data interface{}) error { log.Printf("* Controller #1 | %-10s | state: press\n", "DpadLeft"); return nil })
	controller.On(gods4.EventDPadRightPress, func(data interface{}) error { log.Printf("* Controller #1 | %-10s | state: press\n", "DpadRight"); return nil })
	controller.On(gods4.EventL3Press, func(data interface{}) error { log.Printf("* Controller #1 | %-10s | state: press\n", "L3 (Left Stick Click)"); return nil })
	controller.On(gods4.EventR3Press, func(data interface{}) error { log.Printf("* Controller #1 | %-10s | state: press\n", "R3 (Right Stick Click)"); return nil })
	controller.On(gods4.EventSharePress, func(data interface{}) error { log.Printf("* Controller #1 | %-10s | state: press\n", "Share"); return nil })
	controller.On(gods4.EventOptionsPress, func(data interface{}) error { log.Printf("* Controller #1 | %-10s | state: press\n", "Options"); return nil })
	controller.On(gods4.EventPSPress, func(data interface{}) error { log.Printf("* Controller #1 | %-10s | state: press\n", "PS Button"); return nil })
	controller.On(gods4.EventTouchpadPress, func(data interface{}) error { log.Printf("* Controller #1 | %-10s | state: press (physical click)\n", "Touchpad"); return nil })
	controller.On(gods4.EventTouchpadSwipe, func(data interface{}) error {
		touchData, ok := data.(gods4.Touchpad)
		if !ok { return fmt.Errorf("invalid data type for TouchpadSwipe: %T", data) }
		for i, touch := range touchData.Swipe {
			if touch.IsActive { log.Printf("* Controller #1 | %-10s | TouchIndex: %d, X: %d, Y: %d\n", "TouchSwipe", i, touch.X, touch.Y) }
		}
		return nil
	})
	controller.On(gods4.EventBatteryUpdate, func(data interface{}) error {
		battery := data.(gods4.Battery)
		log.Printf("* Controller #1 | %-10s | capacity: %v%%, charging: %v, cable: %v\n", "Battery", battery.Capacity, battery.IsCharging, battery.IsCableConnected)
		return nil
	})

	log.Println("Starting to listen for controller events...")
	err = controller.Listen() 
	if err != nil { log.Panicf("Error listening for controller events: %v", err) }
}