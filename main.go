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

	"github.com/0xcafed00d/joystick"
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
	// Adjusted for -32768 to 32767 range. Increased to 8192 (25%) to prevent "swing back".
	joystickAxisDeadzone int = 8192
	// L2/R2 triggers are often -32768 (released) to 32767 (pressed)
	// We'll consider "active" if > -28000 (a bit beyond fully released)
	joystickTriggerActivationThreshold int = -28000 
)

// Store axis values directly from joystick library (-32768 to 32767)
var currentLeftStickX  int = 0
var currentLeftStickY  int = 0
var currentRightStickX int = 0
var currentRightStickY int = 0
var currentL2Pressure  int = -32768 // Assuming -32768 is released
var currentR2Pressure  int = -32768 // Assuming -32768 is released

var lastSentPanSpeed byte = 0x00
var lastSentTiltSpeed byte = 0x00
var lastSentPanDirection byte = 0x03
var lastSentTiltDirection byte = 0x03
var lastZoomCommand string

// --- Standard DS4 Axis mapping for github.com/0xcafed00d/joystick on macOS (Typical) ---
// This might need verification/adjustment
// Updated based on Linux testing:
const (
	axisLeftStickX  = 0 // Confirmed
	axisLeftStickY  = 1 // Confirmed
	axisL2Pressure  = 2 // L2 Trigger (was 3)
	axisRightStickX = 3 // Right Stick X (was 2)
	axisRightStickY = 4 // Confirmed
	axisR2Pressure  = 5 // Confirmed
	// Axes 6 and 7 are likely D-Pad
)
// Note: Button constants would also be needed if we use buttons other than L1/R1 for logging.
// For now, focusing on axis-driven controls.

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

// mapJoystickAxisToVisca maps joystick axis (-32768 to 32767) to VISCA speed and direction
func mapJoystickAxisToVisca(axisValue int, maxSpeed byte, deadZone int) (speed byte, direction byte) {
	// axisValue is -32768 (left/up) to 32767 (right/down), 0 is center
	if axisValue < -deadZone { // Left or Up
		direction = 0x01
		// Normalize: 0 (at -deadZone) to 1 (at -32768)
		normalizedVal := float64(-axisValue-deadZone) / float64(32768-deadZone)
		rawSpeed := normalizedVal * float64(maxSpeed-1)
		speed = byte(rawSpeed) + 1
		if speed > maxSpeed { speed = maxSpeed }
	} else if axisValue > deadZone { // Right or Down
		direction = 0x02
		// Normalize: 0 (at deadZone) to 1 (at 32767)
		normalizedVal := float64(axisValue-deadZone) / float64(32767-deadZone)
		rawSpeed := normalizedVal * float64(maxSpeed-1)
		speed = byte(rawSpeed) + 1
		if speed > maxSpeed { speed = maxSpeed }
	} else { // In deadzone
		speed = 0x00
		direction = 0x03
	}
	return
}

// mapJoystickTriggerToViscaZoomSpeed maps trigger pressure (-32768 to 32767) to VISCA zoom speed
func mapJoystickTriggerToViscaZoomSpeed(pressureValue int) byte {
	// Assuming pressureValue is -32768 (released) to 32767 (fully pressed)
	// And we only care about positive pressure (or map it if it's different)
	if pressureValue < joystickTriggerActivationThreshold { // Effectively released or not pressed enough
		return 0
	}
	// Normalize from activation threshold to max value
	// Effective range: joystickTriggerActivationThreshold to 32767
	// Let's simplify: consider any value > 0 as pressed for now, and scale from 0 to 32767
	// This part needs careful adjustment based on actual L2/R2 axis reporting
	
	// For now, let's assume a simple positive scale if pressureValue > 0
	// A more robust approach would map the -32768 to 32767 range to 0-1 for pressure.
	// E.g. (pressureValue + 32768) / (32767 + 32768) -> 0 to 1
	
	normalizedPressure := float64(pressureValue - joystickTriggerActivationThreshold) / float64(32767 - joystickTriggerActivationThreshold)
	if normalizedPressure < 0 { normalizedPressure = 0 }
	if normalizedPressure > 1 { normalizedPressure = 1 }

	if normalizedPressure == 0 { return 0 }

	speed := byte(normalizedPressure*float64(viscaMaxZoomSpeed-1)) + 1
	if speed > viscaMaxZoomSpeed { return viscaMaxZoomSpeed }
	return speed
}

func processInputsAndCamera() {
	if cameraTCPConn == nil { return }

	// Pan/Tilt Logic
	var finalPanX, finalPanY int

	// Re-enable right stick control if left stick is centered
	leftStickActive := currentLeftStickX < -joystickAxisDeadzone || currentLeftStickX > joystickAxisDeadzone ||
					   currentLeftStickY < -joystickAxisDeadzone || currentLeftStickY > joystickAxisDeadzone
	rightStickActive := currentRightStickX < -joystickAxisDeadzone || currentRightStickX > joystickAxisDeadzone ||
						currentRightStickY < -joystickAxisDeadzone || currentRightStickY > joystickAxisDeadzone

	if leftStickActive {
		finalPanX = currentLeftStickX
		finalPanY = currentLeftStickY
	} else if rightStickActive {
		finalPanX = currentRightStickX
		finalPanY = currentRightStickY
	} else {
		finalPanX = 0 // Center for new axis range
		finalPanY = 0 // Center for new axis range
	}

	currentPanSpeed, currentPanDirection := mapJoystickAxisToVisca(finalPanX, viscaMaxPanSpeed, joystickAxisDeadzone)
	currentTiltSpeed, currentTiltDirection := mapJoystickAxisToVisca(finalPanY, viscaMaxTiltSpeed, joystickAxisDeadzone)

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

	// Zoom Logic
	var zoomSpeed byte
	var zoomCmdPart string

	l2ZoomSpeed := mapJoystickTriggerToViscaZoomSpeed(currentL2Pressure)
	r2ZoomSpeed := mapJoystickTriggerToViscaZoomSpeed(currentR2Pressure)

	if l2ZoomSpeed > 0 { // Zoom Out
		zoomSpeed = l2ZoomSpeed
		zoomCmdPart = "3" // VISCA zoom out
	} else if r2ZoomSpeed > 0 { // Zoom In
		zoomSpeed = r2ZoomSpeed
		zoomCmdPart = "2" // VISCA zoom in
	} else { // Zoom Stop
		zoomSpeed = 0
	}

	var cmdHex string
	if zoomSpeed > 0 {
		cmdHex = fmt.Sprintf("%s %s%01x", cmdZoomBase, zoomCmdPart, zoomSpeed)
	} else {
		cmdHex = cmdZoomStop
	}

	if cmdHex != lastZoomCommand {
		_, err := sendRawViscaCommand(cameraTCPConn, cmdHex, false)
		if err != nil { log.Printf("VISCA Zoom error: %v", err) }
		lastZoomCommand = cmdHex
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

	// --- Joystick Initialization using github.com/0xcafed00d/joystick ---
	js, err := joystick.Open(0) // Open the first joystick
	if err != nil {
		log.Panicf("Failed to open joystick: %v", err)
	}
	defer js.Close()

	log.Printf("Opened Joystick: %s", js.Name())
	log.Printf("    Axis Count: %d", js.AxisCount())
	log.Printf("  Button Count: %d", js.ButtonCount())
	// It's useful to log these counts to help verify axis/button mapping later

	signals := make(chan os.Signal, 1)
	signal.Notify(signals, syscall.SIGINT, syscall.SIGTERM)
	
	// Ticker for polling joystick
	ticker := time.NewTicker(20 * time.Millisecond) // Poll 50 times per second
	defer ticker.Stop()

	quit := make(chan struct{})

	go func() {
		for {
			select {
			case <-ticker.C:
				state, err := js.Read()
				if err != nil {
					log.Printf("Error reading joystick state: %v", err)
					// Consider if we should attempt to reconnect or panic
					continue
				}

				// Update global stick/trigger states
				// Ensure AxisData has enough elements before accessing
				if len(state.AxisData) > axisLeftStickX {
					currentLeftStickX = state.AxisData[axisLeftStickX]
				}
				if len(state.AxisData) > axisLeftStickY {
					currentLeftStickY = state.AxisData[axisLeftStickY]
				}
				if len(state.AxisData) > axisRightStickX {
					currentRightStickX = state.AxisData[axisRightStickX]
				}
				if len(state.AxisData) > axisRightStickY {
					currentRightStickY = state.AxisData[axisRightStickY]
				}
				if len(state.AxisData) > axisL2Pressure {
					currentL2Pressure = state.AxisData[axisL2Pressure]
				}
				if len(state.AxisData) > axisR2Pressure {
					currentR2Pressure = state.AxisData[axisR2Pressure]
				}
				
				// For logging buttons (example: L1 and R1)
				// Button mapping for DS4 with this library needs to be confirmed.
				// Typically: Button 0=Cross, 1=Circle, 2=Square, 3=Triangle
				// L1=4, R1=5, L2(button)=6, R2(button)=7, Share=8, Options=9, L3=10, R3=11, PS=12, TouchpadClick=13
				// const buttonL1 = 1 << 4 // Bitmask for button 4
				// const buttonR1 = 1 << 5 // Bitmask for button 5
				// if (state.Buttons & buttonL1) != 0 { log.Println("L1 Pressed") }
				// if (state.Buttons & buttonR1) != 0 { log.Println("R1 Pressed") }


				processInputsAndCamera()

			case <-quit:
				return
			}
		}
	}()

	// Handle OS signals for graceful shutdown
	s := <-signals
	log.Printf("* Received signal: %v\n", s)
	close(quit) // Signal the joystick reading goroutine to stop

	if cameraTCPConn != nil {
		log.Println("Stopping camera movement on exit...")
		if lastSentPanDirection != 0x03 || lastSentTiltDirection != 0x03 || lastSentPanSpeed != 0 || lastSentTiltSpeed != 0 {
			stopCmd := fmt.Sprintf("%s %02x %02x %02x %02x", cmdPanTiltDrive, 0x00, 0x00, 0x03, 0x03)
			_, err := sendRawViscaCommand(cameraTCPConn, stopCmd, false)
			if err != nil { log.Printf("Error stopping pan/tilt on exit: %v", err) }
		}
		if lastZoomCommand != cmdZoomStop {
			_, err := sendRawViscaCommand(cameraTCPConn, cmdZoomStop, false)
			if err != nil { log.Printf("Error stopping zoom on exit: %v", err) }
		}
	}
	log.Println("Application terminated.")
}