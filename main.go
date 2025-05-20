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
// Mappings for macOS with DS4 via Bluetooth (6 axes)
// This will be overridden by build tags for Linux later if needed.
const (
	axisMacOSLeftStickX  = 0 // Pan (from Left Stick)
	axisMacOSLeftStickY  = 1 // Tilt (from Left Stick)
	
	axisMacOSL2         = 2 // Reported as "Tilt Down" - L2 Trigger
	axisMacOSRightStickX = 3 // Reported as "Zoom In/Out"
	axisMacOSRightStickY = 4 // Reported as "Pan Left/Right" (from Right Stick)
	axisMacOSR2         = 5 // Reported as "Does Nothing" - R2 Trigger
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

	// Pan/Tilt Logic for macOS Bluetooth
	var finalPanInput, finalTiltInput int

	// Pan: Prioritize Left Stick X (axisMacOSLeftStickX), fallback to Right Stick Y (axisMacOSRightStickY)
	leftStickXActive := currentLeftStickX < -joystickAxisDeadzone || currentLeftStickX > joystickAxisDeadzone
	rightStickYActiveForPan := currentRightStickY < -joystickAxisDeadzone || currentRightStickY > joystickAxisDeadzone

	if leftStickXActive {
		finalPanInput = currentLeftStickX
	} else if rightStickYActiveForPan {
		finalPanInput = currentRightStickY // Using RS Y for pan
	} else {
		finalPanInput = 0
	}

	// Tilt: Prioritize Left Stick Y (axisMacOSLeftStickY), L2 (axisMacOSL2) for "Tilt Down"
	leftStickYActive := currentLeftStickY < -joystickAxisDeadzone || currentLeftStickY > joystickAxisDeadzone
	l2ActiveForTilt := currentL2Pressure > joystickTriggerActivationThreshold // L2 is an axis, positive might mean down

	if leftStickYActive {
		finalTiltInput = currentLeftStickY
	} else if l2ActiveForTilt {
		// L2 (axisMacOSL2) causes "Tilt Down". mapJoystickAxisToVisca expects negative for up, positive for down.
		// If L2 axis goes from -32768 (rest) to 32767 (pressed), we need to map this.
		// For simplicity, if L2 is pressed, let's simulate a "down" value.
		// This needs refinement: what's the actual range of L2?
		// Assuming positive values from L2 mean "down" pressure.
		// We need to scale currentL2Pressure (e.g. 0 to 32767 if it's half-axis) to a portion of the positive tilt range.
		// For now, if L2 is active, set a fixed downward tilt input. This is a placeholder.
		// A better way: map currentL2Pressure (e.g. -32k to 32k) to a 0-32k "downward pressure" value.
		// If L2 is -32768 (off) to 32767 (full on), then (currentL2Pressure + 32768)/2 could be a 0-32767 value.
		tiltPressure := (currentL2Pressure + 32768) / 2 // crude map to 0-32k range
		if tiltPressure > joystickAxisDeadzone { // only if significantly pressed
			finalTiltInput = tiltPressure // Use this positive value for "down"
		} else {
			finalTiltInput = 0
		}
	} else {
		finalTiltInput = 0
	}
	
	currentPanSpeed, currentPanDirection := mapJoystickAxisToVisca(finalPanInput, viscaMaxPanSpeed, joystickAxisDeadzone)
	currentTiltSpeed, currentTiltDirection := mapJoystickAxisToVisca(finalTiltInput, viscaMaxTiltSpeed, joystickAxisDeadzone)

	if currentPanDirection == 0x03 { currentPanSpeed = 0 }
	if currentTiltDirection == 0x03 { currentTiltSpeed = 0 }

	if currentPanSpeed != lastSentPanSpeed || currentTiltSpeed != lastSentTiltSpeed || currentPanDirection != lastSentPanDirection || currentTiltDirection != lastSentTiltDirection {
		cmdHexPT := fmt.Sprintf("%s %02x %02x %02x %02x", cmdPanTiltDrive, currentPanSpeed, currentTiltSpeed, currentPanDirection, currentTiltDirection)
		_, err := sendRawViscaCommand(cameraTCPConn, cmdHexPT, false)
		if err != nil {
			log.Printf("VISCA PanTiltDrive error: %v", err)
		} else {
			lastSentPanSpeed = currentPanSpeed
			lastSentTiltSpeed = currentTiltSpeed
			lastSentPanDirection = currentPanDirection
			lastSentTiltDirection = currentTiltDirection
		}
	}

	// Zoom Logic for macOS Bluetooth: Right Stick X (axisMacOSRightStickX)
	// Negative for In, Positive for Out (assumption, may need to flip)
	var zoomSpeed byte
	var zoomCmdPart string // "2" for In, "3" for Out

	// currentRightStickX is -32768 (left) to 32767 (right)
	if currentRightStickX < -joystickAxisDeadzone { // Assuming Left on RSX is Zoom In
		zoomCmdPart = "2" // Zoom In
		// Normalize: 0 (at -deadZone) to 1 (at -32768)
		normalizedVal := float64(-currentRightStickX-joystickAxisDeadzone) / float64(32768-joystickAxisDeadzone)
		rawSpeed := normalizedVal * float64(viscaMaxZoomSpeed-1)
		zoomSpeed = byte(rawSpeed) + 1
		if zoomSpeed > viscaMaxZoomSpeed { zoomSpeed = viscaMaxZoomSpeed }
	} else if currentRightStickX > joystickAxisDeadzone { // Assuming Right on RSX is Zoom Out
		zoomCmdPart = "3" // Zoom Out
		// Normalize: 0 (at deadZone) to 1 (at 32767)
		normalizedVal := float64(currentRightStickX-joystickAxisDeadzone) / float64(32767-joystickAxisDeadzone)
		rawSpeed := normalizedVal * float64(viscaMaxZoomSpeed-1)
		zoomSpeed = byte(rawSpeed) + 1
		if zoomSpeed > viscaMaxZoomSpeed { zoomSpeed = viscaMaxZoomSpeed }
	} else { // Zoom Stop
		zoomSpeed = 0
	}
	
	// R2 (axisMacOSR2) is reported as doing nothing, so not used for zoom here.

	var cmdHexZoom string
	if zoomSpeed > 0 {
		cmdHexZoom = fmt.Sprintf("%s %s%01x", cmdZoomBase, zoomCmdPart, zoomSpeed)
	} else {
		cmdHexZoom = cmdZoomStop
	}

	if cmdHexZoom != lastZoomCommand {
		_, err := sendRawViscaCommand(cameraTCPConn, cmdHexZoom, false)
		if err != nil { log.Printf("VISCA Zoom error: %v", err) }
		lastZoomCommand = cmdHexZoom
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

				// Update global stick/trigger states for macOS (6-axis)
				// Ensure AxisData has enough elements before accessing
				if len(state.AxisData) > axisMacOSLeftStickX {
					currentLeftStickX = state.AxisData[axisMacOSLeftStickX]
				}
				if len(state.AxisData) > axisMacOSLeftStickY {
					currentLeftStickY = state.AxisData[axisMacOSLeftStickY]
				}
				if len(state.AxisData) > axisMacOSRightStickX { // Now used for Zoom
					currentRightStickX = state.AxisData[axisMacOSRightStickX]
				}
				if len(state.AxisData) > axisMacOSRightStickY { // Now used for Pan
					currentRightStickY = state.AxisData[axisMacOSRightStickY]
				}
				if len(state.AxisData) > axisMacOSL2 { // Now used for Tilt Down
					currentL2Pressure = state.AxisData[axisMacOSL2]
				}
				if len(state.AxisData) > axisMacOSR2 { // Reported as does nothing
					currentR2Pressure = state.AxisData[axisMacOSR2]
				}

				// Log all raw axis data to help with mapping
				if len(state.AxisData) >= 8 { // Assuming at least 8 axes as reported
					log.Printf("Raw Axes: [0]=%d [1]=%d [2]=%d [3]=%d [4]=%d [5]=%d [6]=%d [7]=%d",
						state.AxisData[0], state.AxisData[1], state.AxisData[2], state.AxisData[3],
						state.AxisData[4], state.AxisData[5], state.AxisData[6], state.AxisData[7])
				} else if len(state.AxisData) > 0 { // Log whatever axes are available
					log.Printf("Raw Axes: %v (Warning: Expected at least 8 axes, got %d)", state.AxisData, len(state.AxisData))
				} else {
					log.Printf("Raw Axes: No axis data received")
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