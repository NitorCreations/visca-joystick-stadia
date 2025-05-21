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

// Calibration related variables
var calibrationMode bool
var lastLoggedAxes []int // To potentially reduce log spam, though user wants continuous
var logCounter int       // Another way to reduce log spam, e.g. log every Nth update

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
	// L2/R2 triggers are 0 (released) to 32768 (pressed) based on calibration
	// We'll consider "active" if > a small threshold, e.g., 2000
	joystickTriggerActivationThreshold int = 2000
)

// Store axis values directly from joystick library (-32768 to 32767 for sticks, 0 to 32768 for triggers)
var currentAxis0 int = 0 // Left Stick X
var currentAxis1 int = 0 // Left Stick Y
var currentAxis2 int = 0 // Right Stick X (physical axis 2)
var currentAxis3 int = 0 // Right Stick Y (physical axis 3)
var currentAxis4 int = 0 // L2 Trigger
var currentAxis5 int = 0 // R2 Trigger
var isR1Pressed bool   // For precision mode toggle
// Add more if your controller has more axes and you need them
// var currentAxisN int = 0 ...


var lastSentPanSpeed byte = 0x00
var lastSentTiltSpeed byte = 0x00
var lastSentPanDirection byte = 0x03
var lastSentTiltDirection byte = 0x03
var lastZoomCommand string

// --- Axis mapping based on user calibration (Right Stick X/Y Swapped) ---
const (
	axisLeftStickX  = 0 // Pan, Tilt
	axisLeftStickY  = 1 // Pan, Tilt
	axisRightStickX = 2 // Corrected: Right Stick X (Horizontal) is physical Axis 2
	axisRightStickY = 3 // Corrected: Right Stick Y (Vertical) is physical Axis 3
	axisL2          = 4 // Zoom In
	axisR2          = 5 // Zoom Out
)

const (
	// Button mapping for DS4 with github.com/0xcafed00d/joystick (needs confirmation if not DS4)
	// L1=4, R1=5, L2(button)=6, R2(button)=7, Share=8, Options=9, L3=10, R3=11, PS=12, TouchpadClick=13
	buttonR1Mask = 1 << 5 // R1 is typically button 5
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

// mapJoystickTriggerToViscaZoomSpeed maps trigger pressure (0 to 32767) to VISCA zoom speed
func mapJoystickTriggerToViscaZoomSpeed(pressureValue int) byte {
	// pressureValue is 0 (released) to 32767 (fully pressed)
	if pressureValue < joystickTriggerActivationThreshold { // Effectively released or not pressed enough
		return 0
	}

	// Normalize from activation threshold to max value (32767)
	// Effective range for scaling: joystickTriggerActivationThreshold to 32767
	normalizedPressure := float64(pressureValue-joystickTriggerActivationThreshold) / float64(32767-joystickTriggerActivationThreshold)
	if normalizedPressure < 0 {
		normalizedPressure = 0
	}
	if normalizedPressure > 1 {
		normalizedPressure = 1
	}

	if normalizedPressure == 0 {
		return 0
	}

	// Scale to VISCA zoom speed (1 to viscaMaxZoomSpeed)
	// Adding 0.5 before casting to byte for better rounding, then ensure it's at least 1.
	speed := byte(normalizedPressure*float64(viscaMaxZoomSpeed-1) + 0.5) + 1
	if speed < 1 && normalizedPressure > 0 { // Ensure minimal speed if pressed
		speed = 1
	}
	if speed > viscaMaxZoomSpeed {
		speed = viscaMaxZoomSpeed
	}
	return speed
}

func processInputsAndCamera() {
	if cameraTCPConn == nil {
		return
	}

	// --- Pan/Tilt Logic ---
	var finalPanInput, finalTiltInput int

	// Read current stick values
	lsX := currentAxis0 // Left Stick X from physical axis 0
	lsY := currentAxis1 // Left Stick Y from physical axis 1
	
	// Corrected Right Stick assignments:
	// Right Stick X (Horizontal) is physical Axis 2, its value is in currentAxis2
	// Right Stick Y (Vertical) is physical Axis 3, its value is in currentAxis3
	rsX := currentAxis2
	rsY := currentAxis3

	// Pan: Prioritize Left Stick X, then Right Stick X
	if lsX < -joystickAxisDeadzone || lsX > joystickAxisDeadzone {
		finalPanInput = lsX
	} else if rsX < -joystickAxisDeadzone || rsX > joystickAxisDeadzone {
		finalPanInput = rsX // Use corrected rsX (from physical axis 2)
	} else {
		finalPanInput = 0
	}

	// Tilt: Prioritize Left Stick Y, then Right Stick Y
	// Note: Joystick Y-axis is often inverted (-32k up, +32k down).
	// mapJoystickAxisToVisca expects negative for up, positive for down.
	// If your joystick's Y is naturally inverted (push up = positive value),
	// you might need to multiply by -1 before passing to mapJoystickAxisToVisca.
	// Assuming standard behavior for now (push up = negative value for sticks).
	if lsY < -joystickAxisDeadzone || lsY > joystickAxisDeadzone {
		finalTiltInput = lsY
	} else if rsY < -joystickAxisDeadzone || rsY > joystickAxisDeadzone {
		finalTiltInput = rsY // Use corrected rsY (from physical axis 3)
	} else {
		finalTiltInput = 0
	}

	// Determine effective max speeds based on R1 state
	effectiveMaxPanSpeed := viscaMaxPanSpeed
	effectiveMaxTiltSpeed := viscaMaxTiltSpeed

	if !isR1Pressed { // Precision mode
		effectiveMaxPanSpeed = 8 // User requested max speed of 8 for precision pan
		if effectiveMaxPanSpeed < 1 { effectiveMaxPanSpeed = 1 }
		// Ensure precision speed doesn't exceed actual max speed (though 8 is well below 24)
		if effectiveMaxPanSpeed > viscaMaxPanSpeed { effectiveMaxPanSpeed = viscaMaxPanSpeed }


		effectiveMaxTiltSpeed = 8 // User requested max speed of 8 for precision tilt
		if effectiveMaxTiltSpeed < 1 { effectiveMaxTiltSpeed = 1 }
		// Ensure precision speed doesn't exceed actual max speed (though 8 is well below 20)
		if effectiveMaxTiltSpeed > viscaMaxTiltSpeed { effectiveMaxTiltSpeed = viscaMaxTiltSpeed }

	}

	currentPanSpeed, currentPanDirection := mapJoystickAxisToVisca(finalPanInput, effectiveMaxPanSpeed, joystickAxisDeadzone)
	currentTiltSpeed, currentTiltDirection := mapJoystickAxisToVisca(finalTiltInput, effectiveMaxTiltSpeed, joystickAxisDeadzone)

	if currentPanDirection == 0x03 {
		currentPanSpeed = 0
	} // Stop if in deadzone
	if currentTiltDirection == 0x03 {
		currentTiltSpeed = 0
	} // Stop if in deadzone

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

	// --- Zoom Logic ---
	// L2 (axisL2 = Axis 4) for Zoom In, R2 (axisR2 = Axis 5) for Zoom Out
	l2Pressure := currentAxis4
	r2Pressure := currentAxis5

	var zoomSpeed byte
	var zoomCmdPart string // "2" for In (Tele), "3" for Out (Wide)

	l2ZoomSpeed := mapJoystickTriggerToViscaZoomSpeed(l2Pressure)
	r2ZoomSpeed := mapJoystickTriggerToViscaZoomSpeed(r2Pressure)

	if l2ZoomSpeed > 0 && r2ZoomSpeed > 0 { // Both pressed, prioritize one or stop? Let's stop.
		zoomSpeed = 0
	} else if l2ZoomSpeed > 0 { // L2 for Zoom In
		zoomCmdPart = "2" // Tele
		zoomSpeed = l2ZoomSpeed
	} else if r2ZoomSpeed > 0 { // R2 for Zoom Out
		zoomCmdPart = "3" // Wide
		zoomSpeed = r2ZoomSpeed
	} else { // No zoom trigger pressed or both are below threshold
		zoomSpeed = 0
	}

	var cmdHexZoom string
	if zoomSpeed > 0 {
		cmdHexZoom = fmt.Sprintf("%s %s%01x", cmdZoomBase, zoomCmdPart, zoomSpeed)
	} else {
		cmdHexZoom = cmdZoomStop
	}

	if cmdHexZoom != lastZoomCommand {
		_, err := sendRawViscaCommand(cameraTCPConn, cmdHexZoom, false)
		if err != nil {
			log.Printf("VISCA Zoom error: %v", err)
		}
		lastZoomCommand = cmdHexZoom
	}
}

func main() {
	flag.StringVar(&cameraHost, "host", "192.168.1.100", "VISCA camera IP address or hostname")
	flag.StringVar(&cameraPort, "port", "52381", "VISCA camera TCP port")
	flag.BoolVar(&calibrationMode, "calibrate", false, "Run joystick axis calibration routine")
	flag.Parse()

	if calibrationMode {
		log.Println("=== Joystick Calibration Mode Activated ===")
		log.Println("Continuously logging raw axis data.")
		log.Println("Move your controller's sticks and triggers to observe changes.")
		log.Println("Press Ctrl+C to stop calibration once you have identified the axes.")
		lastLoggedAxes = make([]int, 0) // Initialize
	} else {
		cameraAddress := fmt.Sprintf("%s:%s", cameraHost, cameraPort)
		log.Printf("Attempting to connect to VISCA camera via TCP at %s...\n", cameraAddress)

		var err error
		cameraTCPConn, err = net.DialTimeout("tcp", cameraAddress, 5*time.Second)
		if err != nil {
			log.Fatalf("Failed to establish TCP connection to VISCA camera: %v. Ensure camera is on and accessible at %s.", err, cameraAddress)
		}
		log.Println("Successfully established TCP connection to VISCA camera.")
		defer cameraTCPConn.Close()

		_, err = sendRawViscaCommand(cameraTCPConn, cmdIFClear, false)
		if err != nil {
			log.Printf("Error sending IF_Clear: %v", err)
		} else {
			log.Println("IF_Clear sent to camera.")
		}
	}

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

				// Update global currentAxis variables based on calibration
				if len(state.AxisData) > axisLeftStickX {
					currentAxis0 = state.AxisData[axisLeftStickX]
				}
				if len(state.AxisData) > axisLeftStickY {
					currentAxis1 = state.AxisData[axisLeftStickY] // Physical axis 1
				}
				// currentAxis2 will store data from physical axis 2 (now Right Stick X)
				if len(state.AxisData) > axisRightStickX { // axisRightStickX is now 2
					currentAxis2 = state.AxisData[axisRightStickX]
				}
				// currentAxis3 will store data from physical axis 3 (now Right Stick Y)
				if len(state.AxisData) > axisRightStickY { // axisRightStickY is now 3
					currentAxis3 = state.AxisData[axisRightStickY]
				}
				if len(state.AxisData) > axisL2 { // Mapped to Axis 4
					currentAxis4 = state.AxisData[axisL2]
				}
				if len(state.AxisData) > axisR2 { // Mapped to Axis 5
					currentAxis5 = state.AxisData[axisR2]
				}
				// Add more if using more axes:
				// if len(state.AxisData) > axisN { currentAxisN = state.AxisData[axisN] }

				// Update R1 pressed state
				if (state.Buttons & buttonR1Mask) != 0 {
					isR1Pressed = true
				} else {
					isR1Pressed = false
				}

				if calibrationMode {
					// Continuous logging of all axis data
					// To reduce spam slightly, only log if data actually changed or periodically
					// For now, log every time to ensure user sees immediate feedback as requested.
					// The user mentioned "not repeat too much" initially, but then asked for no L1 trigger,
					// which implies more repetition is now acceptable for this mode.

					axisLog := "Raw Axes:"
					dataChanged := false

					if len(state.AxisData) != len(lastLoggedAxes) {
						dataChanged = true
					} else {
						for i, val := range state.AxisData {
							if val != lastLoggedAxes[i] {
								dataChanged = true
								break
							}
						}
					}

					if dataChanged {
						if len(state.AxisData) > 0 {
							for i, val := range state.AxisData {
								axisLog += fmt.Sprintf(" [%d]=%d", i, val)
							}
							log.Println(axisLog)
						} else {
							log.Println("Raw Axes: No axis data received")
						}
						// Update lastLoggedAxes with a copy of the current state
						lastLoggedAxes = make([]int, len(state.AxisData))
						copy(lastLoggedAxes, state.AxisData)
					}
					// logCounter++ // logCounter no longer needed for this logic

				} else { // Not in calibration mode, run normal operation
					// Original raw axis logging - can be removed or kept for debugging
					// if len(state.AxisData) >= 8 { // Assuming at least 8 axes as reported
					// 	log.Printf("Raw Axes: [0]=%d [1]=%d [2]=%d [3]=%d [4]=%d [5]=%d [6]=%d [7]=%d",
					// 		state.AxisData[0], state.AxisData[1], state.AxisData[2], state.AxisData[3],
					// 		state.AxisData[4], state.AxisData[5], state.AxisData[6], state.AxisData[7])
					// } else if len(state.AxisData) > 0 { // Log whatever axes are available
					// 	log.Printf("Raw Axes: %v (Warning: Expected at least 8 axes, got %d)", state.AxisData, len(state.AxisData))
					// } else {
					// 	log.Printf("Raw Axes: No axis data received")
					// }

					processInputsAndCamera()
				}

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