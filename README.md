# Buddy Beacon

Buddy Beacon is a hardware project using ESPNOW to broadcast a beacon signal from a transmitter to one or more receivers.
The project is designed to be used with the ESP32-S2 microcontrollers.

The purpose of this project is to allow your pet to press a button on the transmitter device to send a beacon signal to the receiver device, which in this case is a simple PWM LED lamp.

## Repository Structure
- README.md: This file
- Makefile: Makefile for building the project
- src/: Source code for the project
- src/transmitter: Source code for the transmitter device
- src/receiver: Source code for the receiver device

## Transmitter Device
The transmitter device is a simple ESP32 microcontroller with a button connected to a GPIO pin.
When the button is pressed, the device will broadcast a beacon signal to all listening devices.

### Interface Pins
- GPIO_PIN: 32 (Momentary button)

### Models
- [Transmitter](https://www.thingiverse.com/thing:4755888)

## Receiver Device
The receiver device is a simple ESP32 microcontroller with a PWM LED connected to a GPIO pin.
When the receiver device receives a beacon signal from the transmitter device, it will turn on the LED for a short period of time.

# Interface Pins
- GPIO_PIN: 33 (PWM LED)

### Models
- [Receiver](https://www.printables.com/model/825409-waves-midcentury-led-lamp)