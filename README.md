# LinkCom
## Project Overview
This project implements a bidirectional communication system using STM32 microcontrollers, an LCD, a keypad, and GPIO signals. The system enables users to send and receive predefined messages through a simple interface.
## Project Images

### 1️⃣ LinkCom Project

This is the actual LinkCom hardware setup, showcasing microcontrollers, LCD displays, and wiring connections.

![IMG_5203](https://github.com/user-attachments/assets/3745638a-c5e3-4d94-a1d5-cfb75e5dd274)
### 2️⃣ Fusion 360 Model

A 3D CAD model of the LinkCom project designed in **Fusion 360**, providing a structured visualization of the setup.
![image](https://github.com/user-attachments/assets/37ee510d-e32a-4fd7-9e90-f38fc952ad02)

### 3️⃣ Circuit Schematic (Altium)

This is the **electrical schematic** of the LinkCom project, designed in **Altium**, detailing the microcontroller, keypad, LCD display, and other electronic components.
![image](https://github.com/user-attachments/assets/86be4d06-e41e-40d5-b75b-0e8066670dbd)

## Features
- **Message Transmission and Reception**: Sends and receives 4-bit messages using GPIO pins.
- **LCD Integration**: Displays messages being sent and received.
- **Keypad Interface**: Users can select predefined messages via a keypad.
- **Visual and Audio Feedback**: Includes a buzzer and LED indicators for pulse detection.

## Hardware Requirements
- STM32 microcontroller (tested on STM32F4 series)
- 16x2 LCD display
- 4x3 Keypad
- GPIO-connected LEDs and buzzer
- Required wiring and power supply

## Software Details
### Main Components
1. **Message Transmission and Reception**:
   - Pulses are sent and received via GPIO pins to communicate binary data.
   - A predefined set of messages is encoded in 4-bit binary.

2. **LCD Management**:
   - Displays outgoing and incoming messages.
   - Includes initialization and control routines for 4-bit communication.

3. **Keypad Input**:
   - Scans rows and columns to detect key presses.
   - Allows users to select and send messages.

4. **UART Logging** (Optional):
   - Debugging and message logging over UART.

### System States
- **IDLE**: Waiting for user input or incoming messages.
- **TRANSMITTING**: Sending a user-selected message.
- **RECEIVING**: Processing and displaying an incoming message.
- **COMPLETED**: Indicates that the communication process is done.

### Code Highlights
- **Pulse Detection and Handling**:
  ```c
  void sendPulse(uint32_t duration_ms);
  bool waitForPulse(uint32_t timeout_ms);
  ```

- **Keypad Input**:
  ```c
  char readKeypad(void);
  ```

- **LCD Control**:
  ```c
  void LCD_Init(void);
  void LCD_Print(char *str);
  void LCD_SetCursor(uint8_t row, uint8_t col);
  ```

- **Main Application Logic**:
  ```c
  while (1) {
      int key = readKeypad() - 1;
      if (key >= 0 && key < 9) {
          LCD_Print("Sending: ");
          startMessageTransmission();
      }
      listenForMessage();
      if (messageReceived) {
          LCD_Print("Received: ");
      }
  }
  ```

## Predefined Messages
- **Row 1**: "Hello", "Good Morning", "Good Night"
- **Row 2**: "Breakfast", "Lunch", "Dinner"
- **Row 3**: "Yes", "No", "Thanks"

## Setup and Usage
1. Connect the hardware components as specified in the circuit diagram.
2. Flash the firmware onto the STM32 microcontroller.
3. Use the keypad to select messages for transmission.
4. Observe the LCD for feedback on sent and received messages.

---

For further inquiries or contributions, feel free to reach out or submit an issue.
