/*
 * INSTALLATION NOTES FOR VIBRATION-ONLY VERSION:
 * 
 * 1. Hardware Requirements:
 *    - ESP32-S microcontroller
 *    - VC-02 voice recognition module  
 *    - Small vibration motor (3V-5V)
 *    - NPN transistor (2N2222 or similar) for motor driver
 *    - 1kΩ resistor for transistor base
 *    - Breadboard or PCB for connections
 * 
 * 2. Hardware connections:
 *    - ESP32 GPIO18 ↔ VC-02 TX (voice data to ESP32)
 *    - ESP32 GPIO19 ↔ VC-02 RX (commands from ESP32)
 *    - ESP32 GPIO15 ↔ 1kΩ resistor ↔ Transistor Base
 *    - Transistor Collector ↔ Vibration Motor (+)
 *    - Transistor Emitter ↔ Ground
 *    - Vibration Motor (-) ↔ Ground
 *    - ESP32 3V3 ↔ VC-02 VCC
 *    - ESP32 GND ↔ VC-02 GND ↔ Common Ground
 * 
 * 3. Software setup:
 *    - Install ESP32 board package in Arduino IDE
 *    - No additional libraries required (uses built-in HardwareSerial)
 *    - Upload this code to ESP32
 * 
 * 4. VC-02 Voice Module Training:
 *    - Train the VC-02 module with the 20 phishing keywords
 *    - Refer to VC-02 documentation for training procedure
 *    - Test each keyword to ensure proper recognition
 * 
 * 5. Testing:
 *    - Open Serial Monitor at 115200 baud
 *    - Verify initialization messages
 *    - Speak phishing keywords to test vibration response
 *    - Verify 5-second vibration pattern: Long-Short-Short-Long-Long
 * 
 * 6. Power Requirements:
 *    - ESP32: 3.3V, ~200mA during operation
 *    - VC-02: 3.3V, ~50mA
 *    - Vibration Motor: 3-5V, ~100mA through transistor
 *    - Total system: ~350mA at 3.3V
 * 
 * 7. Deployment:
 *    - System operates continuously once powered
 *    - No internet connection required (offline operation)
 *    - Suitable for elderly care, personal security, ATM areas
 *    - Vibration provides discrete alert without disturbing others
 */
