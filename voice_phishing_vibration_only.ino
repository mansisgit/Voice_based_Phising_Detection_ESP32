/*
 * Voice-Based Phishing Detection System - ESP32 Code (Vibration Alert Only)
 * Project: Real-time Voice Surveillance for Phishing Detection
 * Hardware: ESP32-S, VC-02 Voice Recognition Module, Vibration Motor
 * Author: Mansi Chate , Janvi Issar and Irfa Arshad
 
 * This system listens for phishing-related keywords in voice commands and provides
 * haptic feedback through vibration motor when phishing keywords are detected.
 */
// =============================== LIBRARY INCLUDES ===============================
#include <HardwareSerial.h>        // For hardware serial communication with VC-02 module

// =============================== PIN DEFINITIONS ===============================
// ESP32 Pin Assignments (as per your circuit diagram)
#define ESP32_VCC_PIN           3.3     // Power supply pin (3.3V)
#define ESP32_GND_PIN           GND     // Ground pin

// VC-02 Voice Recognition Module Pin Connections
#define VOICE_RX_PIN            18      // ESP32 GPIO18 connects to VC-02 TX
#define VOICE_TX_PIN            19      // ESP32 GPIO19 connects to VC-02 RX

// Vibration Motor Pin Connection (ONLY OUTPUT DEVICE)
#define VIBRATION_MOTOR_PIN     15      // ESP32 GPIO15 controls vibration motor via transistor

// =============================== COMMUNICATION SETUP ===============================
// Initialize hardware serial port for VC-02 voice module
HardwareSerial SerialVoice(1);        // UART1 for VC-02 voice module communication

// =============================== SYSTEM CONFIGURATION ===============================
// System timing and control variables
unsigned long lastAlertTime = 0;           // Track time of last alert to prevent spam
const unsigned long ALERT_COOLDOWN = 3000; // 3-second cooldown between vibration alerts
bool systemActive = true;                  // System active status flag

// =============================== PHISHING KEYWORDS DATABASE ===============================

 //Database of 20 phishing-related keywords commonly used in voice phishing attacks
 
const char* phishingKeywords[20] = {
    "password",        // Request for password information
    "otp",            // One-Time Password requests
    "atm",            // ATM-related scams
    "pin",            // PIN number requests  
    "account",        // Bank account information
    "verify",         // Account verification scams
    "blocked",        // "Your account is blocked" scams
    "suspended",      // "Account suspended" threats
    "urgent",         // Creating urgency in victims
    "immediately",    // Pressure tactics
    "bank",           // Bank impersonation
    "credit",         // Credit card scams
    "lottery",        // Lottery/prize scams
    "winner",         // "You've won" scams
    "tax",            // Tax-related scams
    "refund",         // Fake refund offers
    "expired",        // "Your card/account expired" scams
    "security",       // Security-related social engineering
    "confirm",        // Confirmation requests for sensitive data
    "deposit"         // Deposit-related financial scams
};

// Calculate total number of keywords for loop iterations
const int TOTAL_KEYWORDS = sizeof(phishingKeywords) / sizeof(phishingKeywords[0]);

// =============================== SYSTEM INITIALIZATION ===============================
void setup() {
    // Initialize main serial communication for debugging and monitoring
    Serial.begin(115200);
    delay(1000);
    
    // Print system startup message
    Serial.println("==========================================");
    Serial.println("Voice-Based Phishing Detection System");
    Serial.println("ESP32 Microcontroller - Version 1.0");
    Serial.println("VIBRATION ALERT ONLY VERSION");
    Serial.println("==========================================");
    
    // Initialize all hardware components
    initializePins();           // Setup GPIO pins for vibration motor
    initializeVoiceModule();    // Initialize voice recognition module
    initializeSystem();         // Final system initialization
    
    // System ready notification
    Serial.println("System initialization complete!");
    Serial.println("Listening for phishing-related voice commands...");
    Serial.println("Only vibration motor will activate on detection.");
    Serial.println("==========================================");
    
    // Test vibration motor to indicate system is ready
    testVibrationMotor();
}

// =============================== MAIN PROGRAM LOOP ===============================
void loop() {
    // Main program execution loop - runs continuously
    
    // Check for incoming voice commands from VC-02 module
    checkVoiceCommands();
    
    // Perform system health checks periodically
    performSystemChecks();
    
    // Small delay to prevent overwhelming the system
    delay(100);
}

// =============================== PIN INITIALIZATION FUNCTION ===============================
/*
 * Configure GPIO pin for vibration motor control
 * Set appropriate pin mode for output
 */
void initializePins() {
    Serial.println("Initializing GPIO pins...");
    
    // Configure output pin for vibration motor only
    pinMode(VIBRATION_MOTOR_PIN, OUTPUT);   // Vibration motor control pin
    
    // Set initial state for vibration motor (OFF initially)
    digitalWrite(VIBRATION_MOTOR_PIN, LOW); // Vibration motor OFF
    
    Serial.println("GPIO pins initialized successfully.");
    Serial.print("Vibration motor connected to GPIO");
    Serial.println(VIBRATION_MOTOR_PIN);
}

// =============================== VOICE MODULE INITIALIZATION ===============================
/*
 * Initialize VC-02 voice recognition module
 * Configure UART communication for receiving voice commands
 */
void initializeVoiceModule() {
    Serial.println("Initializing VC-02 voice recognition module...");
    
    // Initialize hardware serial communication with voice module
    // Baud rate: 9600 (standard for VC-02), Data: 8-bit, Parity: None, Stop: 1-bit
    SerialVoice.begin(9600, SERIAL_8N1, VOICE_RX_PIN, VOICE_TX_PIN);
    
    // Clear any existing data in the serial buffer
    while (SerialVoice.available()) {
        SerialVoice.read();
    }
    
    Serial.println("Voice recognition module initialized.");
    Serial.print("VC-02 RX connected to ESP32 GPIO");
    Serial.println(VOICE_RX_PIN);
    Serial.print("VC-02 TX connected to ESP32 GPIO");
    Serial.println(VOICE_TX_PIN);
    Serial.println("Voice module is ready to receive commands.");
    
    // Wait for voice module to stabilize
    delay(2000);
}

// =============================== FINAL SYSTEM INITIALIZATION ===============================
/*
 * Complete final system initialization steps
 * Verify all components are ready and working
 */
void initializeSystem() {
    Serial.println("Performing final system initialization...");
    
    // Display phishing keywords being monitored
    displayMonitoredKeywords();
    
    // Set system as active and ready
    systemActive = true;
    
    Serial.println("Voice-based phishing detection system ready!");
    Serial.println("Only vibration motor will alert on keyword detection.");
}

// =============================== VOICE COMMAND PROCESSING ===============================
/*
 * Check for incoming voice commands from VC-02 module
 * Process received commands and check for phishing keywords
 */
void checkVoiceCommands() {
    // Check if voice module has sent any data
    if (SerialVoice.available()) {
        // Read the incoming voice command data
        String voiceCommand = SerialVoice.readStringUntil('\n');
        voiceCommand.trim();  // Remove whitespace and newline characters
        voiceCommand.toLowerCase();  // Convert to lowercase for comparison
        
        // Log the received voice command for debugging
        Serial.print("Voice Command Received: \"");
        Serial.print(voiceCommand);
        Serial.println("\"");
        
        // Check if the command contains any phishing keywords
        if (isPhishingDetected(voiceCommand)) {
            // PHISHING DETECTED - Execute vibration alert only
            handlePhishingDetection(voiceCommand);
        } else {
            // Normal command - no threat detected
            Serial.println("Command processed - No threat detected.");
        }
    }
}

// =============================== PHISHING DETECTION ALGORITHM ===============================
/*
 * Analyze voice command for phishing-related keywords
 * Returns true if any phishing keyword is detected
 */
bool isPhishingDetected(String command) {
    Serial.println("Analyzing command for phishing keywords...");
    
    // Convert command to lowercase for case-insensitive comparison
    command.toLowerCase();
    
    // Check each phishing keyword against the voice command
    for (int i = 0; i < TOTAL_KEYWORDS; i++) {
        // Use indexOf to check if keyword exists in the command
        if (command.indexOf(phishingKeywords[i]) >= 0) {
            // Keyword found - log which keyword triggered detection
            Serial.print("PHISHING KEYWORD DETECTED: \"");
            Serial.print(phishingKeywords[i]);
            Serial.println("\"");
            return true;  // Return immediately on first match
        }
    }
    
    // No phishing keywords found
    Serial.println("No phishing keywords detected in command.");
    return false;
}

// =============================== PHISHING ALERT HANDLER ===============================

 // Handle detected phishing attempt
void handlePhishingDetection(String detectedCommand) {
    // Check if enough time has passed since last alert (prevent spam)
    unsigned long currentTime = millis();
    if (currentTime - lastAlertTime < ALERT_COOLDOWN) {
        Serial.println("Alert cooldown active - skipping duplicate vibration.");
        return;
    }
    
    Serial.println("=== PHISHING ATTEMPT DETECTED ===");
    Serial.println("Executing vibration alert sequence...");
    
    // Update last alert time
    lastAlertTime = currentTime;
    
    // Execute ONLY vibration alert (no other alerts)
    activateVibrationAlert();    // Vibrate motor for haptic feedback
    
    // Log the complete incident
    logPhishingIncident(detectedCommand);
    
    Serial.println("Vibration alert sequence completed.");
    Serial.println("=================================");
}

// =============================== VIBRATION ALERT FUNCTION ===============================
void activateVibrationAlert() {
    Serial.println("ACTIVATING VIBRATION MOTOR ALERT...");
    
    // Create distinctive vibration pattern for phishing detection
    // Pattern: Long-Short-Short-Long-Long (Morse code style)
    
    // Long vibration (1 second)
    Serial.println("Vibration: LONG pulse");
    digitalWrite(VIBRATION_MOTOR_PIN, HIGH);  // Turn ON vibration motor
    delay(1000);                              // Vibrate for 1 second
    digitalWrite(VIBRATION_MOTOR_PIN, LOW);   // Turn OFF vibration motor
    delay(300);                               // Pause for 300ms
    
    // Short vibration (300ms)
    Serial.println("Vibration: SHORT pulse");
    digitalWrite(VIBRATION_MOTOR_PIN, HIGH);  // Turn ON vibration motor
    delay(300);                               // Vibrate for 300ms
    digitalWrite(VIBRATION_MOTOR_PIN, LOW);   // Turn OFF vibration motor
    delay(300);                               // Pause for 300ms
    
    // Short vibration (300ms)
    Serial.println("Vibration: SHORT pulse");
    digitalWrite(VIBRATION_MOTOR_PIN, HIGH);  // Turn ON vibration motor
    delay(300);                               // Vibrate for 300ms
    digitalWrite(VIBRATION_MOTOR_PIN, LOW);   // Turn OFF vibration motor
    delay(300);                               // Pause for 300ms
    
    // Long vibration (1 second)
    Serial.println("Vibration: LONG pulse");
    digitalWrite(VIBRATION_MOTOR_PIN, HIGH);  // Turn ON vibration motor
    delay(1000);                              // Vibrate for 1 second
    digitalWrite(VIBRATION_MOTOR_PIN, LOW);   // Turn OFF vibration motor
    delay(300);                               // Pause for 300ms
    
    // Long vibration (1 second)
    Serial.println("Vibration: LONG pulse");
    digitalWrite(VIBRATION_MOTOR_PIN, HIGH);  // Turn ON vibration motor
    delay(1000);                              // Vibrate for 1 second
    digitalWrite(VIBRATION_MOTOR_PIN, LOW);   // Turn OFF vibration motor
    
    Serial.println("Vibration alert pattern completed.");
    Serial.println("Total alert duration: ~5 seconds");
}

// =============================== SYSTEM HEALTH CHECKS ===============================
/*
 * Perform periodic system health checks
 * Monitor system performance and component status
 */
void performSystemChecks() {
    // Perform health checks every 60 seconds
    static unsigned long lastHealthCheck = 0;
    unsigned long currentTime = millis();
    
    if (currentTime - lastHealthCheck > 60000) {  // 60 seconds
        lastHealthCheck = currentTime;
        
        Serial.println("--- System Health Check ---");
        
        // Check memory usage
        Serial.print("Free heap memory: ");
        Serial.print(ESP.getFreeHeap());
        Serial.println(" bytes");
        
        // Check system uptime
        Serial.print("System uptime: ");
        Serial.print(currentTime / 1000);
        Serial.println(" seconds");
        
        // Check voice module status
        Serial.print("Voice module: ");
        Serial.println(systemActive ? "ACTIVE" : "INACTIVE");
        
        // Check vibration motor status
        Serial.print("Vibration motor pin: GPIO");
        Serial.println(VIBRATION_MOTOR_PIN);
        
        Serial.println("--- Health Check Complete ---");
    }
}

// =============================== UTILITY FUNCTIONS ===============================

/*
 * Test vibration motor during initialization
 * Verify that vibration motor hardware is working correctly
 */
void testVibrationMotor() {
    Serial.println("Testing vibration motor...");
    
    // Simple test: 3 short pulses
    for (int i = 0; i < 3; i++) {
        Serial.print("Test pulse ");
        Serial.print(i + 1);
        Serial.println("/3");
        
        digitalWrite(VIBRATION_MOTOR_PIN, HIGH);  // Turn ON
        delay(200);                               // 200ms pulse
        digitalWrite(VIBRATION_MOTOR_PIN, LOW);   // Turn OFF
        delay(300);                               // 300ms pause
    }
    
    Serial.println("Vibration motor test completed successfully.");
    Serial.println("System ready for phishing detection.");
}

/*
 * Display list of monitored phishing keywords
 * Shows what the system is actively monitoring for
 */
void displayMonitoredKeywords() {
    Serial.println("Monitored Phishing Keywords:");
    Serial.println("-----------------------------");
    
    for (int i = 0; i < TOTAL_KEYWORDS; i++) {
        Serial.print(i + 1);
        Serial.print(". ");
        Serial.println(phishingKeywords[i]);
    }
    
    Serial.println("-----------------------------");
    Serial.print("Total keywords monitored: ");
    Serial.println(TOTAL_KEYWORDS);
    Serial.println("Alert method: VIBRATION MOTOR ONLY");
}

/*
 * Log phishing incident with details
 * Create record of detected phishing attempt
 */
void logPhishingIncident(String command) {
    Serial.println("=== INCIDENT LOG ===");
    Serial.print("Timestamp: ");
    Serial.print(millis() / 1000);
    Serial.println(" seconds since boot");
    Serial.print("Detected Command: \"");
    Serial.print(command);
    Serial.println("\"");
    Serial.print("System Status: ");
    Serial.println(systemActive ? "ACTIVE" : "INACTIVE");
    Serial.println("Action Taken: VIBRATION MOTOR ALERT ONLY");
    Serial.println("Alert Duration: ~5 seconds");
    Serial.println("Alert Pattern: Long-Short-Short-Long-Long");
    Serial.println("==================");
}

// =============================== END OF PROGRAM ===============================

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