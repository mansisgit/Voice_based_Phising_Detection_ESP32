/*
 * Voice-Based Phishing Detection System - Complete ESP32 Code
 * Project: Real-time Voice Surveillance for Phishing Detection
 * Hardware: ESP32-S, SIM800L GSM Module, VC-02 Voice Recognition Module, Vibration Motor
 * Author: Mansi Chate , Irfa Arshad and Janvi Issar

 * This system listens for phishing-related keywords in voice commands and sends
 * SMS alerts via GSM module while providing haptic feedback through vibration motor.
 */

// =============================== LIBRARY INCLUDES ===============================
#include <HardwareSerial.h>        // For hardware serial communication with modules
#include <SoftwareSerial.h>        // For software serial communication (if needed)
#include <TinyGsmClient.h>          // Library for GSM communication with SIM800L

// =============================== PIN DEFINITIONS ===============================
// ESP32 Pin Assignments (as per circuit diagram)
#define ESP32_VCC_PIN           3.3     // Power supply pin (3.3V)
#define ESP32_GND_PIN           GND     // Ground pin

// SIM800L GSM Module Pin Connections
#define GSM_RX_PIN              16      // ESP32 GPIO16 (U2_RXD) connects to SIM800L TXD
#define GSM_TX_PIN              17      // ESP32 GPIO17 (U2_TXD) connects to SIM800L RXD  
#define GSM_RST_PIN             5       // ESP32 GPIO5 connects to SIM800L RST (reset control)
#define GSM_POWER_PIN           4       // Power control pin for GSM module (optional)

// VC-02 Voice Recognition Module Pin Connections
#define VOICE_RX_PIN            18      // ESP32 GPIO18 connects to VC-02 TX
#define VOICE_TX_PIN            19      // ESP32 GPIO19 connects to VC-02 RX

// Output Device Pin Connections
#define VIBRATION_MOTOR_PIN     15      // ESP32 GPIO15 controls vibration motor via transistor
#define STATUS_LED_PIN          2       // ESP32 GPIO2 controls status LED (built-in LED)
#define BUZZER_PIN              4       // ESP32 GPIO4 for buzzer alert (optional)

// Optional I2C Display Pin Connections (if LCD is used)
#define I2C_SDA_PIN             21      // ESP32 GPIO21 for I2C SDA (display data)
#define I2C_SCL_PIN             22      // ESP32 GPIO22 for I2C SCL (display clock)

// =============================== COMMUNICATION SETUP ===============================
// Initialize hardware serial ports for different modules
HardwareSerial SerialGSM(2);          // UART2 for SIM800L GSM communication
HardwareSerial SerialVoice(1);        // UART1 for VC-02 voice module communication

// TinyGSM configuration for SIM800L module
#define TINY_GSM_MODEM_SIM800           // Define modem type as SIM800
#define TINY_GSM_RX_BUFFER      1024    // Set receive buffer size
TinyGsm modem(SerialGSM);               // Create GSM modem object

// =============================== SYSTEM CONFIGURATION ===============================
// SMS Alert Configuration - IMPORTANT: Replace with actual phone number
#define ALERT_PHONE_NUMBER      "+1234567890"      // Target phone number for SMS alerts
#define SIM_PIN                 ""                  // SIM card PIN (leave empty if no PIN)

// System timing and control variables
unsigned long lastAlertTime = 0;           // Track time of last alert to prevent spam
const unsigned long ALERT_COOLDOWN = 5000; // 5-second cooldown between alerts
bool systemActive = true;                  // System active status flag
bool gsmInitialized = false;               // GSM module initialization status

// =============================== PHISHING KEYWORDS DATABASE ===============================
/*
 * Database of 20 phishing-related keywords commonly used in voice phishing attacks
 * These keywords are checked against incoming voice recognition data
 * Keywords are selected based on common phishing tactics and social engineering phrases
 */
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
    Serial.println("==========================================");
    
    // Initialize all hardware components
    initializePins();           // Setup GPIO pins for outputs
    initializeGSM();           // Initialize GSM module communication
    initializeVoiceModule();   // Initialize voice recognition module
    initializeSystem();        // Final system initialization
    
    // System ready notification
    Serial.println("System initialization complete!");
    Serial.println("Listening for phishing-related voice commands...");
    Serial.println("==========================================");
    
    // Visual indication that system is ready
    blinkStatusLED(3);         // Blink LED 3 times to indicate ready status
}

// =============================== MAIN PROGRAM LOOP ===============================
void loop() {
    // Main program execution loop - runs continuously
    
    // Check for incoming voice commands from VC-02 module
    checkVoiceCommands();
    
    // Monitor GSM module status and handle incoming messages
    monitorGSMStatus();
    
    // Perform system health checks
    performSystemChecks();
    
    // Small delay to prevent overwhelming the system
    delay(100);
}

// =============================== PIN INITIALIZATION FUNCTION ===============================
/*
 * Configure all GPIO pins for their respective functions
 * Set appropriate pin modes for inputs and outputs
 */
void initializePins() {
    Serial.println("Initializing GPIO pins...");
    
    // Configure output pins for various components
    pinMode(VIBRATION_MOTOR_PIN, OUTPUT);   // Vibration motor control pin
    pinMode(STATUS_LED_PIN, OUTPUT);        // Status LED control pin
    pinMode(BUZZER_PIN, OUTPUT);            // Buzzer control pin
    pinMode(GSM_RST_PIN, OUTPUT);           // GSM module reset pin
    pinMode(GSM_POWER_PIN, OUTPUT);         // GSM module power control
    
    // Set initial states for all output pins (all OFF initially)
    digitalWrite(VIBRATION_MOTOR_PIN, LOW); // Vibration motor OFF
    digitalWrite(STATUS_LED_PIN, LOW);      // Status LED OFF
    digitalWrite(BUZZER_PIN, LOW);          // Buzzer OFF
    digitalWrite(GSM_RST_PIN, HIGH);        // GSM module active (reset HIGH)
    digitalWrite(GSM_POWER_PIN, HIGH);      // GSM module powered ON
    
    Serial.println("GPIO pins initialized successfully.");
}

// =============================== GSM MODULE INITIALIZATION ===============================
/*
 * Initialize SIM800L GSM module for SMS communication
 * Configure UART communication and test module connectivity
 */
void initializeGSM() {
    Serial.println("Initializing SIM800L GSM module...");
    
    // Initialize hardware serial communication with GSM module
    // Baud rate: 9600, Data bits: 8, Parity: None, Stop bits: 1
    SerialGSM.begin(9600, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN);
    
    // Power cycle the GSM module for clean initialization
    digitalWrite(GSM_RST_PIN, LOW);     // Assert reset
    delay(1000);                        // Hold reset for 1 second
    digitalWrite(GSM_RST_PIN, HIGH);    // Release reset
    delay(3000);                        // Wait for module to boot up
    
    Serial.println("GSM module hardware initialized.");
    Serial.println("Testing GSM module communication...");
    
    // Initialize GSM modem using TinyGSM library
    if (modem.restart()) {
        Serial.println("GSM module communication successful!");
        
        // Check SIM card status and unlock if PIN is required
        if (strlen(SIM_PIN) > 0 && modem.getSimStatus() != 3) {
            Serial.println("Unlocking SIM card with PIN...");
            modem.simUnlock(SIM_PIN);
        }
        
        // Wait for network registration
        Serial.println("Waiting for network registration...");
        if (modem.waitForNetwork(30000)) {  // Wait up to 30 seconds
            Serial.println("Network registration successful!");
            gsmInitialized = true;
        } else {
            Serial.println("WARNING: Network registration failed!");
        }
    } else {
        Serial.println("ERROR: GSM module initialization failed!");
        Serial.println("Check connections and power supply.");
    }
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
    Serial.println("Voice module is ready to receive commands.");
    
    // Send a test command to voice module (optional)
    // Some VC-02 modules respond to initialization commands
    delay(1000);
}

// =============================== FINAL SYSTEM INITIALIZATION ===============================
/*
 * Complete final system initialization steps
 * Verify all components are ready and working
 */
void initializeSystem() {
    Serial.println("Performing final system initialization...");
    
    // Test all output components briefly
    testAllOutputs();
    
    // Display phishing keywords being monitored
    displayMonitoredKeywords();
    
    // Set system as active and ready
    systemActive = true;
    
    Serial.println("All systems initialized and ready!");
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
            // PHISHING DETECTED - Execute alert sequence
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
    return false;
}

// =============================== PHISHING ALERT HANDLER ===============================
/*
 * Handle detected phishing attempt
 * Execute all alert mechanisms: SMS, vibration, visual indicators
 */
void handlePhishingDetection(String detectedCommand) {
    // Check if enough time has passed since last alert (prevent spam)
    unsigned long currentTime = millis();
    if (currentTime - lastAlertTime < ALERT_COOLDOWN) {
        Serial.println("Alert cooldown active - skipping duplicate alert.");
        return;
    }
    
    Serial.println("=== PHISHING ATTEMPT DETECTED ===");
    Serial.println("Executing emergency alert sequence...");
    
    // Update last alert time
    lastAlertTime = currentTime;
    
    // Execute immediate physical alerts
    activateVibrationAlert();    // Vibrate motor for haptic feedback
    activateVisualAlert();       // Flash LED for visual indication
    activateAudioAlert();        // Sound buzzer for audio alert
    
    // Send SMS alert via GSM module
    sendSMSAlert(detectedCommand);
    
    // Log the complete incident
    logPhishingIncident(detectedCommand);
    
    Serial.println("Alert sequence completed.");
    Serial.println("===============================");
}

// =============================== VIBRATION ALERT FUNCTION ===============================
/*
 * Activate vibration motor for haptic feedback
 * Provides immediate physical alert to user
 */
void activateVibrationAlert() {
    Serial.println("Activating vibration motor...");
    
    // Create vibration pattern: ON-OFF-ON-OFF-ON
    for (int i = 0; i < 3; i++) {
        digitalWrite(VIBRATION_MOTOR_PIN, HIGH);  // Turn ON vibration motor
        delay(500);                               // Vibrate for 500ms
        digitalWrite(VIBRATION_MOTOR_PIN, LOW);   // Turn OFF vibration motor
        delay(200);                               // Pause for 200ms
    }
    
    Serial.println("Vibration alert completed.");
}

// =============================== VISUAL ALERT FUNCTION ===============================
/*
 * Activate LED visual alert
 * Provides visual indication of phishing detection
 */
void activateVisualAlert() {
    Serial.println("Activating visual LED alert...");
    
    // Create flashing pattern: fast blinks to get attention
    for (int i = 0; i < 10; i++) {
        digitalWrite(STATUS_LED_PIN, HIGH);  // Turn ON LED
        delay(100);                          // ON for 100ms
        digitalWrite(STATUS_LED_PIN, LOW);   // Turn OFF LED
        delay(100);                          // OFF for 100ms
    }
    
    Serial.println("Visual alert completed.");
}

// =============================== AUDIO ALERT FUNCTION ===============================
/*
 * Activate buzzer for audio alert
 * Provides audible warning of phishing detection
 */
void activateAudioAlert() {
    Serial.println("Activating audio buzzer alert...");
    
    // Create audio pattern: beep sequence
    for (int i = 0; i < 5; i++) {
        digitalWrite(BUZZER_PIN, HIGH);      // Turn ON buzzer
        delay(200);                          // Beep for 200ms
        digitalWrite(BUZZER_PIN, LOW);       // Turn OFF buzzer
        delay(100);                          // Silence for 100ms
    }
    
    Serial.println("Audio alert completed.");
}

// =============================== SMS ALERT FUNCTION ===============================
/*
 * Send SMS alert via SIM800L GSM module
 * Notifies designated contact of phishing attempt
 */
void sendSMSAlert(String detectedCommand) {
    Serial.println("Sending SMS alert...");
    
    // Check if GSM module is properly initialized
    if (!gsmInitialized) {
        Serial.println("ERROR: GSM module not initialized - cannot send SMS");
        return;
    }
    
    // Create alert message with timestamp and detected command
    String alertMessage = "PHISHING ALERT: Suspicious voice command detected: \"";
    alertMessage += detectedCommand;
    alertMessage += "\". Time: ";
    alertMessage += String(millis() / 1000);  // Simple timestamp in seconds
    alertMessage += "s. Location: ESP32 Device ID.";
    
    // Attempt to send SMS
    Serial.print("Sending to: ");
    Serial.println(ALERT_PHONE_NUMBER);
    Serial.print("Message: ");
    Serial.println(alertMessage);
    
    // Use TinyGSM library to send SMS
    if (modem.sendSMS(ALERT_PHONE_NUMBER, alertMessage)) {
        Serial.println("SMS alert sent successfully!");
    } else {
        Serial.println("ERROR: Failed to send SMS alert!");
        Serial.println("Check network connection and phone number format.");
    }
}

// =============================== GSM STATUS MONITORING ===============================
/*
 * Monitor GSM module status and connectivity
 * Handle any GSM-related maintenance tasks
 */
void monitorGSMStatus() {
    // Check GSM status every 30 seconds (non-blocking)
    static unsigned long lastGSMCheck = 0;
    unsigned long currentTime = millis();
    
    if (currentTime - lastGSMCheck > 30000) {  // 30 seconds
        lastGSMCheck = currentTime;
        
        if (gsmInitialized) {
            // Check network registration status
            if (modem.isNetworkConnected()) {
                // Network is connected - system operational
                digitalWrite(STATUS_LED_PIN, HIGH);  // Solid LED indicates good connection
            } else {
                // Network disconnected - attempt reconnection
                Serial.println("Network disconnected - attempting reconnection...");
                digitalWrite(STATUS_LED_PIN, LOW);
                
                if (modem.waitForNetwork(10000)) {  // Try for 10 seconds
                    Serial.println("Network reconnected successfully!");
                } else {
                    Serial.println("Network reconnection failed!");
                }
            }
        }
    }
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
        
        // Check GSM status
        Serial.print("GSM status: ");
        Serial.println(gsmInitialized ? "OK" : "ERROR");
        
        // Check voice module status
        Serial.print("Voice module: ");
        Serial.println(systemActive ? "ACTIVE" : "INACTIVE");
        
        Serial.println("--- Health Check Complete ---");
    }
}

// =============================== UTILITY FUNCTIONS ===============================

/*
 * Test all output components during initialization
 * Verify that all hardware is working correctly
 */
void testAllOutputs() {
    Serial.println("Testing all output components...");
    
    // Test vibration motor
    Serial.print("Testing vibration motor... ");
    digitalWrite(VIBRATION_MOTOR_PIN, HIGH);
    delay(500);
    digitalWrite(VIBRATION_MOTOR_PIN, LOW);
    Serial.println("OK");
    
    // Test status LED
    Serial.print("Testing status LED... ");
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(500);
    digitalWrite(STATUS_LED_PIN, LOW);
    Serial.println("OK");
    
    // Test buzzer
    Serial.print("Testing buzzer... ");
    digitalWrite(BUZZER_PIN, HIGH);
    delay(300);
    digitalWrite(BUZZER_PIN, LOW);
    Serial.println("OK");
    
    Serial.println("All output tests completed successfully.");
}

/*
 * Blink status LED a specified number of times
 * Used for system status indication
 */
void blinkStatusLED(int times) {
    for (int i = 0; i < times; i++) {
        digitalWrite(STATUS_LED_PIN, HIGH);
        delay(200);
        digitalWrite(STATUS_LED_PIN, LOW);
        delay(200);
    }
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
    Serial.print("GSM Status: ");
    Serial.println(gsmInitialized ? "CONNECTED" : "DISCONNECTED");
    Serial.println("Actions Taken: Vibration + LED + Buzzer + SMS Alert");
    Serial.println("==================");
}

// =============================== END OF PROGRAM ===============================

/*
 * INSTALLATION NOTES:
 * 
 * 1. Install required libraries in Arduino IDE:
 *    - TinyGSM library for GSM communication
 *    - ESP32 board package
 * 
 * 2. Hardware connections as per circuit diagram:
 *    - ESP32 GPIO16 ↔ SIM800L TXD
 *    - ESP32 GPIO17 ↔ SIM800L RXD
 *    - ESP32 GPIO18 ↔ VC-02 TX
 *    - ESP32 GPIO19 ↔ VC-02 RX
 *    - ESP32 GPIO15 ↔ Vibration Motor (via transistor)
 *    - ESP32 GPIO2 ↔ Status LED
 * 
 * 3. Configuration before upload:
 *    - Change ALERT_PHONE_NUMBER to actual phone number
 *    - Set SIM_PIN if your SIM card requires it
 *    - Verify pin connections match your hardware setup
 * 
 * 4. Power requirements:
 *    - ESP32: 3.3V
 *    - SIM800L: 4V (separate power supply recommended)
 *    - VC-02: 3.3V or 5V (check module specifications)
 * 
 * 5. Testing:
 *    - Open Serial Monitor at 115200 baud
 *    - Verify all initialization messages
 *    - Test with VC-02 voice commands containing keywords
 */