#include <TinyGsmClient.h>
#include <HardwareSerial.h>

// Pin configuration
#define VC02_RX 16    // Connect to VC-02 TX
#define VC02_TX 17    // Connect to VC-02 RX
#define VIBRATION_PIN 15
#define MODEM_RST 5
#define MODEM_TX 27
#define MODEM_RX 26
#define SMS_TARGET "+911234567890"  // Replace with actual number

HardwareSerial VC02Serial(2);   // Use UART2 for VC-02
HardwareSerial SerialAT(1);     // Use UART1 for SIM800L

#define TINY_GSM_MODEM_SIM800
TinyGsm modem(SerialAT);

// Phishing keywords (update as needed)
const char* phishingKeywords[] = {
  "password",
  "OTP",
  "ATM",
  "account",
  "bank"
};
const int numKeywords = sizeof(phishingKeywords)/sizeof(phishingKeywords[0]);

void setup() {
  Serial.begin(115200);
  VC02Serial.begin(9600, SERIAL_8N1, VC02_RX, VC02_TX);
  SerialAT.begin(9600, SERIAL_8N1, MODEM_RX, MODEM_TX);

  pinMode(VIBRATION_PIN, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  digitalWrite(MODEM_RST, HIGH); // Keep GSM module in reset

  // Wait for GSM to initialize
  delay(3000);
  modem.restart();

  Serial.println("Voice Phishing Detection System Started");
}

void loop() {
  // Listen for voice commands from VC-02 module
  if (VC02Serial.available()) {
    String command = VC02Serial.readStringUntil('\n');
    command.trim();
    Serial.print("Received Voice Command: ");
    Serial.println(command);

    if (isPhishingCommand(command)) {
      Serial.println("Phishing keyword detected!");

      // Vibrate motor for feedback
      digitalWrite(VIBRATION_PIN, HIGH);
      delay(500);  // Vibrate for 0.5 seconds
      digitalWrite(VIBRATION_PIN, LOW);

      // Send SMS alert
      String smsMsg = "Phishing detected: " + command;
      if (modem.sendSMS(SMS_TARGET, smsMsg)) {
        Serial.println("SMS alert sent successfully!");
      } else {
        Serial.println("Failed to send SMS.");
      }
      delay(2000); // Avoid repeated alerts for same command
    }
  }
}

// Function to check for phishing keywords in voice commands
bool isPhishingCommand(String cmd) {
  cmd.toLowerCase();
  for (int i = 0; i < numKeywords; i++) {
    if (cmd.indexOf(phishingKeywords[i]) >= 0) {
      return true;
    }
  }
  return false;
}
