/**
 * ============================================================
 * SmartDrive Assist - ESP32 Peripheral Controller
 * ============================================================
 * 
 * Receives commands from STM32L432KC via UART and:
 * - Displays status on Digilent PmodCLP LCD
 * - Relays motor commands to Dagu Wild Thumper motor driver
 * 
 * Hardware:
 * - ESP32 DevKit
 * - Digilent PmodCLP (PB200-142) - HD44780 16x2 LCD
 * - Dagu Wild Thumper Motor Controller (9600 baud serial)
 * 
 * Author: SmartDrive Assist Project
 * Date: 2025
 * ============================================================
 */

#include <LiquidCrystal.h>
#include <HardwareSerial.h>

// ============================================================
//                     PIN DEFINITIONS
// ============================================================

// ---------- PmodCLP LCD Pins (4-bit mode) ----------
// Control signals from J2 header
#define LCD_RS    13    // J2 Pin 1 - Register Select
#define LCD_EN    12    // J2 Pin 3 - Enable
// Note: J2 Pin 2 (R/W) MUST be connected to GND!

// Data bus from J1 header (bottom row: pins 7-10)
#define LCD_D4    14    // J1 Pin 7  - Data bit 4
#define LCD_D5    27    // J1 Pin 8  - Data bit 5
#define LCD_D6    26    // J1 Pin 9  - Data bit 6
#define LCD_D7    25    // J1 Pin 10 - Data bit 7

// ---------- UART Pins ----------
// STM32 Communication (UART2)
#define STM32_RX  16    // GPIO16 - Receives from STM32 TX (PB6)
#define STM32_TX  17    // GPIO17 - Transmits to STM32 RX (PB7)

// Motor Driver Communication (UART1)
#define MOTOR_TX   4    // GPIO4 - Transmits to Motor Driver RX
#define MOTOR_RX   5    // GPIO5 - Receives from Motor Driver TX (optional)

// ---------- Status LED ----------
#define STATUS_LED  2   // Built-in LED on most ESP32 DevKit boards

// ============================================================
//                   PROTOCOL DEFINITIONS
//              (Must match STM32 definitions!)
// ============================================================

#define PROTO_START     0xAA    // Frame start byte

// Message types
#define MSG_SPEED       0x01    // Speed value update
#define MSG_MODE        0x02    // Mode update
#define MSG_DISTANCE    0x03    // Ultrasonic distance
#define MSG_MOTOR       0x04    // Motor values (M1, M2)
#define MSG_STATUS      0x05    // System status flags
#define MSG_MOTOR_CMD   0x06    // Motor command to relay

// Mode codes
#define MODE_STOP       0x00
#define MODE_FORWARD    0x01
#define MODE_BACKWARD   0x02
#define MODE_LEFT       0x03
#define MODE_RIGHT      0x04
#define MODE_SPIN_LEFT  0x05
#define MODE_SPIN_RIGHT 0x06
#define MODE_PRECISION  0x07

// Motor driver commands (Dagu Wild Thumper)
#define CMD_M1_FORWARD  0xCA
#define CMD_M1_BACKWARD 0xC9
#define CMD_M1_STOP     0xC8
#define CMD_M2_FORWARD  0xC2
#define CMD_M2_BACKWARD 0xC1
#define CMD_M2_STOP     0xC0

// ============================================================
//                    GLOBAL OBJECTS
// ============================================================

// LCD (RS, EN, D4, D5, D6, D7)
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

// Hardware Serial ports
HardwareSerial STM32Serial(2);   // UART2 for STM32 communication
HardwareSerial MotorSerial(1);   // UART1 for motor driver

// ============================================================
//                    STATE VARIABLES
// ============================================================

// Protocol state machine
enum RxState {
    WAIT_START,
    WAIT_TYPE,
    WAIT_LENGTH,
    WAIT_DATA,
    WAIT_CHECKSUM
};

RxState rxState = WAIT_START;
uint8_t msgType = 0;
uint8_t msgLength = 0;
uint8_t dataBuffer[16];
uint8_t dataIndex = 0;

// System state (received from STM32)
uint8_t currentSpeed = 64;          // 0-127
uint8_t currentMode = MODE_STOP;
uint8_t currentDistance = 0;        // cm
int8_t motorM1 = 0;                 // -127 to +127
int8_t motorM2 = 0;                 // -127 to +127
bool precisionMode = false;
bool obstacleWarning = false;
bool emergencyStop = false;

// Timing
unsigned long lastLcdUpdate = 0;
unsigned long lastHeartbeat = 0;
unsigned long lastMessageTime = 0;

const unsigned long LCD_UPDATE_INTERVAL = 200;      // ms
const unsigned long HEARTBEAT_INTERVAL = 500;       // ms
const unsigned long COMM_TIMEOUT = 3000;            // ms

// Statistics
unsigned long messagesReceived = 0;
unsigned long checksumErrors = 0;
// ============================================================
//                 CUSTOM LCD CHARACTERS
// ============================================================

// Arrow Up (Forward)
byte charArrowUp[8] = {
    0b00100,
    0b01110,
    0b11111,
    0b00100,
    0b00100,
    0b00100,
    0b00100,
    0b00000
};

// Arrow Down (Backward)
byte charArrowDown[8] = {
    0b00100,
    0b00100,
    0b00100,
    0b00100,
    0b11111,
    0b01110,
    0b00100,
    0b00000
};

// Spin Left
byte charSpinLeft[8] = {
    0b00000,
    0b00110,
    0b01000,
    0b11110,
    0b01000,
    0b00110,
    0b00000,
    0b00000
};

// Spin Right
byte charSpinRight[8] = {
    0b00000,
    0b01100,
    0b00010,
    0b01111,
    0b00010,
    0b01100,
    0b00000,
    0b00000
};

// Obstacle Warning
byte charObstacle[8] = {
    0b00100,
    0b00100,
    0b01110,
    0b01110,
    0b11111,
    0b11111,
    0b00100,
    0b00000
};

// Precision Mode
byte charPrecision[8] = {
    0b00000,
    0b01110,
    0b10001,
    0b10101,
    0b10001,
    0b01110,
    0b00000,
    0b00000
};

// Stop Sign
byte charStop[8] = {
    0b00000,
    0b01110,
    0b10001,
    0b10001,
    0b10001,
    0b01110,
    0b00000,
    0b00000
};

// Connected indicator
byte charConnected[8] = {
    0b00000,
    0b01110,
    0b10001,
    0b00100,
    0b00100,
    0b00000,
    0b00100,
    0b00000
};

// ============================================================
//                   FUNCTION PROTOTYPES
// ============================================================

void processReceivedByte(uint8_t byte);
void processMessage();
uint8_t calculateChecksum(uint8_t type, uint8_t len, uint8_t* data);
void sendMotorCommand(uint8_t cmd, uint8_t speed);
void updateLCD();
const char* getModeString(uint8_t mode);
uint8_t getModeIcon(uint8_t mode);
void blinkLED(int times, int delayMs);

// ============================================================
//                        SETUP
// ============================================================

void setup() {
    // Initialize USB Serial for debugging
    Serial.begin(115200);
    delay(100);
    
    Serial.println();
    Serial.println("╔══════════════════════════════════════════╗");
    Serial.println("║   SmartDrive Assist - ESP32 Controller   ║");
    Serial.println("║   PmodCLP + Dagu Motor Driver Interface  ║");
    Serial.println("╚══════════════════════════════════════════╝");
    Serial.println();
    
    // Initialize status LED
    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(STATUS_LED, LOW);
    
    // Initialize STM32 UART (UART2)
    STM32Serial.begin(115200, SERIAL_8N1, STM32_RX, STM32_TX);
    Serial.printf("[UART] STM32 Serial: 115200 baud (RX=%d, TX=%d)\n", STM32_RX, STM32_TX);
    
    // Initialize Motor Driver UART (UART1)
    MotorSerial.begin(19200, SERIAL_8N1, MOTOR_RX, MOTOR_TX);
    Serial.printf("[UART] Motor Serial: 19200 baud (RX=%d, TX=%d)\n", MOTOR_RX, MOTOR_TX);
    
    // Initialize LCD
    Serial.println("[LCD] Initializing PmodCLP...");
    lcd.begin(16, 2);
    lcd.clear();
    
    // Create custom characters
    lcd.createChar(0, charArrowUp);
    lcd.createChar(1, charArrowDown);
    lcd.createChar(2, charSpinLeft);
    lcd.createChar(3, charSpinRight);
    lcd.createChar(4, charObstacle);
    lcd.createChar(5, charPrecision);
    lcd.createChar(6, charStop);
    lcd.createChar(7, charConnected);
    
    // Display startup message
    lcd.setCursor(0, 0);
    lcd.print("SmartDrive v1.0");
    lcd.setCursor(0, 1);
    lcd.print("Waiting STM32...");
    
    Serial.println("[LCD] PmodCLP initialized");
    Serial.println();
    Serial.println("Pin Configuration:");
    Serial.println("  LCD RS  = GPIO13 -> J2.1");
    Serial.println("  LCD R/W = GND    -> J2.2");
    Serial.println("  LCD EN  = GPIO12 -> J2.3");
    Serial.println("  LCD D4  = GPIO14 -> J1.7");
    Serial.println("  LCD D5  = GPIO27 -> J1.8");
    Serial.println("  LCD D6  = GPIO26 -> J1.9");
    Serial.println("  LCD D7  = GPIO25 -> J1.10");
    Serial.println();
    
    // Startup indication
    blinkLED(3, 150);
    
    delay(1500);
    lcd.clear();
    
    lastMessageTime = millis();
    
    Serial.println("[SYSTEM] Ready - Waiting for STM32 data...\n");
}

// ============================================================
//                       MAIN LOOP
// ============================================================

void loop() {
    unsigned long now = millis();
    
    // Process incoming data from STM32
    while (STM32Serial.available()) {
        uint8_t byte = STM32Serial.read();
        processReceivedByte(byte);
        lastMessageTime = now;
    }
    
    // Update LCD periodically
    if (now - lastLcdUpdate >= LCD_UPDATE_INTERVAL) {
        lastLcdUpdate = now;
        updateLCD();
    }
    
    // Heartbeat LED (blink pattern indicates connection status)
    if (now - lastHeartbeat >= HEARTBEAT_INTERVAL) {
        lastHeartbeat = now;
        
        bool connected = (now - lastMessageTime) < COMM_TIMEOUT;
        
        if (connected) {
            // Connected: toggle LED
            digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
        } else {
            // Disconnected: rapid blink
            digitalWrite(STATUS_LED, HIGH);
            delay(50);
            digitalWrite(STATUS_LED, LOW);
        }
    }
    
    // Check for communication timeout
    if (now - lastMessageTime > COMM_TIMEOUT) {
        static unsigned long lastTimeoutMsg = 0;
        if (now - lastTimeoutMsg > 5000) {
            lastTimeoutMsg = now;
            Serial.println("[WARNING] No data from STM32 - Check connections!");
        }
    }
}

// ============================================================
//                   PROTOCOL PROCESSING
// ============================================================

void processReceivedByte(uint8_t byte) {
    switch (rxState) {
        case WAIT_START:
            if (byte == PROTO_START) {
                rxState = WAIT_TYPE;
            }
            break;
            
        case WAIT_TYPE:
            msgType = byte;
            rxState = WAIT_LENGTH;
            break;
            
        case WAIT_LENGTH:
            msgLength = byte;
            if (msgLength > 0 && msgLength <= sizeof(dataBuffer)) {
                dataIndex = 0;
                rxState = WAIT_DATA;
            } else if (msgLength == 0) {
                rxState = WAIT_CHECKSUM;
            } else {
                // Invalid length, reset
                Serial.printf("[ERROR] Invalid length: %d\n", msgLength);
                rxState = WAIT_START;
            }
            break;
            
        case WAIT_DATA:
            dataBuffer[dataIndex++] = byte;
            if (dataIndex >= msgLength) {
                rxState = WAIT_CHECKSUM;
            }
            break;
            
        case WAIT_CHECKSUM:
            {
                uint8_t expected = calculateChecksum(msgType, msgLength, dataBuffer);
                
                if (byte == expected) {
                    processMessage();
                    messagesReceived++;
                } else {
                    checksumErrors++;
                    Serial.printf("[ERROR] Checksum mismatch: got 0x%02X, expected 0x%02X\n", 
                                  byte, expected);
                }
            }
            rxState = WAIT_START;
            break;
    }
}

uint8_t calculateChecksum(uint8_t type, uint8_t len, uint8_t* data) {
    uint8_t checksum = type ^ len;
    for (uint8_t i = 0; i < len; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

void processMessage() {
    switch (msgType) {
        case MSG_SPEED:
            if (msgLength >= 1) {
                currentSpeed = dataBuffer[0];
                int pct = (currentSpeed * 100) / 127;
                Serial.printf("[MSG] Speed: %d (%d%%)\n", currentSpeed, pct);
            }
            break;
            
        case MSG_MODE:
            if (msgLength >= 1) {
                currentMode = dataBuffer[0];
                emergencyStop = (currentMode == MODE_STOP);
                Serial.printf("[MSG] Mode: %s (0x%02X)\n", getModeString(currentMode), currentMode);
            }
            break;
            
        case MSG_DISTANCE:
            if (msgLength >= 1) {
                currentDistance = dataBuffer[0];
                obstacleWarning = (currentDistance > 0 && currentDistance < 50);
                
                if (currentDistance < 20) {
                    Serial.printf("[MSG] Distance: %d cm [!!! DANGER !!!]\n", currentDistance);
                } else if (obstacleWarning) {
                    Serial.printf("[MSG] Distance: %d cm [WARNING]\n", currentDistance);
                } else {
                    Serial.printf("[MSG] Distance: %d cm\n", currentDistance);
                }
            }
            break;
            
        case MSG_MOTOR:
            if (msgLength >= 2) {
                // Decode motor values (bit 7 = sign)
                motorM1 = (dataBuffer[0] & 0x80) ? -(dataBuffer[0] & 0x7F) : (dataBuffer[0] & 0x7F);
                motorM2 = (dataBuffer[1] & 0x80) ? -(dataBuffer[1] & 0x7F) : (dataBuffer[1] & 0x7F);
                Serial.printf("[MSG] Motors: M1=%+4d, M2=%+4d\n", motorM1, motorM2);
            }
            break;
            
        case MSG_STATUS:
            if (msgLength >= 1) {
                precisionMode = (dataBuffer[0] & 0x01) != 0;
                Serial.printf("[MSG] Status: Precision=%s\n", precisionMode ? "ON" : "OFF");
            }
            break;
            
        case MSG_MOTOR_CMD:
            if (msgLength >= 2) {
                uint8_t cmd = dataBuffer[0];
                uint8_t speed = dataBuffer[1];
                sendMotorCommand(cmd, speed);
            }
            break;
            
        default:
            Serial.printf("[MSG] Unknown type: 0x%02X\n", msgType);
            break;
    }
}

// ============================================================
//                    MOTOR CONTROL
// ============================================================

void sendMotorCommand(uint8_t cmd, uint8_t speed) {
    // Send command to Dagu Wild Thumper motor controller
    MotorSerial.write(cmd);
    MotorSerial.write(speed);
    
    // Debug output
    const char* cmdName;
    const char* motor;
    const char* direction;
    
    switch (cmd) {
        case CMD_M1_FORWARD:  motor = "M1"; direction = "FWD"; break;
        case CMD_M1_BACKWARD: motor = "M1"; direction = "BWD"; break;
        case CMD_M1_STOP:     motor = "M1"; direction = "STOP"; break;
        case CMD_M2_FORWARD:  motor = "M2"; direction = "FWD"; break;
        case CMD_M2_BACKWARD: motor = "M2"; direction = "BWD"; break;
        case CMD_M2_STOP:     motor = "M2"; direction = "STOP"; break;
        default:              motor = "??"; direction = "???"; break;
    }
    
    Serial.printf("[MOTOR] %s %s @ %d\n", motor, direction, speed);
}

// ============================================================
//                    LCD DISPLAY
// ============================================================

void updateLCD() {
    bool connected = (millis() - lastMessageTime) < COMM_TIMEOUT;
    
    // ===== LINE 1: Mode and Status =====
    lcd.setCursor(0, 0);
    
    if (!connected) {
        lcd.print("NO CONNECTION!  ");
        lcd.setCursor(0, 1);
        lcd.print("Check STM32 Link");
        return;
    }
    
    if (emergencyStop) {
        lcd.write((uint8_t)6);  // Stop icon
        lcd.print(" STOPPED!      ");
    } else {
        // Mode icon
        switch (currentMode) {
            case MODE_FORWARD:
                lcd.write((uint8_t)0);  // Arrow up
                break;
            case MODE_BACKWARD:
                lcd.write((uint8_t)1);  // Arrow down
                break;
            case MODE_SPIN_LEFT:
                lcd.write((uint8_t)2);  // Spin left
                break;
            case MODE_SPIN_RIGHT:
                lcd.write((uint8_t)3);  // Spin right
                break;
            case MODE_STOP:
            default:
                lcd.write((uint8_t)6);  // Stop
                break;
        }
        
        // Mode name
        lcd.print(" ");
        const char* modeName = getModeString(currentMode);
        lcd.print(modeName);
        
        // Pad to consistent length
        int padLen = 10 - strlen(modeName);
        for (int i = 0; i < padLen; i++) lcd.print(" ");
        
        // Status icons (right side of line 1)
        if (precisionMode) {
            lcd.write((uint8_t)5);  // Precision icon
        } else {
            lcd.print(" ");
        }
        
        if (obstacleWarning) {
            lcd.write((uint8_t)4);  // Obstacle icon
        } else {
            lcd.print(" ");
        }
        
        // Connection indicator
        lcd.write((uint8_t)7);  // Connected icon
    }
    
    // ===== LINE 2: Speed and Distance =====
    lcd.setCursor(0, 1);
    
    int speedPercent = (currentSpeed * 100) / 127;
    char line2[17];
    
    if (obstacleWarning && currentDistance < 20) {
        // Critical warning
        snprintf(line2, sizeof(line2), "D:%2dcm !!STOP!! ", currentDistance);
    } else if (obstacleWarning) {
        // Warning
        snprintf(line2, sizeof(line2), "D:%2dcm Spd:%3d%%", currentDistance, speedPercent);
    } else {
        // Normal display
        snprintf(line2, sizeof(line2), "D:%3dcm Spd:%3d%%", currentDistance, speedPercent);
    }
    
    lcd.print(line2);
}

const char* getModeString(uint8_t mode) {
    switch (mode) {
        case MODE_STOP:       return "Idle";
        case MODE_FORWARD:    return "Forward";
        case MODE_BACKWARD:   return "Reverse";
        case MODE_LEFT:       return "Left";
        case MODE_RIGHT:      return "Right";
        case MODE_SPIN_LEFT:  return "SpinR";
        case MODE_SPIN_RIGHT: return "SpinL";
        case MODE_PRECISION:  return "Precise";
        default:              return "Unknown";
    }
}

// ============================================================
//                    UTILITY FUNCTIONS
// ============================================================

void blinkLED(int times, int delayMs) {
    for (int i = 0; i < times; i++) {
        digitalWrite(STATUS_LED, HIGH);
        delay(delayMs);
        digitalWrite(STATUS_LED, LOW);
        delay(delayMs);
    }
}
