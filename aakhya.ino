
/**************************************************************************
 * SUN TX V1 Transmitter - SOLID CRSF VERSION
 * With proven E28 compatibility
 * Added Safety Check Feature
 **************************************************************************/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "image.h"
#include <EEPROM.h>
#include "crsf.h"

// OLED Display Configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Pin Definitions
const int BATTERY_PIN = 13;
const int BTN_UP     = 22;
const int BTN_DOWN   = 23;  
const int BTN_SELECT = 18;
const int BTN_BACK   = 19;
const int BUZZER_PIN = 14;  // Buzzer for low battery warning

// Joystick pins
const int YAW_PIN      = 35;
const int THROTTLE_PIN = 34;
const int ROLL_PIN     = 27;
const int PITCH_PIN    = 26;

// AUX Channel pins (D2, D4, D5, D21)
const int AUX1_PIN = 2;   // D2
const int AUX2_PIN = 4;   // D4  
const int AUX3_PIN = 5;   // D5
const int AUX4_PIN = 21;  // D21

// Menu system variables
enum AppState { SPLASH, HOME, CONTROL, JOYSTICK, CONFIG, TX_SETTING, CALIBRATE_CENTER, CALIBRATE_RANGE, REVERSE, BINDING, BATTERY_WARNING, DATA_SCREEN, SAFETY_CHECK };
AppState currentState = SPLASH;
int selectedOption = 0;
const char* menuItems[] = {"Joystick", "Config", "Data", "TX Setting"};
const char* txSettingItems[] = {"Calibration", "Reverse", "Bind"};
const int menuItemCount = 4;
const int txSettingItemCount = 3;

// Config presets
const char* presetNames[] = {"25mW-500Hz", "50mW-250Hz", "100mW-150Hz", "500mW-100Hz"};
const int presetCount = 4;
int selectedPreset = 0;

// CRSF Power settings
const char* crsfPowerOptions[] = {"10mW", "25mW", "50mW", "100mW", "250mW", "500mW", "1000mW"};
const int crsfPowerCount = 7;

// CRSF Packet Rate settings
const char* crsfRateOptions[] = {"25Hz", "50Hz", "100Hz", "200Hz", "333Hz", "500Hz"};
const int crsfRateCount = 6;

// Current CRSF settings (will be set by presets)
int selectedCrsfPower = 2; // Default to 50mW
int selectedCrsfRate = 2;  // Default to 100Hz
int selectedCrsfTelem = 0; // Default to Off

// Current settings
String currentPower = "100mW";
String currentFrequency = "500Hz";

// Battery percentage
int batteryLevel = 0;
bool batteryWarningShown = false;
unsigned long lastBatteryBeep = 0;
const unsigned long batteryBeepInterval = 1000; // Beep every 1 second

// Button debouncing variables
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 100;
bool lastUpState = HIGH;
bool lastDownState = HIGH;
bool lastSelectState = HIGH;
bool lastBackState = HIGH;

// Long press detection variables
unsigned long upButtonPressTime = 0;
bool upButtonPressed = false;
const unsigned long longPressDuration = 1000;

// Joystick variables
int yaw_raw, throttle_raw, roll_raw, pitch_raw;
int yaw_x, throttle_y, roll_x, pitch_y;

// Calibration variables
int calCenter[4] = {2048, 2048, 2048, 2048}; // Center values
int calMin[4] = {0, 0, 0, 0};               // Minimum values
int calMax[4] = {4095, 4095, 4095, 4095};    // Maximum values

// Reverse variables
enum ReverseState { REV_MAIN, REV_OPTIONS };
ReverseState reverseState = REV_MAIN;
int selectedReverseChannel = 0;
bool reverseStates[4] = {false, false, false, false};
const char* reverseChannels[] = {"Throttle", "Yaw", "Roll", "Pitch"};
const char* reverseOptions[] = {"Normal", "Reversed"};

// AUX Channel variables
bool aux1State = false;
bool aux2State = false; 
bool aux3State = false;
bool aux4State = false;

// Safety Check variables
bool safetyCheckPassed = false;
unsigned long safetyCheckStartTime = 0;
const unsigned long safetyCheckTimeout = 30000; // 30 seconds timeout
bool safetyWarningBeeped = false;

// CRSF variables
CRSF crsf;
uint8_t crsfPacket[26];
uint8_t crsfCmdPacket[8];
int16_t channels[16];
uint32_t crsfTime = 0;
int loopCount = 0;

// Bind variables
bool bindInProgress = false;
unsigned long bindStartTime = 0;
const unsigned long bindTimeout = 30000;
String bindStatus = "Ready";

// EEPROM addresses
#define REVERSE_ADDR 50
#define EEPROM_SIZE 256  // Increased EEPROM size for safety
#define PRESET_ADDR 0
#define CAL_CENTER_ADDR 10
#define CAL_MIN_ADDR 30
#define CAL_MAX_ADDR 50
#define EEPROM_SIGNATURE 0xAA
#define EEPROM_SIG_ADDR 70

// ADC Constants
#define ADC_MIN 0
#define ADC_MAX 1023
#define ADC_MID 511

// Safety thresholds
#define THROTTLE_SAFETY_THRESHOLD 100  // Throttle must be below this value (0-1023)
void setup() {
  Serial.begin(115200);
  
  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);
  
  // Load saved preset from EEPROM
  selectedPreset = EEPROM.read(PRESET_ADDR);
  if (selectedPreset >= presetCount) selectedPreset = 0;
  updateSettingsFromPreset(selectedPreset);
  
  // Load calibration data from EEPROM
  loadCalibrationData();
  
  // Load reverse states from EEPROM
  loadReverseStates();
  
  // Initialize pins
  pinMode(BATTERY_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW); // Ensure buzzer is off initially
  
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_SELECT, INPUT_PULLUP);
  pinMode(BTN_BACK, INPUT_PULLUP);
  
  // Initialize AUX pins
  pinMode(AUX1_PIN, INPUT_PULLUP);
  pinMode(AUX2_PIN, INPUT_PULLUP);
  pinMode(AUX3_PIN, INPUT_PULLUP);
  pinMode(AUX4_PIN, INPUT_PULLUP);
  
  // Initialize I2C with custom pins
  Wire.begin(32, 33);
  
  // Initialize CRSF
  crsf.begin();
  
  // Add debug serial
  //Serial2.begin(416666, SERIAL_8N1, 16, 17); // RX=16, TX=17
  //Serial.println("CRSF Serial initialized on GPIO 16/17");
  
  // Initialize OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  
  // Show splash screen
  display.clearDisplay();
  display.drawBitmap(0, 0, myBitmap, 128, 64, SSD1306_WHITE);
  display.display();
  delay(2000);
  
  // Start safety check after splash screen
  currentState = SAFETY_CHECK;
  safetyCheckStartTime = millis();
  safetyCheckPassed = false;
  safetyWarningBeeped = false;
  
  Serial.println("SUN TX V1 STARTED - Safety Check Active");
  
  // Print initial calibration status
  Serial.println("Initial Calibration Status:");
  Serial.print("Centers: ");
  for(int i = 0; i < 4; i++) {
    Serial.print(calCenter[i]);
    Serial.print(" ");
  }
  Serial.println();
}

void loadCalibrationData() {
  // Check if EEPROM has valid calibration data
  uint8_t signature = EEPROM.read(EEPROM_SIG_ADDR);
  
  if (signature == EEPROM_SIGNATURE) {
    // Load center values
    for(int i = 0; i < 4; i++) {
      calCenter[i] = EEPROM.readInt(CAL_CENTER_ADDR + i * sizeof(int));
      // Validate loaded values
      if(calCenter[i] < 500 || calCenter[i] > 3500) {
        calCenter[i] = 2048;
        Serial.println("Invalid center value, using default");
      }
    }
    
    // Load min values
    for(int i = 0; i < 4; i++) {
      calMin[i] = EEPROM.readInt(CAL_MIN_ADDR + i * sizeof(int));
      // Validate loaded values
      if(calMin[i] < 0 || calMin[i] > 2000) {
        calMin[i] = 0;
        Serial.println("Invalid min value, using default");
      }
    }
    
    // Load max values
    for(int i = 0; i < 4; i++) {
      calMax[i] = EEPROM.readInt(CAL_MAX_ADDR + i * sizeof(int));
      // Validate loaded values
      if(calMax[i] < 2500 || calMax[i] > 4095) {
        calMax[i] = 4095;
        Serial.println("Invalid max value, using default");
      }
    }
    Serial.println("Calibration data loaded from EEPROM");
  } else {
    // First time use, set defaults and mark EEPROM as initialized
    resetCalibrationToDefaults();
    EEPROM.write(EEPROM_SIG_ADDR, EEPROM_SIGNATURE);
    EEPROM.commit();
    Serial.println("First boot: Default calibration set");
  }
}

void loadReverseStates() {
  for(int i = 0; i < 4; i++) {
    reverseStates[i] = EEPROM.read(REVERSE_ADDR + i);
  }
}

void saveCalibrationData() {
  // Verify values are reasonable before saving
  for(int i = 0; i < 4; i++) {
    calMin[i] = constrain(calMin[i], 0, 4095);
    calMax[i] = constrain(calMax[i], 0, 4095);
    calCenter[i] = constrain(calCenter[i], 0, 4095);
    
    // Ensure min < center < max
    if (calMin[i] >= calCenter[i]) calMin[i] = calCenter[i] - 100;
    if (calMax[i] <= calCenter[i]) calMax[i] = calCenter[i] + 100;
  }
  
  // Save all calibration data to EEPROM
  for(int i = 0; i < 4; i++) {
    EEPROM.writeInt(CAL_CENTER_ADDR + i * sizeof(int), calCenter[i]);
  }
  for(int i = 0; i < 4; i++) {
    EEPROM.writeInt(CAL_MIN_ADDR + i * sizeof(int), calMin[i]);
  }
  for(int i = 0; i < 4; i++) {
    EEPROM.writeInt(CAL_MAX_ADDR + i * sizeof(int), calMax[i]);
  }
  
  // Mark EEPROM as initialized
  EEPROM.write(EEPROM_SIG_ADDR, EEPROM_SIGNATURE);
  
  EEPROM.commit();
  Serial.println("All calibration data saved to EEPROM");
  
  // Print calibration values for verification
  Serial.println("Calibration Values Saved:");
  Serial.print("Centers: ");
  for(int i = 0; i < 4; i++) {
    Serial.print(calCenter[i]);
    Serial.print(" ");
  }
  Serial.println();
  Serial.print("Mins: ");
  for(int i = 0; i < 4; i++) {
    Serial.print(calMin[i]);
    Serial.print(" ");
  }
  Serial.println();
  Serial.print("Maxs: ");
  for(int i = 0; i < 4; i++) {
    Serial.print(calMax[i]);
    Serial.print(" ");
  }
  Serial.println();
}

void resetCalibrationToDefaults() {
  Serial.println("Resetting calibration to defaults");
  
  // Set default calibration values
  for(int i = 0; i < 4; i++) {
    calCenter[i] = 2048;
    calMin[i] = 500;
    calMax[i] = 3500;
  }
  
  // Save to EEPROM
  saveCalibrationData();
  Serial.println("Default calibration saved");
}

// NEW: Safety Check Function
bool performSafetyCheck() {
  // Read current throttle position
  int throttle_raw_check = analogRead(THROTTLE_PIN);
  int throttle_calibrated = map(throttle_raw_check, calMin[1], calMax[1], ADC_MIN, ADC_MAX);
  if (reverseStates[0]) throttle_calibrated = ADC_MAX - throttle_calibrated;
  throttle_calibrated = constrain(throttle_calibrated, ADC_MIN, ADC_MAX);
  
  // Read AUX switches
  bool aux1_check = !digitalRead(AUX1_PIN);
  bool aux2_check = !digitalRead(AUX2_PIN);
  bool aux3_check = !digitalRead(AUX3_PIN);
  bool aux4_check = !digitalRead(AUX4_PIN);
  
  // Check conditions
  bool throttleSafe = (throttle_calibrated <= THROTTLE_SAFETY_THRESHOLD);
  bool auxSafe = (!aux1_check && !aux2_check && !aux3_check && !aux4_check);
  
  Serial.print("Safety Check - Throttle: ");
  Serial.print(throttle_calibrated);
  Serial.print(" AUX: ");
  Serial.print(aux1_check);
  Serial.print(aux2_check);
  Serial.print(aux3_check);
  Serial.print(aux4_check);
  Serial.print(" | Safe: ");
  Serial.println(throttleSafe && auxSafe ? "YES" : "NO");
  
  return (throttleSafe && auxSafe);
}

// NEW: Draw Safety Check Screen
void drawSafetyCheckScreen() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  
  // Title
  display.setCursor(22, 2);
  display.print("SAFETY CHECK");
  display.drawLine(0, 12, 128, 12, SSD1306_WHITE);
  
  // Warning symbol
  
  
  // Messages
  display.setCursor(2, 20);
  display.print("Throttle not at zero");
  display.setCursor(4, 30);
  display.print("or AUX switches ON");
  
  display.setCursor(10, 50);
  display.print("Fix to continue...");
  
  // Countdown timer
  unsigned long elapsed = (millis() - safetyCheckStartTime) / 1000;
  unsigned long remaining = (safetyCheckTimeout / 1000) - elapsed;
  if (remaining < 0) remaining = 0;
  
  display.setCursor(100, 2);
  display.print(remaining);
  display.print("s");
  
  display.display();
  
  // Beep warning every 2 seconds
  if (!safetyWarningBeeped || (millis() - lastBatteryBeep) > 2000) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(200);
    digitalWrite(BUZZER_PIN, LOW);
    safetyWarningBeeped = true;
    lastBatteryBeep = millis();
  }
}
void sendCrsfCommands() {
  if (loopCount <= 500) {
    // Stage 1: Build connection with channel data
    crsf.crsfPrepareDataPacket(crsfPacket, channels);
    crsf.CrsfWritePacket(crsfPacket, CRSF_PACKET_SIZE);
    loopCount++;
  }
  else if (loopCount > 500 && loopCount <= 505) {
    // Stage 2: Send packet rate commands
    crsf.crsfPrepareCmdPacket(crsfCmdPacket, ELRS_PKT_RATE_COMMAND, selectedCrsfRate);
    crsf.CrsfWritePacket(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
    loopCount++;
  }
  else if (loopCount > 505 && loopCount <= 510) {
    // Stage 3: Send power commands
    crsf.crsfPrepareCmdPacket(crsfCmdPacket, ELRS_POWER_COMMAND, selectedCrsfPower);
    crsf.CrsfWritePacket(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
    loopCount++;
  }
  else if (loopCount > 510 && loopCount <= 515) {
    // Stage 4: Send telemetry commands
    crsf.crsfPrepareCmdPacket(crsfCmdPacket, ELRS_TLM_RATIO_COMMAND, selectedCrsfTelem);
    crsf.CrsfWritePacket(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
    loopCount++;
  }
  else {
    // Stage 5: Continuous channel data
    crsf.crsfPrepareDataPacket(crsfPacket, channels);
    crsf.CrsfWritePacket(crsfPacket, CRSF_PACKET_SIZE);
  }
}

void sendCrsfBindCommand() {
  crsf.crsfPrepareCmdPacket(crsfCmdPacket, ELRS_BIND_COMMAND, 1);
  crsf.CrsfWritePacket(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
  bindInProgress = true;
  bindStartTime = millis();
  bindStatus = "Binding...";
  loopCount = 0;
  Serial.println("Bind command sent");
}

void checkBindStatus() {
  if (bindInProgress) {
    unsigned long currentTime = millis();
    if (currentTime - bindStartTime > bindTimeout) {
      bindInProgress = false;
      bindStatus = "Bind Timeout";
    } else if (currentTime - bindStartTime > 5000) {
      bindInProgress = false;
      bindStatus = "Bind Done!";
    }
  }
}

void updateSettingsFromPreset(int preset) {
  switch(preset) {
    case 0: 
      currentPower = "25mW"; currentFrequency = "500Hz"; 
      selectedCrsfPower = 1; selectedCrsfRate = 5; break;
    case 1: 
      currentPower = "50mW"; currentFrequency = "250Hz"; 
      selectedCrsfPower = 2; selectedCrsfRate = 3; break;
    case 2: 
      currentPower = "100mW"; currentFrequency = "150Hz"; 
      selectedCrsfPower = 3; selectedCrsfRate = 2; break;
    case 3: 
      currentPower = "500mW"; currentFrequency = "100Hz"; 
      selectedCrsfPower = 5; selectedCrsfRate = 2; break;
  }
  loopCount = 0;
}

void readBatteryLevel() {
  int rawValue = analogRead(BATTERY_PIN);
  float voltage = (rawValue * 3.3 / 4095.0);
  if (voltage >= 3.3) batteryLevel = 100;
  else if (voltage <= 2.3) batteryLevel = 0;
  else batteryLevel = map(voltage * 100, 230, 330, 0, 100);
  
  // Check for low battery
  if (batteryLevel <= 20 && batteryLevel > 0) {
    if (!batteryWarningShown) {
      batteryWarningShown = true;
      currentState = BATTERY_WARNING;
    }
  } else {
    batteryWarningShown = false;
  }
}

void handleBatteryWarning() {
  unsigned long currentTime = millis();
  
  // Beep every second
  if (currentTime - lastBatteryBeep >= batteryBeepInterval) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
    lastBatteryBeep = currentTime;
  }
  
  // Check if battery recovered
  if (batteryLevel > 20) {
    batteryWarningShown = false;
    currentState = HOME;
  }
}

void updateJoystickData() {
  yaw_raw = analogRead(YAW_PIN);
  throttle_raw = analogRead(THROTTLE_PIN);
  roll_raw = analogRead(ROLL_PIN);
  pitch_raw = analogRead(PITCH_PIN);

  // Apply calibration - FIXED MAPPING
  int yaw_calibrated, throttle_calibrated, roll_calibrated, pitch_calibrated;
  
  // Map each channel using the full calibrated range to full ADC range
  yaw_calibrated = map(yaw_raw, calMin[0], calMax[0], ADC_MIN, ADC_MAX);
  throttle_calibrated = map(throttle_raw, calMin[1], calMax[1], ADC_MIN, ADC_MAX);
  roll_calibrated = map(roll_raw, calMin[2], calMax[2], ADC_MIN, ADC_MAX);
  pitch_calibrated = map(pitch_raw, calMin[3], calMax[3], ADC_MIN, ADC_MAX);

  // Apply reverse
  if (reverseStates[0]) throttle_calibrated = ADC_MAX - throttle_calibrated;
  if (reverseStates[1]) yaw_calibrated = ADC_MAX - yaw_calibrated;
  if (reverseStates[2]) roll_calibrated = ADC_MAX - roll_calibrated;
  if (reverseStates[3]) pitch_calibrated = ADC_MAX - pitch_calibrated;

  // Constrain values to ensure they stay within bounds
  yaw_calibrated = constrain(yaw_calibrated, ADC_MIN, ADC_MAX);
  throttle_calibrated = constrain(throttle_calibrated, ADC_MIN, ADC_MAX);
  roll_calibrated = constrain(roll_calibrated, ADC_MIN, ADC_MAX);
  pitch_calibrated = constrain(pitch_calibrated, ADC_MIN, ADC_MAX);

  // Map to CRSF range
  channels[0] = map(throttle_calibrated, ADC_MIN, ADC_MAX, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX);
  channels[1] = map(yaw_calibrated, ADC_MIN, ADC_MAX, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX);
  channels[2] = map(roll_calibrated, ADC_MIN, ADC_MAX, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX);
  channels[3] = map(pitch_calibrated, ADC_MIN, ADC_MAX, CRSF_DIGITAL_CHANNEL_MIN, CRSF_DIGITAL_CHANNEL_MAX);
  
  // Read AUX channels (inverted because of INPUT_PULLUP)
  aux1State = !digitalRead(AUX1_PIN);
  aux2State = !digitalRead(AUX2_PIN);
  aux3State = !digitalRead(AUX3_PIN);
  aux4State = !digitalRead(AUX4_PIN);
  
  // Set AUX channels in CRSF data
  channels[4] = aux1State ? CRSF_DIGITAL_CHANNEL_MAX : CRSF_DIGITAL_CHANNEL_MIN;  // AUX1
  channels[5] = aux2State ? CRSF_DIGITAL_CHANNEL_MAX : CRSF_DIGITAL_CHANNEL_MIN;  // AUX2
  channels[6] = aux3State ? CRSF_DIGITAL_CHANNEL_MAX : CRSF_DIGITAL_CHANNEL_MIN;  // AUX3
  channels[7] = aux4State ? CRSF_DIGITAL_CHANNEL_MAX : CRSF_DIGITAL_CHANNEL_MIN;  // AUX4
  
  // Set remaining channels to center
  for(int i = 8; i < 16; i++) {
    channels[i] = CRSF_DIGITAL_CHANNEL_MID;
  }

  // Update display coordinates - FIXED MAPPING
  yaw_x = map(yaw_calibrated, ADC_MIN, ADC_MAX, 12, 52);
  throttle_y = map(throttle_calibrated, ADC_MIN, ADC_MAX, 52, 12);
  roll_x = map(roll_calibrated, ADC_MIN, ADC_MAX, 76, 116);
  pitch_y = map(pitch_calibrated, ADC_MIN, ADC_MAX, 52, 12);

  // Debug output to verify calibration is working
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 2000) {
    Serial.println("Calibration Debug:");
    Serial.print("Yaw: raw="); Serial.print(yaw_raw); Serial.print(" cal="); Serial.print(yaw_calibrated); Serial.print(" x="); Serial.println(yaw_x);
    Serial.print("Thr: raw="); Serial.print(throttle_raw); Serial.print(" cal="); Serial.print(throttle_calibrated); Serial.print(" y="); Serial.println(throttle_y);
    Serial.print("Roll: raw="); Serial.print(roll_raw); Serial.print(" cal="); Serial.print(roll_calibrated); Serial.print(" x="); Serial.println(roll_x);
    Serial.print("Pitch: raw="); Serial.print(pitch_raw); Serial.print(" cal="); Serial.print(pitch_calibrated); Serial.print(" y="); Serial.println(pitch_y);
    lastDebug = millis();
  }
}

void sendCrsfData() {
  uint32_t currentMicros = micros();
  
  if (currentMicros > crsfTime) {
    sendCrsfCommands();
    crsfTime = currentMicros + CRSF_TIME_BETWEEN_FRAMES_US;
  }
}
void drawHomeScreen() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(2, 3);
  display.print("SUN TX V1");
  drawHomeBattery();
  display.setCursor(5, 25);
  display.print("FREQ-");
  display.setCursor(5, 35);
  display.print(currentFrequency);
  display.setCursor(85, 25);
  display.print("POWER-");
  display.setCursor(85, 35);
  display.print(currentPower);
  
  // Show AUX status
  display.setCursor(5, 45);
  display.print("AUX:");
  display.print(aux1State ? "1" : "0");
  display.print(aux2State ? "1" : "0"); 
  display.print(aux3State ? "1" : "0");
  display.print(aux4State ? "1" : "0");
  
  display.setCursor(5, 55);
  display.print("[HOLD UP TO CONFIG]");
  display.display();
}

void drawHomeBattery() {
  int batteryX = 95, batteryY = 2, batteryWidth = 30, batteryHeight = 12;
  display.drawRect(batteryX, batteryY, batteryWidth, batteryHeight, SSD1306_WHITE);
  display.fillRect(batteryX + batteryWidth, batteryY + 3, 2, 6, SSD1306_WHITE);
  int fillWidth = map(batteryLevel, 0, 100, 0, batteryWidth - 2);
  if (fillWidth > 0) {
    display.fillRect(batteryX + 1, batteryY + 1, fillWidth, batteryHeight - 2, SSD1306_WHITE);
  }
  display.setCursor(70, 3);
  display.print(batteryLevel);
  display.print("%");
}

void drawBatteryWarningScreen() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  
  // Warning symbol
 
  
  display.setTextSize(1);
  display.setCursor(25, 20);
  display.print("LOW BATTERY");
  display.setCursor(20, 30);
  display.print("Level: ");
  display.print(batteryLevel);
  display.print("%");
  display.setCursor(15, 45);
  display.print("CHARGE NOW");
  
  display.display();
}

void drawControlScreen() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(30, 2);
  display.print("CONTROL");
  display.drawLine(0, 18, 128, 18, SSD1306_WHITE);
  display.setTextSize(1);
  for (int i = 0; i < menuItemCount; i++) {
    int yPos = 23 + i * 10;
    if (i == selectedOption) {
      display.fillRect(5, yPos - 1, 118, 9, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(2, yPos);
      display.print(">");
    } else {
      display.setTextColor(SSD1306_WHITE);
    }
    display.setCursor(10, yPos);
    display.print(menuItems[i]);
    display.setTextColor(SSD1306_WHITE);
  }
  display.display();
}

void drawConfigScreen() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(40, 2);
  display.print("CONFIG");
  display.drawLine(0, 18, 128, 18, SSD1306_WHITE);
  display.setTextSize(1);
  for (int i = 0; i < presetCount; i++) {
    int yPos = 25 + i * 10;
    if (i == selectedPreset) {
      display.fillRect(5, yPos - 1, 118, 9, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(2, yPos);
      display.print(">");
    } else {
      display.setTextColor(SSD1306_WHITE);
    }
    display.setCursor(10, yPos);
    display.print(presetNames[i]);
    display.setTextColor(SSD1306_WHITE);
  }
  display.display();
}

void drawJoystickScreen() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(40, 0);
  display.print("JOYSTICK");
  drawVerticalBattery();
  
  display.drawLine(12, 32, 52, 32, SSD1306_WHITE);
  display.drawLine(32, 12, 32, 52, SSD1306_WHITE);
  display.setCursor(2, 8);
  display.print("THR");
  display.setCursor(2, 58);
  display.print("YAW");
  display.fillCircle(yaw_x, throttle_y, 3, SSD1306_WHITE);
  display.drawLine(76, 32, 116, 32, SSD1306_WHITE);
  display.drawLine(96, 12, 96, 52, SSD1306_WHITE);
  display.setCursor(95, 8);
  display.print("PITCH");
  display.setCursor(102, 58);
  display.print("ROLL");
  display.fillCircle(roll_x, pitch_y, 3, SSD1306_WHITE);
  display.display();
}

void drawVerticalBattery() {
  int batteryX = 60, batteryY = 25, batteryWidth = 8, batteryHeight = 20;
  display.drawRect(batteryX, batteryY, batteryWidth, batteryHeight, SSD1306_WHITE);
  display.fillRect(batteryX + batteryWidth/2 - 2, batteryY - 2, 4, 2, SSD1306_WHITE);
  int fillHeight = map(batteryLevel, 0, 100, 0, batteryHeight - 2);
  if (fillHeight > 0) {
    display.fillRect(batteryX + 1, batteryY + batteryHeight - 1 - fillHeight, batteryWidth - 2, fillHeight, SSD1306_WHITE);
  }
}

void drawDataScreen() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  
  // Title
  display.setCursor(45, 0);
  display.print("DATA");
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);
  
  // Raw values
  display.setCursor(5, 15);
  display.print("Raw:");
  display.setCursor(5, 25);
  display.print("Y:"); display.print(yaw_raw);
  display.setCursor(45, 25);
  display.print("T:"); display.print(throttle_raw);
  display.setCursor(5, 35);
  display.print("R:"); display.print(roll_raw);
  display.setCursor(45, 35);
  display.print("P:"); display.print(pitch_raw);
  
  // Calibrated values
  display.setCursor(85, 15);
  display.print("Cal:");
  display.setCursor(85, 25);
  display.print("Y:"); display.print(map(yaw_raw, calMin[0], calMax[0], ADC_MIN, ADC_MAX));
  display.setCursor(85, 35);
  display.print("T:"); display.print(map(throttle_raw, calMin[1], calMax[1], ADC_MIN, ADC_MAX));
  display.setCursor(85, 45);
  display.print("R:"); display.print(map(roll_raw, calMin[2], calMax[2], ADC_MIN, ADC_MAX));
  display.setCursor(85, 55);
  display.print("P:"); display.print(map(pitch_raw, calMin[3], calMax[3], ADC_MIN, ADC_MAX));
  
  // Calibration status
  display.setCursor(5, 45);
  display.print("Calib: ");
  display.print(EEPROM.read(EEPROM_SIG_ADDR) == EEPROM_SIGNATURE ? "OK" : "Need");
  
  // Battery
  display.setCursor(5, 55);
  display.print("Batt: ");
  display.print(batteryLevel);
  display.print("%");
  
  display.display();
}

void drawReverseScreen() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  if (reverseState == REV_MAIN) {
    display.setTextSize(1);
    display.setCursor(45, 2);
    display.print("REVERSE");
    display.drawLine(0, 12, 128, 12, SSD1306_WHITE);
    display.setTextSize(1);
    for (int i = 0; i < 4; i++) {
      int yPos = 18 + i * 10;
      if (i == selectedReverseChannel) {
        display.fillRect(5, yPos - 1, 118, 9, SSD1306_WHITE);
        display.setTextColor(SSD1306_BLACK);
        display.setCursor(2, yPos);
        display.print(">");
      } else {
        display.setTextColor(SSD1306_WHITE);
      }
      display.setCursor(10, yPos);
      display.print(reverseChannels[i]);
      display.setCursor(70, yPos);
      display.print(reverseStates[i] ? "Reversed" : "Normal");
      display.setTextColor(SSD1306_WHITE);
    }
  } else if (reverseState == REV_OPTIONS) {
    display.setTextSize(1);
    display.setCursor(35, 2);
    display.print(reverseChannels[selectedReverseChannel]);
    display.drawLine(0, 12, 128, 12, SSD1306_WHITE);
    display.setTextSize(1);
    for (int i = 0; i < 2; i++) {
      int yPos = 25 + i * 15;
      if (i == (reverseStates[selectedReverseChannel] ? 1 : 0)) {
        display.fillRect(30, yPos - 1, 68, 9, SSD1306_WHITE);
        display.setTextColor(SSD1306_BLACK);
      } else {
        display.setTextColor(SSD1306_WHITE);
      }
      display.setCursor(35, yPos);
      display.print(reverseOptions[i]);
      display.setTextColor(SSD1306_WHITE);
    }
  }
  display.display();
}

void drawTXSettingScreen() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(40, 2);
  display.print("TX SETTING");
  display.drawLine(0, 12, 128, 12, SSD1306_WHITE);
  display.setTextSize(1);
  for (int i = 0; i < txSettingItemCount; i++) {
    int yPos = 20 + i * 10;
    if (i == selectedOption) {
      display.fillRect(5, yPos - 1, 118, 9, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(2, yPos);
      display.print(">");
    } else {
      display.setTextColor(SSD1306_WHITE);
    }
    display.setCursor(10, yPos);
    display.print(txSettingItems[i]);
    if (i == 2) {
      display.setCursor(70, yPos);
      display.print(bindStatus);
    }
    display.setTextColor(SSD1306_WHITE);
  }
  display.display();
}

void drawBindingScreen() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(40, 5);
  display.print("BINDING");
  display.setCursor(15, 20);
  display.print("Status: ");
  display.print(bindStatus);
  
  // Smaller messages
  display.setCursor(10, 35);
  display.print("Put RX in bind mode");
  display.setCursor(25, 45);
  display.print("BACK to cancel");
  
  if (bindInProgress) {
    int elapsed = (millis() - bindStartTime) / 1000;
    display.setCursor(50, 55);
    display.print(elapsed);
    display.print("s");
  }
  display.display();
}

void drawCalibrateCenterScreen() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(25, 5);
  display.print("CALIBRATION");
  display.setCursor(10, 20);
  display.print("Center Sticks");
  display.setCursor(15, 30);
  display.print("Keep all sticks");
  display.setCursor(25, 40);
  display.print("in center");
  display.setCursor(30, 55);
  display.print("Press SELECT");
  display.display();
  
  static unsigned long startTime = millis();
  if (millis() - startTime > 2000) {
    calCenter[0] = analogRead(YAW_PIN);
    calCenter[1] = analogRead(THROTTLE_PIN);
    calCenter[2] = analogRead(ROLL_PIN);
    calCenter[3] = analogRead(PITCH_PIN);
    
    // ✅ SAVE CENTER VALUES TO EEPROM IMMEDIATELY
    for(int i = 0; i < 4; i++) {
      EEPROM.writeInt(CAL_CENTER_ADDR + i * sizeof(int), calCenter[i]);
    }
    EEPROM.commit();
    Serial.println("Center calibration saved to EEPROM");
  }
}

void drawCalibrateRangeScreen() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(25, 5);
  display.print("CALIBRATION");
  display.setCursor(10, 20);
  display.print("Move All Sticks");
  display.setCursor(15, 30);
  display.print("Through full");
  display.setCursor(25, 40);
  display.print("range");
  display.setCursor(30, 55);
  display.print("Press SELECT");
  display.display();

  // Update min/max values in real-time
  int currentValues[4] = {
    analogRead(YAW_PIN),
    analogRead(THROTTLE_PIN),
    analogRead(ROLL_PIN),
    analogRead(PITCH_PIN)
  };

  bool valuesChanged = false;
  for(int i = 0; i < 4; i++) {
    if(currentValues[i] < calMin[i]) {
      calMin[i] = currentValues[i];
      valuesChanged = true;
    }
    if(currentValues[i] > calMax[i]) {
      calMax[i] = currentValues[i];
      valuesChanged = true;
    }
  }

  // ✅ SAVE TO EEPROM WHENEVER VALUES CHANGE
  if (valuesChanged) {
    for(int i = 0; i < 4; i++) {
      EEPROM.writeInt(CAL_MIN_ADDR + i * sizeof(int), calMin[i]);
      EEPROM.writeInt(CAL_MAX_ADDR + i * sizeof(int), calMax[i]);
    }
    EEPROM.commit();
    Serial.println("Range calibration updated in EEPROM");
  }
}

void debugCRSF() {
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 2000) {
    Serial.print("CRSF Status - LoopCount: ");
    Serial.print(loopCount);
    Serial.print(" | Ch1: ");
    Serial.print(channels[0]);
    Serial.print(" | Ch2: ");
    Serial.print(channels[1]);
    Serial.print(" | Ch3: ");
    Serial.print(channels[2]);
    Serial.print(" | Ch4: ");
    Serial.println(channels[3]);
    
    // Also print calibration status
    Serial.print("Cal Centers: ");
    for(int i = 0; i < 4; i++) {
      Serial.print(calCenter[i]);
      Serial.print(" ");
    }
    Serial.println();
    
    lastDebug = millis();
  }
}

void handleNavigation() {
  unsigned long currentTime = millis();
  bool upPressed = !digitalRead(BTN_UP);
  bool downPressed = !digitalRead(BTN_DOWN);
  bool selectPressed = !digitalRead(BTN_SELECT);
  bool backPressed = !digitalRead(BTN_BACK);

  if (currentTime - lastDebounceTime < debounceDelay) return;

  if (upPressed && !lastUpState) {
    if (currentState == HOME) {
      upButtonPressTime = currentTime;
      upButtonPressed = true;
    } else if (currentState == CONTROL) {
      selectedOption = (selectedOption - 1 + menuItemCount) % menuItemCount;
      lastDebounceTime = currentTime;
    } else if (currentState == CONFIG) {
      selectedPreset = (selectedPreset - 1 + presetCount) % presetCount;
      lastDebounceTime = currentTime;
    } else if (currentState == TX_SETTING) {
      selectedOption = (selectedOption - 1 + txSettingItemCount) % txSettingItemCount;
      lastDebounceTime = currentTime;
    } else if (currentState == REVERSE) {
      if (reverseState == REV_MAIN) {
        selectedReverseChannel = (selectedReverseChannel - 1 + 4) % 4;
        lastDebounceTime = currentTime;
      }
    }
  }
  
  if (!upPressed && upButtonPressed) {
    if (currentTime - upButtonPressTime >= longPressDuration && currentState == HOME) {
      currentState = CONTROL;
      selectedOption = 0;
    }
    upButtonPressed = false;
  }

  if (downPressed && !lastDownState) {
    if (currentState == CONTROL) {
      selectedOption = (selectedOption + 1) % menuItemCount;
    } else if (currentState == CONFIG) {
      selectedPreset = (selectedPreset + 1) % presetCount;
    } else if (currentState == TX_SETTING) {
      selectedOption = (selectedOption + 1) % txSettingItemCount;
    } else if (currentState == REVERSE) {
      if (reverseState == REV_MAIN) {
        selectedReverseChannel = (selectedReverseChannel + 1) % 4;
        lastDebounceTime = currentTime;
      }
    }
    lastDebounceTime = currentTime;
  }

  if (selectPressed && !lastSelectState) {
    if (currentState == CONTROL) {
      switch(selectedOption) {
        case 0: currentState = JOYSTICK; break;
        case 1: currentState = CONFIG; selectedPreset = 0; break;
        case 2: currentState = DATA_SCREEN; break;
        case 3: currentState = TX_SETTING; selectedOption = 0; break;
      }
    } else if (currentState == CONFIG) {
      EEPROM.write(PRESET_ADDR, selectedPreset);
      EEPROM.commit();
      updateSettingsFromPreset(selectedPreset);
      currentState = HOME;
    } else if (currentState == TX_SETTING) {
      if (selectedOption == 0) {
        currentState = CALIBRATE_CENTER;
      } else if (selectedOption == 1) {
        currentState = REVERSE;
        reverseState = REV_MAIN;
        selectedReverseChannel = 0;
      } else if (selectedOption == 2) {
        sendCrsfBindCommand();
        currentState = BINDING;
      }
    } else if (currentState == CALIBRATE_CENTER) {
      currentState = CALIBRATE_RANGE;
      for(int i = 0; i < 4; i++) {
        calMin[i] = 4095;
        calMax[i] = 0;
      }
    } else if (currentState == CALIBRATE_RANGE) {
      saveCalibrationData();
      currentState = TX_SETTING;
    } else if (currentState == REVERSE) {
      if (reverseState == REV_MAIN) {
        reverseState = REV_OPTIONS;
      } else {
        reverseStates[selectedReverseChannel] = !reverseStates[selectedReverseChannel];
        EEPROM.write(REVERSE_ADDR + selectedReverseChannel, reverseStates[selectedReverseChannel]);
        EEPROM.commit();
        reverseState = REV_MAIN;
      }
      lastDebounceTime = currentTime;
    } else if (currentState == BINDING) {
      if (!bindInProgress) {
        currentState = TX_SETTING;
      }
    }
    lastDebounceTime = currentTime;
  }

  if (backPressed && !lastBackState) {
    if (currentState == REVERSE) {
      if (reverseState == REV_OPTIONS) {
        reverseState = REV_MAIN;
      } else {
        currentState = TX_SETTING;
      }
    } else if (currentState == BINDING) {
      bindInProgress = false;
      bindStatus = "Cancelled";
      currentState = TX_SETTING;
    } else if (currentState == BATTERY_WARNING) {
      currentState = HOME;
    } else if (currentState == DATA_SCREEN) {
      currentState = CONTROL;
    } else if (currentState == CONTROL || currentState == JOYSTICK || currentState == CONFIG || 
        currentState == TX_SETTING || currentState == CALIBRATE_CENTER || currentState == CALIBRATE_RANGE) {
      currentState = HOME;
    }
    lastDebounceTime = currentTime;
  }

  lastUpState = upPressed;
  lastDownState = downPressed;
  lastSelectState = selectPressed;
  lastBackState = backPressed;
}

void loop() {
  // Always read joysticks and prepare channel data
  updateJoystickData();
  
  // Handle safety check state
  if (currentState == SAFETY_CHECK) {
    // Check timeout
    if (millis() - safetyCheckStartTime > safetyCheckTimeout) {
      // Timeout reached, force continue (safety override)
      safetyCheckPassed = true;
      currentState = HOME;
      Serial.println("Safety check timeout - proceeding anyway");
    }
    
    // Check if safety conditions are met
    if (performSafetyCheck()) {
      safetyCheckPassed = true;
      currentState = HOME;
      Serial.println("Safety check PASSED - proceeding to home screen");
      // Beep once to indicate success
      digitalWrite(BUZZER_PIN, HIGH);
      delay(100);
      digitalWrite(BUZZER_PIN, LOW);
    }
  } else {
    // Only send CRSF data after safety check passed
    if (safetyCheckPassed) {
      sendCrsfData();
    }
  }

  debugCRSF();
  readBatteryLevel();
  
  // Only handle navigation if safety check passed or we're in safety check screen
  if (safetyCheckPassed || currentState == SAFETY_CHECK) {
    handleNavigation();
  }

  if (bindInProgress) {
    checkBindStatus();
  }

  switch(currentState) {
    case HOME:
      drawHomeScreen();
      break;
    case CONTROL:
      drawControlScreen();
      break;
    case JOYSTICK:
      drawJoystickScreen();
      break;
    case CONFIG:
      drawConfigScreen();
      break;
    case TX_SETTING:
      drawTXSettingScreen();
      break;
    case BINDING:
      drawBindingScreen();
      break;
    case CALIBRATE_CENTER:
      drawCalibrateCenterScreen();
      break;
    case CALIBRATE_RANGE:
      drawCalibrateRangeScreen();
      break;
    case REVERSE:
      drawReverseScreen();
      break;
    case BATTERY_WARNING:
      drawBatteryWarningScreen();
      handleBatteryWarning();
      break;
    case DATA_SCREEN:
      drawDataScreen();
      break;
    case SAFETY_CHECK:
      drawSafetyCheckScreen();
      break;
    case SPLASH:
      break;
  }
}
