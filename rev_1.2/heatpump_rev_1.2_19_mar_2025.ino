#include <Wire.h>
#include <RTClib.h>
#include <DHT.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>

// Define pins and constants
#define DHTPIN 7
#define DHTTYPE DHT22
#define BUZZER_PIN A0
#define RELAY_HUMIDIFIER_1 5
#define RELAY_HUMIDIFIER_2 15
#define RELAY_SSR 4       // Controls the heater (binary on/off)
#define RELAY_FAN 10      // Not used directly (fan controlled via PWM)
#define FAN_PWM_PIN 6
#define EEPROM_START_ADDR 0
#define MIN_TEMP 0
#define MAX_TEMP 100
#define MIN_HUMIDITY 0
#define MAX_HUMIDITY 100

// Thresholds for binary heater control (in °C)
#define DEAD_BAND 2.0      // ±2°C around target temperature
#define EMERGENCY_DIFF 5.0 // Emergency cutoff if temperature exceeds target+5°C

RTC_DS3231 rtc;
DHT dht(DHTPIN, DHTTYPE);
SoftwareSerial SerialPi(9, 8);

#pragma pack(push, 1)
struct SystemState {
    float targetTemp;
    float targetHumidity;
    uint32_t targetTime;
    uint8_t checksum;
};
#pragma pack(pop)

SystemState currentState;
bool recipeReceived = false;

// Previous state tracking
float prevTemp = 0, prevHumidity = 0;
int prevFanSpeed = 0;
bool prevHeater = LOW, prevHum1 = LOW, prevHum2 = LOW;

void setup() {
    Serial.begin(115200);
    SerialPi.begin(9600);
    Wire.begin();
    dht.begin();

    // Initialize RTC
    if (!rtc.begin()) {
        Serial.println(F("RTC initialization failed!"));
        while(1);
    }
    
    if (rtc.lostPower()) {
        Serial.println(F("RTC reset to compile time"));
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }

    // Initialize hardware
    pinMode(RELAY_HUMIDIFIER_1, OUTPUT);
    pinMode(RELAY_HUMIDIFIER_2, OUTPUT);
    pinMode(RELAY_SSR, OUTPUT);
    pinMode(RELAY_FAN, OUTPUT);
    allRelaysOff();
    analogWrite(FAN_PWM_PIN, 0);

    startupTone();
    loadSystemState();
}

void loop() {
    static unsigned long lastSave = 0;
    DateTime now = rtc.now();

    if (recipeReceived) {
        // Check for recipe completion
        if (now.unixtime() >= currentState.targetTime) {
            completeRecipe();
            return;
        }

        // Save state every 60 seconds (reduces EEPROM wear)
        if (now.unixtime() - lastSave >= 60) {
            saveSystemState();
            lastSave = now.unixtime();
        }

        // Control logic
        fuzzyControl();
    }

    processSerialCommands();
}

// Improved EEPROM handling with checksum
void saveSystemState() {
    currentState.checksum = calculateChecksum();
    EEPROM.put(EEPROM_START_ADDR, currentState);
}

void loadSystemState() {
    EEPROM.get(EEPROM_START_ADDR, currentState);
    
    if (!validateState()) {
        Serial.println(F("Invalid stored state, resetting to defaults"));
        resetSystemState();
        return;
    }

    DateTime now = rtc.now();
    if (currentState.targetTime > now.unixtime()) {
        recipeReceived = true;
        digitalWrite(RELAY_FAN, HIGH);
        Serial.print(F("Resuming recipe. Time remaining: "));
        Serial.println(currentState.targetTime - now.unixtime());
    } else {
        Serial.println(F("No valid recipe to resume"));
        resetSystemState();
    }
}

void resetSystemState() {
    memset(&currentState, 0, sizeof(SystemState));
    saveSystemState(); // Save the reset state to EEPROM
}

uint8_t calculateChecksum() {
    uint8_t sum = 0;
    uint8_t* data = (uint8_t*)&currentState;
    for (size_t i = 0; i < sizeof(SystemState)-1; i++) {
        sum ^= data[i];
    }
    return sum;
}

bool validateState() {
    // Validate temperature range
    if (currentState.targetTemp < MIN_TEMP || currentState.targetTemp > MAX_TEMP) return false;
    
    // Validate humidity range
    if (currentState.targetHumidity < MIN_HUMIDITY || currentState.targetHumidity > MAX_HUMIDITY) return false;
    
    // Validate checksum
    return currentState.checksum == calculateChecksum();
}

// Updated control logic based on binary heater and table guidelines
void fuzzyControl() {
    float currentTemp = dht.readTemperature();
    float currentHumidity = dht.readHumidity();
    
    if (isnan(currentTemp)) currentTemp = prevTemp;
    if (isnan(currentHumidity)) currentHumidity = prevHumidity;

    int fanSpeed = 0;
    bool heater = prevHeater;  // Default: keep previous heater state within deadband

    // Temperature control with hysteresis
    if (currentTemp < currentState.targetTemp - DEAD_BAND) {
        heater = true;   // Below target - 2°C: heater ON
        fanSpeed = 120;  // Use a baseline fan speed (moderate airflow)
    } else if (currentTemp > currentState.targetTemp + EMERGENCY_DIFF) {
        heater = false;  // Emergency: Above target + 5°C, force heater OFF
        fanSpeed = 255;  // Set fan to maximum speed
    } else if (currentTemp > currentState.targetTemp + DEAD_BAND) {
        heater = false;  // Above target + 2°C: heater OFF
        // Map temperature from target+2 to target+5°C to fan speed between 200 and 255
        fanSpeed = map(constrain(currentTemp, currentState.targetTemp + DEAD_BAND, currentState.targetTemp + EMERGENCY_DIFF),
                       currentState.targetTemp + DEAD_BAND, currentState.targetTemp + EMERGENCY_DIFF, 200, 255);
    } else {
        // Within the deadband: maintain previous heater state and use a moderate fan speed
        fanSpeed = 150;
    }

    // Humidity control adjustments:
    // If RH is significantly above target, we force higher fan speed; if too low, we lower it and optionally engage humidifiers.
    if (currentHumidity > currentState.targetHumidity + 10) {
        fanSpeed = 255;  // Overly humid: maximize airflow
    } else if (currentHumidity > currentState.targetHumidity + 5) {
        fanSpeed = max(fanSpeed, 200);  // Ensure sufficiently high airflow
    } else if (currentHumidity < currentState.targetHumidity - 10) {
        fanSpeed = min(fanSpeed, 100);  // Too dry: reduce airflow
    } else if (currentHumidity < currentState.targetHumidity - 5) {
        fanSpeed = min(fanSpeed, 120);
    }

    // Humidifier control logic: add moisture if the chamber is too dry
    bool hum1 = false, hum2 = false;
    if (currentHumidity < currentState.targetHumidity - 5) {
        hum1 = true;
        if ((currentState.targetHumidity - currentHumidity) > 10) {
            hum2 = true;
        }
    } else {
        hum1 = false;
        hum2 = false;
    }

    // Update outputs if changed
    if (fanSpeed != prevFanSpeed) {
        analogWrite(FAN_PWM_PIN, fanSpeed);
        prevFanSpeed = fanSpeed;
    }
    if (heater != prevHeater) {
        digitalWrite(RELAY_SSR, heater ? HIGH : LOW);
        prevHeater = heater;
    }
    if (hum1 != prevHum1) {
        digitalWrite(RELAY_HUMIDIFIER_1, hum1 ? HIGH : LOW);
        prevHum1 = hum1;
    }
    if (hum2 != prevHum2) {
        digitalWrite(RELAY_HUMIDIFIER_2, hum2 ? HIGH : LOW);
        prevHum2 = hum2;
    }

    // Log state changes if there is any update
    if (currentTemp != prevTemp || currentHumidity != prevHumidity) {
        Serial.print("Temperature: ");
        Serial.print(currentTemp);
        Serial.print(" °C | Target: ");
        Serial.print(currentState.targetTemp);
        Serial.print(" °C | Heater: ");
        Serial.print(heater ? "ON" : "OFF");
        Serial.print(" | Fan PWM: ");
        Serial.println(fanSpeed);

        Serial.print("Humidity: ");
        Serial.print(currentHumidity);
        Serial.print("% | Target: ");
        Serial.println(currentState.targetHumidity);
        printSystemStatus(currentTemp, currentHumidity);
        prevTemp = currentTemp;
        prevHumidity = currentHumidity;
    }
}

void processSerialCommands() {
    if (!SerialPi.available()) return;

    StaticJsonDocument<256> doc;
    if (deserializeJson(doc, SerialPi) != DeserializationError::Ok) {
        Serial.println(F("Invalid JSON received"));
        return;
    }

    const char* command = doc["command"];
    if (!command) return;

    if (strcmp(command, "recipe") == 0) {
        startNewRecipe(doc);
    } else if (strcmp(command, "stop") == 0) {
        completeRecipe();
    } else if (strcmp(command, "settime") == 0) {
        setRTCTime(doc["unixtime"]);
    } else if (strcmp(command, "query") == 0) {
        sendStatus(SerialPi); // Send status when query command is received
        queryTone();
    }
}

void startNewRecipe(JsonDocument& doc) {
    currentState.targetTemp = doc["temperature"];
    currentState.targetHumidity = doc["humidity"];
    uint32_t duration = doc["time"];
    
    DateTime now = rtc.now();
    currentState.targetTime = now.unixtime() + duration;
    
    recipeReceived = true;
    digitalWrite(RELAY_FAN, HIGH);
    saveSystemState();
    operationTone();
    printSystemStatus(dht.readTemperature(), dht.readHumidity());
}

void completeRecipe() {
    recipeReceived = false;
    allRelaysOff();
    analogWrite(FAN_PWM_PIN, 0);
    resetSystemState(); // Reset state without clearing EEPROM
    operationTone();
    Serial.println(F("Recipe completed"));
}

// Helper functions
void allRelaysOff() {
    digitalWrite(RELAY_HUMIDIFIER_1, LOW);
    digitalWrite(RELAY_HUMIDIFIER_2, LOW);
    digitalWrite(RELAY_SSR, LOW);
    digitalWrite(RELAY_FAN, LOW);
}

void printSystemStatus(float temp, float humidity) {
    DateTime now = rtc.now();
    DateTime target(currentState.targetTime);

    Serial.println(F("\n=== SYSTEM STATUS ==="));
    Serial.print(F("Temperature: ")); Serial.print(temp);
    Serial.print(F("°C / Target: ")); Serial.print(currentState.targetTemp); Serial.println(F("°C"));
    Serial.print(F("Humidity: ")); Serial.print(humidity);
    Serial.print(F("% / Target: ")); Serial.print(currentState.targetHumidity); Serial.println(F("%"));
    Serial.print(F("Time remaining: ")); Serial.print(currentState.targetTime - now.unixtime()); Serial.println(F("s"));
    Serial.print(F("RTC: ")); printTime(now);
    Serial.print(F("Target: ")); printTime(target);
    Serial.println(F("=====================\n"));
}

void printTime(DateTime t) {
    char buf[20];
    snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
             t.year(), t.month(), t.day(),
             t.hour(), t.minute(), t.second());
    Serial.println(buf);
}

// Tone functions
void startupTone() { beep(1000, 200); }
void operationTone() { beep(1500, 200); }
void queryTone() { beep(1200, 150); }
void beep(int freq, int duration) {
    tone(BUZZER_PIN, freq, duration);
    delay(duration);
    noTone(BUZZER_PIN);
}

// Set RTC time using Unix timestamp
void setRTCTime(uint32_t unixtime) {
    rtc.adjust(DateTime(unixtime));
    Serial.print(F("RTC set to: ")); printTime(rtc.now());
}

// Send system status to Raspberry Pi
void sendStatus(SoftwareSerial &serialPort) {
    DateTime now = rtc.now();
    float currentTemp = dht.readTemperature();
    float currentHumidity = dht.readHumidity();
    long timeRemaining = currentState.targetTime - now.unixtime(); // Calculate time remaining
    
    StaticJsonDocument<256> doc;
    doc["temperature"] = currentTemp;
    doc["humidity"] = currentHumidity;
    doc["targetTemp"] = currentState.targetTemp;
    doc["targetHumidity"] = currentState.targetHumidity;
    doc["currentTime"] = now.unixtime();
    doc["targetTime"] = currentState.targetTime;
    doc["timeRemaining"] = timeRemaining; // Add time remaining to the response
    serializeJson(doc, serialPort);
    serialPort.println();
    Serial.println(F("Status sent to Raspberry Pi"));
}