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
#define RELAY_SSR 4
#define RELAY_FAN 10
#define FAN_PWM_PIN 6
#define EEPROM_START_ADDR 0
#define MIN_TEMP 0
#define MAX_TEMP 100
#define MIN_HUMIDITY 0
#define MAX_HUMIDITY 100

// Fuzzy logic control parameters
#define COLD_THRESHOLD 5.0    // Temp difference for full heating (degrees below target)
#define WARM_THRESHOLD 2.0    // Transition zone (degrees around target)
#define HOT_THRESHOLD  2.0    // Temp difference for cooling (degrees above target)


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

// Restructured control logic
void fuzzyControl() {
    float currentTemp = dht.readTemperature();
    float currentHumidity = dht.readHumidity();
    
    if (isnan(currentTemp)) currentTemp = prevTemp;
    if (isnan(currentHumidity)) currentHumidity = prevHumidity;

    float tempDiff = currentState.targetTemp - currentTemp;
    float absTempDiff = abs(tempDiff);

    // Fuzzy membership calculations
    float coldMembership = 0.0;
    float warmMembership = 0.0;
    float hotMembership = 0.0;

    // Cold region (needs heating)
    if(tempDiff > WARM_THRESHOLD) {
        coldMembership = 1.0;
    } else if(tempDiff > 0) {
        coldMembership = tempDiff / WARM_THRESHOLD;
    }

    // Warm region (maintain temperature)
    if(absTempDiff <= WARM_THRESHOLD) {
        warmMembership = 1.0 - (absTempDiff / WARM_THRESHOLD);
    }

    // Hot region (needs cooling)
    if(tempDiff < -HOT_THRESHOLD) {
        hotMembership = 1.0;
    } else if(tempDiff < 0) {
        hotMembership = abs(tempDiff) / HOT_THRESHOLD;
    }

    // Defuzzification - calculate fan speed
    int fanSpeed;
    if(tempDiff > 0) {  // Below target temp
        // Slower fan when colder, faster as approaching target
        fanSpeed = map(constrain(tempDiff, 0, COLD_THRESHOLD), 
                     0, COLD_THRESHOLD, 
                     100, 200);
    } else {  // At or above target temp
        // Full speed cooling when overheated
        fanSpeed = map(constrain(absTempDiff, 0, HOT_THRESHOLD), 
                     0, HOT_THRESHOLD, 
                     200, 255);
    }

    // Heater control rules
    bool heater = false;
    if(tempDiff > WARM_THRESHOLD) {
        heater = true;  // Full heating
    } else if(tempDiff > 0) {
        // Proportional heating in warm zone
        heater = (random(0,100) < (coldMembership * 100));  // Pseudo-fuzzy PWM
    }

    // Humidity control (unchanged)
    bool hum1 = (currentHumidity < currentState.targetHumidity);
    bool hum2 = (hum1 && (currentState.targetHumidity - currentHumidity) > 10);

    // Update outputs
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

    // Log state changes
    if (currentTemp != prevTemp || currentHumidity != prevHumidity) {
        Serial.print("Fuzzy memberships - Cold: ");
        Serial.print(coldMembership);
        Serial.print(" Warm: ");
        Serial.print(warmMembership);
        Serial.print(" Hot: ");
        Serial.println(hotMembership);
        printSystemStatus(currentTemp, currentHumidity);
        prevTemp = currentTemp;
        prevHumidity = currentHumidity;
    }
}
// Improved command processing
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
