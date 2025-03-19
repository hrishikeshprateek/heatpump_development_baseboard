# Heater Baseboard Control System

## 3. Pin Configuration
| **Component**          | **Microcontroller Pin** |
|------------------------|------------------------|
| DHT22 Sensor          | Pin 7                  |
| Buzzer                | Pin A0                 |
| Relay for Humidifier 1 | Pin 5                  |
| Relay for Humidifier 2 | Pin 15                 |
| Relay for SSR (Heater) | Pin 4                  |
| Relay for Fan         | Pin 10                 |
| BLDC Fan PWM Control  | Pin 6                  |
| SoftwareSerial RX     | Pin 9                  |
| SoftwareSerial TX     | Pin 8                  |

## 4. Control System Logic
- The system reads **temperature and humidity** values from the **DHT22 sensor**.
- Based on the **received JSON recipe**, it adjusts the **heater, fan, and humidifiers** accordingly.
- The **buzzer** provides audio feedback for different system states (e.g., start-up, warnings, completion).
- The **fan speed** is regulated through PWM, ensuring smooth air circulation.

---

## 5. Control System Reaction Table
The following table defines the system's reaction based on different environmental conditions.

| **Condition**                        | **System Reaction**                                     | **Components Activated**              |
|--------------------------------------|---------------------------------------------------------|--------------------------------------|
| **Temperature below setpoint**       | Heater turns ON, Fan at low speed                      | Heater (ON), Fan (Low)              |
| **Temperature above setpoint**       | Heater turns OFF, Fan increases speed                  | Heater (OFF), Fan (High)            |
| **Humidity below setpoint**          | Humidifier turns ON                                    | Humidifier (ON)                     |
| **Humidity above setpoint**          | Humidifier turns OFF                                  | Humidifier (OFF)                    |
| **Temperature & Humidity below setpoint** | Heater & Humidifier ON, Fan at low speed           | Heater (ON), Humidifier (ON), Fan (Low) |
| **Temperature & Humidity above setpoint** | Heater & Humidifier OFF, Fan at high speed         | Heater (OFF), Humidifier (OFF), Fan (High) |
| **Sudden temperature drop**          | Heater ON, Buzzer alert (if required)                  | Heater (ON), Buzzer (if needed)      |
| **Sudden humidity drop**             | Humidifier ON                                          | Humidifier (ON)                     |
| **Power failure and restart**        | Resume last running command from memory               | Based on stored state               |
| **System idle mode**                 | All components OFF, periodic checks                    | None (Monitoring Mode)               |
| **Emergency stop activated**         | All components OFF, buzzer alert                       | All OFF, Buzzer ON                   |

---

## 6. Memory Handling
- The **last command and state** are stored in EEPROM to resume operation after power failure.
- If the **stop command** is received, the memory is cleared.

## 7. Communication
The **Raspberry Pi 4** communicates with the **ATMEGA328P** over **Serial1 (Rx1/Tx1)**:
- Sends **JSON recipe** (Temperature, Humidity, Time).
- Receives **environmental readings** and **status updates**.

---

## 8. Buzzer Feedback Tones
| **Event**                | **Buzzer Tone**                      |
|--------------------------|--------------------------------------|
| System Startup           | Short ascending beep sequence       |
| Command Acknowledged     | Single short beep                   |
| Error / Fault Detected   | Three short beeps                   |
| Emergency Stop Triggered | Continuous long beep                |

---

## 9. Future Improvements
- Implementing **hysteresis** for smooth switching.
- Enhancing **error detection** and **failure alerts**.
- Adding **mobile app integration** for remote monitoring.
