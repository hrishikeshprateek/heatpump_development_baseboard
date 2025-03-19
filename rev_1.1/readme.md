Fuzzy Logic-Based Thermal Control System Documentation

Overview

This code implements a fuzzy logic-based control system for a thermal system, designed to maintain precise temperature and humidity levels. It uses a three-state fuzzy logic controller to manage the heating element (SSR) and fan speed, ensuring smooth transitions between heating, maintenance, and cooling modes. The system is optimized for energy efficiency and stability, with built-in protection against overheating.

Key Features

1. Fuzzy Logic Control:
   - Uses three fuzzy states: Cold, Warm, and Hot.
   - Smooth transitions between states for stable operation.
   - Proportional control in transition zones.

2. Temperature Management:
   - Heating: Activates the SSR when the temperature is below the target.
   - Cooling: Runs the fan at full speed when the temperature exceeds the target.
   - Maintenance: Balances heating and airflow near the target temperature.

3. Humidity Control:
   - Maintains target humidity using two humidifiers.
   - Activates the second humidifier for larger humidity deficits.

4. Diagnostics:
   - Logs fuzzy membership values and system status.
   - Provides real-time feedback for tuning and debugging.

 Fuzzy Logic States

 1. Cold Zone (ΔT > +2°C)
- Condition: Temperature is significantly below the target.

- Actions:
  - Heater: ON (full power).
  - Fan Speed: Slow (100 PWM) to retain heat.
- Purpose: Rapidly heats the system to approach the target temperature.

 2. Warm Zone (-2°C ≤ ΔT ≤ +2°C)
- Condition: Temperature is near the target.

- Actions:
  - Heater: PWM (proportional control based on temperature difference).
  - Fan Speed: Medium (100-200 PWM) for balanced airflow.
- Purpose: Maintains the target temperature with minimal oscillations.

 3. Hot Zone (ΔT < -2°C)
- Condition: Temperature exceeds the target.
- Actions:
  - Heater: OFF.
  - Fan Speed: Fast (200-255 PWM) for aggressive cooling.
- Purpose: Prevents overheating and brings the system back to the target temperature.

Control Logic

 Fuzzy Membership Functions

- Cold Membership:

  - Full membership when ΔT > +2°C.
  - Linear transition from 0 to 1 between 0°C and +2°C.

- Warm Membership:

  - Full membership when -2°C ≤ ΔT ≤ +2°C.
  - Linear transition outside this range.

- Hot Membership:

  - Full membership when ΔT < -2°C.
  - Linear transition from 0 to 1 between 0°C and -2°C.

Defuzzification

- Fan Speed:

  - Mapped based on temperature difference:
    - Cold Zone: 100-200 PWM.
    - Hot Zone: 200-255 PWM.

- Heater:
  - ON in the Cold Zone.
  - PWM in the Warm Zone.
  - OFF in the Hot Zone.

 Code Structure

 1. Setup
	- Initializes hardware (RTC, DHT sensor, relays, fan).
	- Loads the system state from EEPROM (if valid).
	- Plays a startup tone.

 2. Main Loop
	- Checks for recipe completion.
	- Saves the system state to EEPROM periodically.
	- Calls `fuzzyControl()` for temperature and humidity management.
	- Processes serial commands (e.g., start/stop recipe, query status).

 3. Fuzzy Control Logic
	- Reads current temperature and humidity.
	- Calculates fuzzy membership values for Cold, Warm, and Hot states.
	- Determines fan speed and heater state based on fuzzy rules.
	- Updates outputs (fan, heater, humidifiers) only if changes are needed.

 4. Serial Command Processing
	- Handles commands from the Raspberry Pi:
  	- `recipe`: Starts a new recipe with target temperature, humidity, and duration.
  	- `stop`: Stops the current recipe and resets the system.
  	- `settime`: Sets the RTC time using a Unix timestamp.
  	- `query`: Sends the current system status (temperature, humidity, time remaining, etc.).

 System Behavior

| Condition               | Temp Difference | Heater State | Fan Speed | Description                  |
|-------------------------|-----------------|--------------|-----------|------------------------------|
| Far Below Target        | ΔT > +5°C       | ON           | 100       | Max heating, minimal airflow |
| Approaching Target      | +2°C < ΔT < +5°C| PWM          | 100-200   | Balanced heating/circulation |
| Near Target             | -2°C < ΔT < +2°C| OFF/PWM      | 200-255   | Maintenance mode             |
| Above Target            | ΔT < -2°C       | OFF          | 255       | Emergency cooling            |

 Tuning Parameters

| Parameter               | Description                                  | Default Value |
|-------------------------|----------------------------------------------|---------------|
| `COLD_THRESHOLD`        | Temperature difference for full heating      | 5.0°C         |
| `WARM_THRESHOLD`        | Transition zone around the target temperature| 2.0°C         |
| `HOT_THRESHOLD`         | Temperature difference for cooling           | 2.0°C         |
| `FAN_MIN_SPEED`         | Minimum fan speed (PWM)                      | 100           |
| `FAN_MAX_SPEED`         | Maximum fan speed (PWM)                      | 255           |

 Example JSON Commands


{
  "command": "recipe",
  "temperature": 80.0,
  "humidity": 50.0,
  "time": 3600
}


 Stop Recipe

{
  "command": "stop"
}


 Diagnostic Logging


Fuzzy memberships - Cold: 0.75 Warm: 0.25 Hot: 0.00
=== SYSTEM STATUS ===
Temperature: 29.50°C / Target: 80.00°C
Humidity: 59.80% / Target: 80.00%
Time remaining: 3600s
RTC: 2025-03-11 22:34:55
Target: 2025-03-11 23:34:55
=====================


Conclusion

This fuzzy logic-based control system provides precise and stable temperature and humidity management. It is highly configurable and includes robust diagnostics for tuning and troubleshooting. The use of fuzzy logic ensures smooth transitions between states, minimizing oscillations and improving energy efficiency.
