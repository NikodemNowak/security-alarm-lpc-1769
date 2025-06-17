# Security Alarm System for LPC1769

This project implements a security alarm system on the NXP LPC1769 microcontroller. It utilizes various peripherals and sensors to detect intrusions and alert the user.

## Features

*   **RFID Access Control:**
    *   Uses an RC522 RFID reader to authorize and deauthorize the system.
    *   A specific UID is hardcoded for authorization.
*   **Motion Detection:**
    *   Employs an HC-SR04 ultrasonic sensor to detect objects within a predefined range (currently < 200cm).
    *   Motion detection triggers an alarm state if the system is armed and not authorized.
*   **Door Sensor (Reed Switch):**
    *   Monitors a reed switch connected to a GPIO pin to detect if a door/window is open.
    *   An open door when the system is armed and unauthorized will trigger the alarm.
*   **OLED Display Interface:**
    *   Utilizes an OLED display (likely via SPI) to show system status, including:
        *   Armed/Disarmed state.
        *   Authorized/Unauthorized status.
        *   Time remaining for authorization.
        *   Alarm triggered status.
        *   Motion detection status.
        *   Door status (Open/Closed).
*   **Light Sensor with Auto Display Inversion:**
    *   Measures ambient light levels.
    *   Automatically inverts the OLED display colors for better readability in dark conditions (night mode).
*   **Audible Alerts:**
    *   Generates different sound patterns using a speaker for:
        *   System arming.
        *   System disarming.
        *   Successful RFID authorization.
        *   Alarm triggering.
*   **Visual Alerts:**
    *   Controls LEDs via a PCA9532 I2C expander to provide visual feedback (e.g., blinking during an alarm).
    *   Uses an RGB LED to indicate system status (e.g., green when disarmed/safe).
*   **Joystick Control:**
    *   Allows arming and disarming the system using a joystick (center press).
*   **Timers:**
    *   Uses timers for various delays, event timing (e.g., authorization timeout), and periodic checks.

## Hardware Components (Inferred from `main.c`)

*   NXP LPC1769 Microcontroller
*   OLED Display (interfaced via SPI)
*   RC522 RFID Reader (interfaced via SPI)
*   HC-SR04 Ultrasonic Distance Sensor
*   Light Sensor (analog or I2C)
*   Joystick
*   PCA9532 I2C LED Driver/GPIO Expander
*   Speaker/Buzzer with LM4811 Audio Amplifier (or similar driver)
*   Reed Switch (Kontaktron)
*   RGB LED

## Software Structure

The main application logic resides in `security_system/src/main.c`. It includes:

*   **Initialization routines** for all peripherals:
    *   `GPIO_init_internal()`: For general GPIO, including the reed switch.
    *   `init_i2c_internal()`: For I2C communication (e.g., with PCA9532).
    *   `init_ssp_internal()`: For SPI communication (e.g., with OLED and RC522).
    *   `oled_init()`: For the OLED display.
    *   `rgb_init()`: For the RGB LED.
    *   `initAlarm_internal()`: For the speaker/buzzer.
    *   `initUltrasonic_internal()`: For the HC-SR04 sensor.
    *   `rc522_init()`: For the RFID reader.
    *   `light_init()`: For the light sensor.
*   **Main loop (`while(1)`):**
    *   Reads joystick input for arming/disarming.
    *   Periodically checks the light sensor and adjusts OLED display inversion.
    *   If armed:
        *   Checks for RFID tag presence for authorization.
        *   Manages an authorization timer (10 seconds).
        *   Periodically checks the ultrasonic sensor for motion.
        *   Checks the reed switch for door status.
        *   Triggers alarm (visual and audible) if unauthorized and motion is detected or the door is open.
    *   Updates the OLED display with the current system status.
*   **Helper functions** for:
    *   Reading sensor data (ultrasonic distance, light level, RFID UID).
    *   Controlling actuators (OLED display, speaker, LEDs).
    *   Managing system states (armed, authorized, alarm triggered).
    *   Playing different sound patterns.

## Key System Variables

*   `isArmed`: Flag indicating if the alarm system is active.
*   `isAuthorized`: Flag indicating if a valid RFID tag has been presented recently.
*   `alarmTriggered`: Flag indicating if an alarm condition has been met.
*   `authorizedUID`: Stores the UID of the authorized RFID tag.
*   `motionDetected`: Flag indicating recent motion detection.
*   `displayInverted`: Current state of OLED display inversion.

## Building and Running

*   This project appears to be configured for an NXP LPC17xx development environment, possibly using MCUXpresso IDE or similar tools.
*   Libraries used include standard LPC17xx peripheral libraries (pinsel, i2c, gpio, timer, ssp) and custom libraries for specific components (oled, light, temp, acc, pca9532, joystick, rc522, rgb).
*   Ensure all necessary libraries (`Lib_CMSISv1p30_LPC17xx`, `Lib_EaBaseBoard`, `Lib_MCU`) are correctly linked.
*   Compile the code and flash it to the LPC1769 target board.

## Usage

1.  **Power On:** The system will initialize and display a startup message on the OLED.
2.  **Arm/Disarm:**
    *   Press the **center joystick button** to toggle the alarm system between ARMED and DISARMED states.
    *   The OLED display will update to reflect the current state.
    *   Corresponding sounds will be played.
3.  **Authorization (when ARMED):**
    *   Present the **authorized RFID tag** to the RC522 reader.
    *   If the UID matches, the system will enter an AUTHORIZED state for 10 seconds.
    *   During this time, motion or door opening will **not** trigger the alarm.
    *   A confirmation sound will play, and the OLED will indicate "AUTORYZOWANY".
4.  **Alarm Conditions (when ARMED and NOT AUTHORIZED):**
    *   **Motion Detection:** If the ultrasonic sensor detects an object closer than 200cm.
    *   **Door Open:** If the reed switch indicates the door is open.
    *   When an alarm is triggered:
        *   The OLED will display "ALARM!".
        *   The speaker will sound an alarm.
        *   LEDs controlled by PCA9532 will flash.
5.  **Silencing Alarm:**
    *   Present the **authorized RFID tag** to disarm and silence the alarm temporarily (system remains armed but enters authorized state for 10s).
    *   Press the **center joystick button** to fully disarm the system.
6.  **Automatic Display Inversion:**
    *   The system periodically checks the ambient light. If it's below `DARK_THRESHOLD` (30 lux), the OLED display colors will invert for better night visibility.

## To-Do / Potential Improvements

*   Implement more sophisticated alarm logic (e.g., entry/exit delays).
*   Allow dynamic configuration of authorized RFID UIDs (e.g., via a serial interface or a "learning mode").
*   Add functionality to store and retrieve event logs.
*   Enhance error handling and reporting.
*   Improve power management.
*   Refine sensor calibration and thresholds.

## Disclaimer

This is a basic security alarm system. For critical security applications, consider using professionally designed and certified systems.
The `authorizedUID` is currently hardcoded; ensure this is changed or managed appropriately for your security needs.
