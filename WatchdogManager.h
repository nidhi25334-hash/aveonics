// WatchdogManager.h
#ifndef WATCHDOG_MANAGER_H
#define WATCHDOG_MANAGER_H

#include <Arduino.h>
#include <IWatchdog.h>

class WatchdogManager {
public:
    // Explicit tracking IDs for every critical thread in your avionics stack
    enum SystemTask {
        TASK_IMU = 0,   // MPU9250 I2C Read
        TASK_BARO,      // BMP388 SPI Read
        TASK_GPS,       // NMEA Serial Parsing
        TASK_XBEE,      // Wireless Telemetry TX
        TASK_EKF,       // Kalman Matrix Math Processing
        NUM_TASKS
    };

    // Initialize Independent Watchdog Layer
    static void begin(uint32_t hardwareTimeoutMs = 3000) {
        unsigned long now = millis();

        // --- INDUSTRY TIME-BUDGET ALLOCATIONS ---
        // Core navigation loop executes at 50Hz (20ms intervals).
        _maxTimeout[TASK_IMU]  = 150;   // Allow up to 7 consecutive dropped I2C frames
        _maxTimeout[TASK_BARO] = 150;   // Allow up to 7 consecutive dropped SPI frames
        _maxTimeout[TASK_EKF]  = 150;   // Math loop execution deadline
        
        // Downsampled Telemetry executes at 10Hz (100ms intervals)
        _maxTimeout[TASK_XBEE] = 400;   // Allow 4 missed telemetry frames before reset
        
        // GPS data streams asynchronously and can experience transient lag
        _maxTimeout[TASK_GPS]  = 2500;  // Allow up to 2.5 seconds for cold starts/signal drops

        // Initialize all tasks to current uptime stamp
        for (int i = 0; i < NUM_TASKS; i++) {
            _lastCheckInTime[i] = now;
        }

        // Initialize STM32 Hardware Watchdog
        IWatchdog.begin(hardwareTimeoutMs * 1000);
    }

    // Check if the system is recovering from a watchdog crash mid-flight
    static bool bootedFromCrash() {
        return IWatchdog.isReset(true);
    }

    // Submodules log health independently when data transactions complete
    static void checkIn(SystemTask task) {
        _lastCheckInTime[task] = millis();
    }

    // Critical supervisor called at the absolute peak of the loop() execution block
    static void update() {
        unsigned long now = millis();
        bool systemHealthy = true;
        int failedTaskIndex = -1;

        // Evaluate every single task's last check-in against its dedicated maximum timeout
        for (int i = 0; i < NUM_TASKS; i++) {
            if (now - _lastCheckInTime[i] > _maxTimeout[i]) {
                systemHealthy = false;
                failedTaskIndex = i;
                break; // Break instantly to prioritize fault containment
            }
        }

        if (systemHealthy) {
            IWatchdog.reload(); // PET THE DOG: All software constraints satisfied
        } else {
            // FAULT ISOLATION DETECTED: Intentionally withhold hardware kick.
            // Hardware countdown will hit zero and issue a clean hardware-level restart.
            Serial.print(F("\n!!! WATCHDOG FAULT DETECTED !!! Subsystem Dead Lock ID: "));
            Serial.println(failedTaskIndex);
            Serial.flush(); // Push out bytes before hardware termination
        }
    }

    // Safe delay loop for non-flight initializations
    static void safeDelay(uint32_t ms) {
        unsigned long start = millis();
        while (millis() - start < ms) {
            IWatchdog.reload();
        }
    }

private:
    static unsigned long _lastCheckInTime[NUM_TASKS];
    static unsigned long _maxTimeout[NUM_TASKS];
};

// Allocate memory for architectural static properties
unsigned long WatchdogManager::_lastCheckInTime[WatchdogManager::NUM_TASKS] = {0};
unsigned long WatchdogManager::_maxTimeout[WatchdogManager::NUM_TASKS] = {0};

#endif