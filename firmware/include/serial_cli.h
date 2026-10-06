#pragma once
#include <Arduino.h>
#include "config.h"
#include "controllers.h"
#include "ekf_estimator.h"
#include "encoder.h"
#include "stepper_timer.h"

// =================================================================================================
// 📟 SERIAL CLI & TELEMETRIA NON-BLOCCANTE (Zero Watchdog Latency)
// =================================================================================================

class SerialInterface {
private:
    char rxBuffer[64];
    uint8_t rxIndex = 0;
    unsigned long lastTelemetryMs = 0;

public:
    void begin(long baudRate = 115200) {
        Serial.begin(baudRate);
        rxIndex = 0;
    }

    void printBanner() {
        Serial.println("\n===========================================================");
        Serial.println(">>> PENDOLO INVERSO: NONLINEAR ESTIMATION + LQR v4.0    <<<");
        Serial.println(">>> Architettura Modulare | Loop 200 Hz | Step 50 kHz   <<<");
        Serial.println("===========================================================");
    }

    // Parsing non-bloccante: elabora un carattere per volta senza mai bloccare il loop
    void processInput(PendulumController &ctrl, QuadratureEncoder &enc, StepperTimer &stepper, ExtendedKalmanFilter &ekf, bool &isArmed) {
        while (Serial.available() > 0) {
            char c = (char)Serial.read();

            if (c == '\n' || c == '\r') {
                if (rxIndex > 0) {
                    rxBuffer[rxIndex] = '\0';
                    executeCommand(rxBuffer, ctrl, enc, stepper, ekf, isArmed);
                    rxIndex = 0;
                }
            } else {
                if (rxIndex < sizeof(rxBuffer) - 1) {
                    rxBuffer[rxIndex++] = c;
                }
            }
        }
    }

    void streamTelemetry(const PendulumController &ctrl, const StepperTimer &stepper, const ExtendedKalmanFilter &ekf, bool isArmed) {
        if (millis() - lastTelemetryMs < TELEMETRY_PERIOD_MS) return;
        lastTelemetryMs = millis();

        float angleDeg = ekf.theta * 180.0f / PI;
        float E_mech = ctrl.computeMechanicalEnergy(ekf.theta, ekf.omega);
        float cart_x = stepper.getCartPositionMeters();

        const char* stateStr = (!isArmed) ? "STOP_SAFE" : 
                               ((ctrl.state == STATE_LQR_BALANCE) ? "LQR_CATCH" : "SWING_UP ");

        Serial.printf("[%s] Th:%+6.1f° | w:%+5.1f r/s | E:%+6.3fJ | x:%+5.1fcm | a:%+5.2f\n",
                      stateStr, angleDeg, ekf.omega, E_mech, cart_x * 100.0f, ctrl.filtered_accel);
    }

private:
    void executeCommand(char* cmd, PendulumController &ctrl, QuadratureEncoder &enc, StepperTimer &stepper, ExtendedKalmanFilter &ekf, bool &isArmed) {
        // Rimuove spazi iniziali
        while (*cmd == ' ') cmd++;

        if (strcmp(cmd, "s") == 0 || strcmp(cmd, "") == 0) {
            isArmed = !isArmed;
            Serial.printf("\n>>> [SISTEMA %s] <<<\n", isArmed ? "ARMATO / START" : "DISARMATO / STOP");
            if (isArmed) {
                stepper.resetOdometry();
                ctrl.resetState();
            } else {
                stepper.stop();
            }
        } else if (strcmp(cmd, "r") == 0) {
            stepper.resetOdometry();
            enc.reset();
            ctrl.resetState();
            ekf.init(PI);
            Serial.println("\n>>> [RESET ZERO]: Posizione carrello ed encoder azzerati! <<<");
        } else if (strncmp(cmd, "th ", 3) == 0) {
            ctrl.gains.K_theta = atof(cmd + 3);
            Serial.printf("-> [TUNING] K_theta = %.2f\n", ctrl.gains.K_theta);
        } else if (strncmp(cmd, "w ", 2) == 0) {
            ctrl.gains.K_omega = atof(cmd + 2);
            Serial.printf("-> [TUNING] K_omega = %.2f\n", ctrl.gains.K_omega);
        } else if (strncmp(cmd, "x ", 2) == 0) {
            ctrl.gains.K_x = atof(cmd + 2);
            Serial.printf("-> [TUNING] K_x = %.2f\n", ctrl.gains.K_x);
        } else if (strncmp(cmd, "v ", 2) == 0) {
            ctrl.gains.K_v = atof(cmd + 2);
            Serial.printf("-> [TUNING] K_v = %.2f\n", ctrl.gains.K_v);
        } else if (strncmp(cmd, "xi ", 3) == 0) {
            ctrl.gains.K_xi = atof(cmd + 3);
            Serial.printf("-> [TUNING] K_xi = %.2f\n", ctrl.gains.K_xi);
        } else if (strncmp(cmd, "sw ", 3) == 0) {
            ctrl.gains.enableAutoSwingUp = (atoi(cmd + 3) == 1);
            Serial.printf("-> [TUNING] Auto Swing-Up = %s\n", ctrl.gains.enableAutoSwingUp ? "ABILITATO" : "DISABILITATO");
        } else if (strncmp(cmd, "ke ", 3) == 0) {
            ctrl.gains.K_E_swing = atof(cmd + 3);
            Serial.printf("-> [TUNING] K_E_swing = %.2f\n", ctrl.gains.K_E_swing);
        } else if (strncmp(cmd, "asw ", 4) == 0) {
            ctrl.gains.MAX_ACCEL_SWING = atof(cmd + 4);
            Serial.printf("-> [TUNING] MAX_ACCEL_SWING = %.2f m/s2\n", ctrl.gains.MAX_ACCEL_SWING);
        } else if (strncmp(cmd, "c ", 2) == 0) {
            ctrl.gains.catch_angle_deg = atof(cmd + 2);
            Serial.printf("-> [TUNING] Cono di cattura = %.1f deg\n", ctrl.gains.catch_angle_deg);
        } else if (strncmp(cmd, "o ", 2) == 0) {
            ctrl.gains.theta_calib_offset_deg = atof(cmd + 2);
            Serial.printf("-> [TUNING] Offset tara = %.2f deg\n", ctrl.gains.theta_calib_offset_deg);
        }
    }
};
