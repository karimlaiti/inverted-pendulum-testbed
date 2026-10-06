#include <Arduino.h>
#include "config.h"
#include "encoder.h"
#include "stepper_timer.h"
#include "ekf_estimator.h"
#include "controllers.h"
#include "serial_cli.h"

// =================================================================================================
// 🎓 FIRMWARE PENDOLO INVERSO SU GUIDA LINEARE (ESP8266 v4.0 - Modular Architecture)
// =================================================================================================

// Istanze dei moduli di sistema
QuadratureEncoder    encoder;
StepperTimer         stepper;
ExtendedKalmanFilter ekf;
PendulumController   controller;
SerialInterface      serialCli;

// Stato operativo e sicurezza
bool isArmed = true;
int prevBtnVal = HIGH;
unsigned long lastBtnTime = 0;
unsigned long lastControlUs = 0;

void handleHardwareButton() {
    int btnVal = digitalRead(PIN_BTN_FLASH);
    if (btnVal == LOW && prevBtnVal == HIGH && (millis() - lastBtnTime > 250)) {
        lastBtnTime = millis();
        isArmed = !isArmed;
        if (isArmed) {
            Serial.println("\n>>> [SISTEMA ARMATO / START]: Controllo attivo! <<<");
            stepper.resetOdometry();
            controller.resetState();
        } else {
            Serial.println("\n>>> [SISTEMA DISARMATO / STOP]: Motore Disabilitato in Sicurezza! <<<");
            stepper.stop();
            digitalWrite(PIN_LED, HIGH);
        }
    }
    prevBtnVal = btnVal;
}

void setup() {
    // 1. Inizializzazione Interfaccia Seriale
    serialCli.begin(115200);
    delay(1000);
    serialCli.printBanner();

    // 2. Inizializzazione GPIO & Pulsanti
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_BTN_FLASH, INPUT_PULLUP);
    digitalWrite(PIN_LED, HIGH);

    // 3. Inizializzazione Hardware Subsystems
    encoder.begin();
    stepper.begin();
    ekf.init(PI); // Zero iniziale a riposo in basso (theta = PI)

    Serial.println("-> Calibrazione completata. Zero posizionato in basso (theta = PI).");
    Serial.println("-> Sistema pronto. Solleva l'asta in alto per agganciare l'LQR!\n");
}

void loop() {
    // A. Gestione Tasto Fisico Onboard (GPIO 0: Start / Stop)
    handleHardwareButton();

    // B. Parser Seriale Non-Bloccante (Zero Timeout / Zero Jitter)
    serialCli.processInput(controller, encoder, stepper, ekf, isArmed);

    unsigned long nowUs = micros();

    // C. Control Loop Real-Time @ 200 Hz (5000 us deterministici)
    if (nowUs - lastControlUs >= CONTROL_PERIOD_US) {
        float dt = (nowUs - lastControlUs) / 1000000.0f;
        lastControlUs = nowUs;

        // 1. Lettura Sensori Odometrici ed Angolari
        float cart_x = stepper.getCartPositionMeters();
        float z_theta = encoder.getWrappedAngle(controller.gains.theta_calib_offset_deg);

        // 2. Filtro di Kalman Dinamico Nonlineare (Stima theta, omega, bias)
        ekf.update(z_theta, controller.filtered_accel, dt);

        // 3. Gestione LED di stato
        if (!isArmed) {
            digitalWrite(PIN_LED, ((millis() / 500) % 2 == 0) ? LOW : HIGH); // Lampeggio lento
        } else {
            digitalWrite(PIN_LED, (controller.state == STATE_LQR_BALANCE) ? LOW : HIGH); // Acceso fisso in LQR
        }

        // 4. Aggiornamento Supervisore Ibrido (FSM)
        controller.updateFSM(ekf.theta, ekf.omega, cart_x, isArmed);

        // 5. Calcolo Legge di Controllo (LQR o Swing-Up)
        float targetVelocity = controller.compute(ekf.theta, ekf.omega, cart_x, dt);

        // 6. Invio Velocita' al Generatore Hardware Timer1
        stepper.setSpeed(targetVelocity);
    }

    // D. Telemetria Seriale a 10 Hz
    serialCli.streamTelemetry(controller, stepper, ekf, isArmed);

    yield(); // Gestione task di sistema ESP8266
}
