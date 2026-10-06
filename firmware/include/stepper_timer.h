#pragma once
#include <Arduino.h>
#include "config.h"

// =================================================================================================
// ⚙️ GENERATORE DI PASSI HARDWARE (Timer1 @ 50 kHz - DDA / Bresenham)
// =================================================================================================

class StepperTimer {
public:
    static volatile long targetStepSpeed;      // Velocita' richiesta in microstep/sec (+/-)
    static volatile long stepTimerAccum;       // Accumulatore di fase DDA
    static volatile long physicalStepsEmitted; // Odometria esatta dei passi eseguiti (+/-)
    static volatile bool stepPinState;         // Livello logico del pin STEP

    static void IRAM_ATTR isrHandler() {
        if (targetStepSpeed == 0) {
            if (stepPinState) {
                stepPinState = false;
                digitalWrite(PIN_STEP, LOW);
            }
            return;
        }

        // Imposta la direzione logica prima di emettere l'impulso
        if (targetStepSpeed > 0) {
            digitalWrite(PIN_DIR, HIGH);
            stepTimerAccum += (2 * targetStepSpeed);
        } else {
            digitalWrite(PIN_DIR, LOW);
            stepTimerAccum += (2 * (-targetStepSpeed));
        }

        // Accumulatore Bresenham: genera onde quadre a 50% duty cycle
        while (stepTimerAccum >= TIMER_FREQ_HZ) {
            stepTimerAccum -= TIMER_FREQ_HZ;
            stepPinState = !stepPinState;
            digitalWrite(PIN_STEP, stepPinState ? HIGH : LOW);
            if (stepPinState) {
                // Conteggio odometrico sul fronte di salita (Rising Edge)
                physicalStepsEmitted += (targetStepSpeed > 0) ? 1 : -1;
            }
        }
    }

    void begin() {
        pinMode(PIN_STEP, OUTPUT);
        pinMode(PIN_DIR, OUTPUT);
        digitalWrite(PIN_STEP, LOW);
        digitalWrite(PIN_DIR, HIGH);

        resetOdometry();
        setSpeed(0.0f);

        // Timer1 @ 50 kHz: CPU 80 MHz / 16 = 5 MHz -> 100 tick = 20 microsecondi
        timer1_attachInterrupt(isrHandler);
        timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP);
        timer1_write(100);
    }

    void setSpeed(float cartVelocityMps) {
        long stepsPerSec = (long)(cartVelocityMps * STEPS_PER_METER);
        noInterrupts();
        targetStepSpeed = stepsPerSec;
        interrupts();
    }

    void stop() {
        setSpeed(0.0f);
        digitalWrite(PIN_STEP, LOW);
    }

    void resetOdometry() {
        noInterrupts();
        physicalStepsEmitted = 0;
        stepTimerAccum = 0;
        interrupts();
    }

    long getStepsAtomic() const {
        noInterrupts();
        long steps = physicalStepsEmitted;
        interrupts();
        return steps;
    }

    float getCartPositionMeters() const {
        return (float)getStepsAtomic() / STEPS_PER_METER;
    }
};

// Allocazione storage statico
volatile long StepperTimer::targetStepSpeed = 0;
volatile long StepperTimer::stepTimerAccum = 0;
volatile long StepperTimer::physicalStepsEmitted = 0;
volatile bool StepperTimer::stepPinState = false;
