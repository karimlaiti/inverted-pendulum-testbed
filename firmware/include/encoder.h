#pragma once
#include <Arduino.h>
#include "config.h"

// =================================================================================================
// 🔍 DECODIFICA ENCODER OTTICO IN QUADRATURA 4X (Macchina a stati su Look-Up Table)
// =================================================================================================

class QuadratureEncoder {
public:
    static volatile long encoderTicks;
    static volatile int lastEncoded;

    static void IRAM_ATTR isrHandler() {
        int MSB = digitalRead(PIN_ENC_A);
        int LSB = digitalRead(PIN_ENC_B);
        int encoded = (MSB << 1) | LSB;
        int sum = (lastEncoded << 2) | encoded;

        // Transizioni di quadratura valide: orarie (+1) e antiorarie (-1)
        if (sum == 0b0001 || sum == 0b0111 || sum == 0b1110 || sum == 0b1000) encoderTicks++;
        if (sum == 0b0010 || sum == 0b1011 || sum == 0b1101 || sum == 0b0100) encoderTicks--;

        lastEncoded = encoded;
    }

    void begin() {
        pinMode(PIN_ENC_A, INPUT_PULLUP);
        pinMode(PIN_ENC_B, INPUT_PULLUP);

        int MSB = digitalRead(PIN_ENC_A);
        int LSB = digitalRead(PIN_ENC_B);
        lastEncoded = (MSB << 1) | LSB;

        attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), isrHandler, CHANGE);
        attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), isrHandler, CHANGE);
        reset();
    }

    void reset() {
        noInterrupts();
        encoderTicks = 0;
        interrupts();
    }

    long getTicksAtomic() const {
        noInterrupts();
        long ticks = encoderTicks;
        interrupts();
        return ticks;
    }

    // Calcola l'angolo grezzo avvolto in [-PI, +PI] con 0 = Verticale in Alto, +/- PI = In Basso
    float getWrappedAngle(float calibOffsetDeg = 0.0f) const {
        long ticks = getTicksAtomic();
        float offsetRad = calibOffsetDeg * PI / 180.0f;
        float rawAngle = PI - ((float)ticks * (2.0f * PI / ENCODER_CPR)) - offsetRad;
        
        float z = fmod(rawAngle + PI, 2.0f * PI);
        if (z < 0) z += 2.0f * PI;
        return z - PI;
    }
};

// Allocazione storage statico
volatile long QuadratureEncoder::encoderTicks = 0;
volatile int QuadratureEncoder::lastEncoded = 0;
