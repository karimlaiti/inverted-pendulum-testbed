#pragma once
#include <Arduino.h>
#include "config.h"

// =================================================================================================
// 📐 FILTRO DI KALMAN NONLINEARE DINAMICO (EKF a 3 stati: theta, omega, bias @ 200 Hz)
// =================================================================================================
//
// Equazioni dinamiche nonlineari di Lagrange:
// d(theta)/dt = omega
// d(omega)/dt = (m*g*l / J) * sin(theta - bias) - (m*l / J) * cos(theta) * a_cmd
// d(bias)/dt  = 0 (stima adattiva lenta su residuo in zona lineare)
// =================================================================================================

class ExtendedKalmanFilter {
public:
    float theta;  // Stima angolo filtrato [rad]
    float omega;  // Stima velocita' angolare [rad/s]
    float bias;   // Stima offset di verticalita' dello zero meccanico [rad]

    // Guadagni di Kalman stazionari (calcolati via DARE / Riccati)
    const float L_theta = 0.6845f;
    const float L_omega = 43.37f;
    const float L_bias  = -0.4291f;

    // Parametri dinamici nonlineari
    const float omega_0_sq = 53.69f; // (m * g * l) / J = (0.020 * 9.81 * 0.135) / 0.0004933
    const float alpha      = 5.47f;  // (m * l) / J     = (0.020 * 0.135) / 0.0004933

    void init(float initialTheta = PI) {
        theta = initialTheta;
        omega = 0.0f;
        bias  = 0.0f;
    }

    void update(float z_theta, float commandedAccel, float dt) {
        // 1. Predizione dello stato tramite modello fisico nonlineare
        float theta_pred = theta + dt * omega;
        float omega_pred = omega + dt * (omega_0_sq * sin(theta - bias) - alpha * cos(theta) * commandedAccel);
        float bias_pred  = bias;

        // 2. Residuo di misura sensore vs predizione (avvolto in [-PI, +PI])
        float residual = z_theta - theta_pred;
        while (residual > PI)  residual -= 2.0f * PI;
        while (residual < -PI) residual += 2.0f * PI;

        // 3. Correzione ottima (Fusione Sensore + Modello)
        theta = theta_pred + L_theta * residual;
        omega = omega_pred + L_omega * residual;

        // Stima adattiva lenta del bias solo in prossimita' dell'equilibrio instabile (|theta| < 23 deg)
        if (abs(theta) < 0.40f) {
            bias = bias_pred + L_bias * residual * 0.04f;
            bias = constrain(bias, -0.15f, 0.15f); // Limite max +/- 8.5 gradi
        }

        // Normalizzazione circolare dell'angolo stimato in [-PI, +PI]
        while (theta > PI)  theta -= 2.0f * PI;
        while (theta < -PI) theta += 2.0f * PI;
    }
};
