#pragma once
#include <Arduino.h>
#include "config.h"

// =================================================================================================
// 🎮 CONTROLLER ENGINE & SUPERVISORE IBRIDO (LQR 5-Stati + Swing-Up Nonlineare)
// =================================================================================================

enum SystemState {
    STATE_DISARMED,      // Motore spento, standby di sicurezza
    STATE_SWING_UP,      // Pompaggio energetico non lineare dal basso
    STATE_LQR_BALANCE    // Stabilizzazione lineare LQR nel cono di cattura
};

class PendulumController {
public:
    PhysicalParams params;
    ControllerGains gains;

    SystemState state = STATE_SWING_UP;
    float cart_x_integral = 0.0f;
    float filtered_accel  = 0.0f;
    float commanded_vel   = 0.0f;

    void resetIntegral() {
        cart_x_integral = 0.0f;
    }

    void resetState() {
        state = STATE_SWING_UP;
        cart_x_integral = 0.0f;
        filtered_accel = 0.0f;
        commanded_vel = 0.0f;
    }

    // Calcolo dell'Energia Meccanica Reale (E = 0 al vertice, E = -E_0 a riposo in basso)
    float computeMechanicalEnergy(float theta, float omega) const {
        return 0.5f * params.J_pivot * omega * omega + params.m_pend * params.g_acc * params.l_com * (cos(theta) - 1.0f);
    }

    // Valutazione transizione di stato della FSM
    void updateFSM(float theta, float omega, float cart_x, bool isArmed) {
        if (!isArmed) {
            state = STATE_DISARMED;
            return;
        }

        float angleDeg = theta * 180.0f / PI;

        if (state == STATE_DISARMED || state == STATE_SWING_UP) {
            // Condizione di cattura nel cono LQR
            if (abs(angleDeg) < gains.catch_angle_deg && 
                abs(omega) < gains.catch_max_omega && 
                abs(cart_x) < (MAX_CART_X - 0.03f)) {
                state = STATE_LQR_BALANCE;
                resetIntegral();
            } else {
                state = STATE_SWING_UP;
            }
        } else if (state == STATE_LQR_BALANCE) {
            // Isteresi di sgancio dal cono LQR
            if (abs(angleDeg) > (gains.catch_angle_deg + 3.0f)) {
                state = STATE_SWING_UP;
                resetIntegral();
            }
        }
    }

    // Calcolo dell'accelerazione e velocita' di comando al carrello
    float compute(float theta, float omega, float cart_x, float dt) {
        if (state == STATE_DISARMED) {
            commanded_vel = 0.0f;
            filtered_accel = 0.0f;
            return 0.0f;
        }

        float raw_accel = 0.0f;
        float angleDeg = theta * 180.0f / PI;

        if (state == STATE_LQR_BALANCE) {
            // --- 1. LQR 5-STATI CON AZIONE INTEGRALE SULLA POSIZIONE ---
            cart_x_integral += cart_x * dt;
            cart_x_integral = constrain(cart_x_integral, -0.15f, 0.15f);

            // Legge di controllo di stato aumentato:
            // a_cmd = (K_th * theta + K_w * omega) + (K_x * x + K_v * v + K_xi * integral(x))
            raw_accel = (gains.K_theta * theta + gains.K_omega * omega) + 
                        (gains.K_x * cart_x + gains.K_v * commanded_vel + gains.K_xi * cart_x_integral);

            raw_accel = constrain(raw_accel, -MAX_ACCEL, MAX_ACCEL);
            filtered_accel = 0.25f * filtered_accel + 0.75f * raw_accel;

            commanded_vel += filtered_accel * dt;
            commanded_vel = constrain(commanded_vel, -MAX_VEL, MAX_VEL);

        } else if (state == STATE_SWING_UP) {
            // --- 2. SWING-UP NONLINEARE AD ENERGIA (ÅSTRÖM-FURUTA) ---
            if (gains.enableAutoSwingUp) {
                float E_mech = computeMechanicalEnergy(theta, omega);

                if (abs(angleDeg) > 155.0f && abs(omega) < 0.15f) {
                    // Impulso iniziale di sblocco da riposo
                    raw_accel = (cart_x <= 0.0f) ? 3.50f : -3.50f;
                } else {
                    float a_pump = 0.0f;
                    if (abs(angleDeg) > 70.0f) {
                        float energy_deficit = constrain(-E_mech / params.E_0, 0.25f, 1.0f);
                        a_pump = (omega >= 0.0f ? 1.0f : -1.0f) * (gains.MAX_ACCEL_SWING * energy_deficit);
                    } else {
                        a_pump = -(omega >= 0.0f ? 1.0f : -1.0f) * (cos(theta) >= 0.0f ? 1.0f : -1.0f) * (gains.MAX_ACCEL_SWING * 0.60f);
                    }
                    raw_accel = a_pump - (gains.K_x_swing * cart_x + gains.K_v_swing * commanded_vel);
                }

                raw_accel = constrain(raw_accel, -gains.MAX_ACCEL_SWING, gains.MAX_ACCEL_SWING);
                filtered_accel = 0.30f * filtered_accel + 0.70f * raw_accel;

                commanded_vel += filtered_accel * dt;
                commanded_vel = constrain(commanded_vel, -gains.MAX_VEL_SWING, gains.MAX_VEL_SWING);
            } else {
                commanded_vel = 0.0f;
                filtered_accel = 0.0f;
                resetIntegral();
            }
        }

        // --- 3. SOFT LIMITS DI SICUREZZA SUI FINECORSA ---
        if ((cart_x >= MAX_CART_X && commanded_vel > 0.0f) || (cart_x <= -MAX_CART_X && commanded_vel < 0.0f)) {
            commanded_vel = 0.0f;
            filtered_accel = 0.0f;
            state = STATE_SWING_UP;
        }

        return commanded_vel;
    }
};
