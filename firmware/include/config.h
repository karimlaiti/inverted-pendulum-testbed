#pragma once
#include <Arduino.h>

// =================================================================================================
// 🎓 CONFIGURAZIONE GLOBALE: PENDOLO INVERSO SU GUIDA LINEARE (ESP8266)
// =================================================================================================

// -------------------------------------------------------------------------------------------------
// 1. PINOUT HARDWARE (ESP8266 NodeMCU v2)
// -------------------------------------------------------------------------------------------------
#define PIN_STEP      5   // Pin D1 (GPIO 5)  -> Driver TMC2209 STEP (Ingresso treno impulsi)
#define PIN_DIR       4   // Pin D2 (GPIO 4)  -> Driver TMC2209 DIR (Direzione: HIGH=DX, LOW=SX)
#define PIN_ENC_A     14  // Pin D5 (GPIO 14) -> Encoder OMCH Cavo NERO (Fase A - Open Collector)
#define PIN_ENC_B     12  // Pin D6 (GPIO 12) -> Encoder OMCH Cavo BIANCO (Fase B - Open Collector)
#define PIN_LED       2   // Pin D4 (GPIO 2)  -> LED Onboard NodeMCU (Active LOW)
#define PIN_BTN_FLASH 0   // Pin D3 (GPIO 0)  -> Pulsante fisico FLASH onboard (Start/Stop manuale)

// -------------------------------------------------------------------------------------------------
// 2. TEMPISTICHE E FREQUENZE
// -------------------------------------------------------------------------------------------------
const long  TIMER_FREQ_HZ       = 50000;    // Frequenza base del timer hardware passo-passo (50 kHz / 20 us)
const unsigned long CONTROL_PERIOD_US = 5000; // Periodo loop di controllo: 5 ms (200 Hz)
const unsigned long TELEMETRY_PERIOD_MS = 100; // Periodo streaming telemetria: 100 ms (10 Hz)

// -------------------------------------------------------------------------------------------------
// 3. PARAMETRI FISICI IDENTIFICATI DEL SISTEMA (SysID)
// -------------------------------------------------------------------------------------------------
struct PhysicalParams {
    const float M_cart   = 0.380f;          // Massa carrello + blocco MGN12H + piastra (kg)
    const float m_pend   = 0.020f;          // Massa pendolo: asta carbonio + contrappeso + collarino (kg)
    const float l_com    = 0.135f;          // Distanza fulcro - centro di massa (m)
    const float J_pivot  = 0.0004933f;      // Momento d'inerzia polare al fulcro (kg*m^2)
    const float g_acc    = 9.81f;           // Accelerazione di gravita' (m/s^2)
    const float E_0      = 2.0f * 0.020f * 9.81f * 0.135f; // Energia potenziale vertice: 0.05297 J
};

// -------------------------------------------------------------------------------------------------
// 4. CINEMATICA DELLA GUIDA & LIMITI DI SICUREZZA
// -------------------------------------------------------------------------------------------------
// Puleggia GT2 20 denti, passo 2mm -> 40 mm per giro.
// Driver TMC2209 a 1/16 microstepping -> 3200 passi/giro.
// STEPS_PER_METER = 3200 passi / 0.040 m = 80.000 passi/metro (80 microstep/mm).
const float STEPS_PER_METER = 80000.0f;
const float ENCODER_CPR     = 4800.0f;      // 1200 P/R x 4x decodifica in quadratura

const float MAX_CART_X      = 0.22f;        // Corsa massima software dal centro: +/- 22 cm (guida 65 cm)
const float MAX_ACCEL       = 10.0f;        // Accelerazione max di bilanciamento (m/s^2)
const float MAX_VEL         = 0.65f;        // Velocita' max carrello (m/s)

// -------------------------------------------------------------------------------------------------
// 5. GUADAGNI DI DEFAULT CONTROLLO
// -------------------------------------------------------------------------------------------------
struct ControllerGains {
    // --- LQR 5-Stati: [theta, omega, x, v, integral(x)] ---
    float K_theta = 60.00f;   // Rigidita' richiamo asta [N*s^2/rad o m/(s^2*rad)]
    float K_omega = 9.50f;    // Smorzamento velocita' asta
    float K_x     = 4.50f;    // Richiamo posizione carrello al centro
    float K_v     = 6.50f;    // Smorzamento velocita' carrello
    float K_xi    = 0.50f;    // Azione integrale per compensazione pendenza banco

    // --- Swing-Up Åström ad Energia & Lyapunov ---
    bool  enableAutoSwingUp = true;
    float K_E_swing        = 75.0f;   // Guadagno iniezione energia
    float MAX_ACCEL_SWING  = 5.50f;   // Accelerazione max in fase di swing-up (m/s^2)
    float MAX_VEL_SWING    = 0.55f;   // Velocita' max in swing-up (m/s)
    float K_x_swing        = 2.60f;   // Molla di centratura durante swing-up
    float K_v_swing        = 1.20f;   // Smorzamento durante swing-up
    float catch_angle_deg  = 22.0f;   // Cono di cattura LQR [deg]
    float catch_max_omega  = 4.20f;   // Velocita' angolare max per aggancio [rad/s]

    // --- Calibrazione Meccanica ---
    float theta_calib_offset_deg = 0.00f; // Offset tara verticale
};
