#include <Arduino.h>

// =================================================================================================
// 🎓 FIRMWARE ARCHITETTURA DI CONTROLLO: PENDOLO INVERSO SU GUIDA LINEARE (ESP8266 v3.4)
// =================================================================================================
//
// STRUTTURA DEL SISTEMA:
// 1. GENERATORE DI PASSI HARDWARE (Timer1 @ 50 kHz):
//    - Produce impulsi STEP ad onda quadra pulita e priva di jitter per il driver TMC2209.
//    - L'accumulatore frazionario (Bresenham) converte la velocita' continua (m/s) in frequenza di step.
//
// 2. DECODIFICA ENCODER OTTICO IN QUADRATURA 4x (Interrupt ISR):
//    - 1200 P/R su 2 canali sfasati di 90 deg -> 4800 fronti/giro (0.075 gradi di risoluzione).
//    - Macchina a stati su tabella di lookup binaria a 4 bit per immunità al rimbalzo.
//
// 3. FILTRO DI KALMAN DINAMICO NONLINEARE A 3 STATI (EKF @ 200 Hz):
//    - Vettore di stato: x = [theta (angolo), omega (velocita' angolare), bias (offset verticale)]^T.
//    - Usa le equazioni nonlineari di Lagrange per eliminare rumore di quantizzazione e derivate sporche.
//
// 4. REGOLATORE LQR LINEARE AD ALTA RIGIDITA' (Linear Quadratic Regulator):
//    - Formula: a_cmd = (K_th * theta + K_w * omega) - (K_x * x + K_v * v + K_xi * integral(x))
//    - K_x / K_v: Mantengono il carrello rigorosamente al centro della guida (x = 0).
//    - K_th / K_w: Calcolati via CARE per stabilizzare l'asta in verticale a theta = 0.
//    - K_xi: Azione integrale per eliminare la deriva lenta causata da micro-inclinazioni del banco.
//
// =================================================================================================

// -------------------------------------------------------------------------------------------------
// 1. PINOUT HARDWARE (ESP8266 NodeMCU v2)
// -------------------------------------------------------------------------------------------------
#define PIN_STEP      5   // Pin D1 (GPIO 5) -> Driver TMC2209 STEP (Ingresso treno impulsi)
#define PIN_DIR       4   // Pin D2 (GPIO 4) -> Driver TMC2209 DIR (Direzione: HIGH=DX, LOW=SX)
#define PIN_ENC_A     14  // Pin D5 (GPIO 14) -> Encoder OMCH Cavo NERO (Fase A - Open Collector)
#define PIN_ENC_B     12  // Pin D6 (GPIO 12) -> Encoder OMCH Cavo BIANCO (Fase B - Open Collector)
#define PIN_LED       2   // Pin D4 (GPIO 2) -> LED Onboard NodeMCU (Active LOW)
#define PIN_BTN_FLASH 0   // Pin D3 (GPIO 0) -> Pulsante fisico FLASH onboard (Start/Stop manuale)

// -------------------------------------------------------------------------------------------------
// 2. STATO DI SICUREZZA & ARMING
// -------------------------------------------------------------------------------------------------
bool isArmed = true;            // true = Motore attivo e reattivo, false = Standby/Fermo
unsigned long lastBtnTime = 0;  // Debounce software per tasto FLASH
int prevBtnVal = HIGH;

// -------------------------------------------------------------------------------------------------
// 3. PARAMETRI FISICI IDENTIFICATI DEL SISTEMA (SysID)
// -------------------------------------------------------------------------------------------------
const float M_cart   = 0.380f;          // Massa carrello + blocco MGN12H + piastra (kg)
const float m_pend   = 0.020f;          // Massa pendolo: 7g tubo carbonio + 10g dado punta + 3g collarino (kg)
const float l_com    = 0.135f;          // Posizione del baricentro dal centro di rotazione (m)
const float J_pivot  = 0.0004933f;      // Momento d'inerzia polare rispetto al fulcro (kg*m^2)
const float g_acc    = 9.81f;           // Accelerazione di gravita' terrestre (m/s^2)
const float E_0      = 2.0f * m_pend * g_acc * l_com; // Energia gravitazionale al vertice = 0.0530 J

// --- Offset di Calibrazione Verticale Fisico (Disabilitato = 0.00 deg) ---
float theta_calib_offset_deg = 0.00f;

// -------------------------------------------------------------------------------------------------
// 4. CINEMATICA DELLA GUIDA & LIMITI DI SICUREZZA
// -------------------------------------------------------------------------------------------------
// Puleggia GT2 20 denti, passo 2mm -> 40 mm per giro. 
// Driver TMC2209 a 1/16 microstepping -> 3200 passi/giro.
// STEPS_PER_METER = 3200 passi / 0.040 m = 80.000 passi/metro (80 microstep/mm).
const float STEPS_PER_METER = 80000.0f; 
const float MAX_CART_X      = 0.22f;    // Corsa massima software dal centro: +/- 22 cm (guida totale 65 cm)
const float MAX_ACCEL       = 10.0f;    // Accelerazione massima morbida (10 m/s^2)
const float MAX_VEL         = 0.65f;    // Velocita' massima consentita al carrello (0.65 m/s)

// -------------------------------------------------------------------------------------------------
// 5. GUADAGNI DEL CONTROLLORE LQR & SWING-UP DI LYAPUNOV
// -------------------------------------------------------------------------------------------------
// --- Parametri LQR Asintotico 5-Stati (Stabilizzazione al vertice theta = 0) ---
float K_x     = 4.50f;    // Posizione carrello (Non-minimum phase LQR term)
float K_v     = 6.50f;    // Smorzamento velocita' carrello
float K_xi    = 0.50f;    // Integrale centratura carrello su x = 0
float K_theta = 60.00f;   // Stabilizzazione primaria asta (Forza di richiamo)
float K_omega = 9.50f;    // Smorzamento oscillazioni asta (Damping)

// --- Parametri Swing-Up Nonlineare ad Energia (Åström-Furuta Energy Shaping) ---
bool  enableAutoSwingUp = true;   // true = Swing-Up autonomo attivo; false = Solo LQR
float K_E_swing        = 75.0f;  // Guadagno di pompaggio energetico
float MAX_ACCEL_SWING  = 5.50f;  // Accelerazione massima consentita in swing-up [m/s^2]
float MAX_VEL_SWING    = 0.55f;  // Velocità massima consentita in swing-up [m/s]
float K_x_swing        = 2.60f;  // Molla di centratura carrello durante swing-up
float K_v_swing        = 1.20f;  // Smorzamento carrello durante swing-up
float catch_angle_deg  = 22.0f;  // Angolo del cono di cattura LQR [deg]
float catch_max_omega  = 4.20f;  // Velocità angolare massima per aggancio LQR [rad/s]


// -------------------------------------------------------------------------------------------------
// 6. ENCODER OTTICO: ISR AD ALTA VELOCITA' (Tabella di transizione stati)
// -------------------------------------------------------------------------------------------------
volatile long encoderTicks = 0; // Contatore incrementale impulsi encoder
volatile int lastEncoded = 0;   // Ultimo stato letto (2 bit)

void IRAM_ATTR isrEncoder() {
    int MSB = digitalRead(PIN_ENC_A);
    int LSB = digitalRead(PIN_ENC_B);
    int encoded = (MSB << 1) | LSB;
    int sum = (lastEncoded << 2) | encoded;

    // Transizioni orarie (+1) e antiorarie (-1) valide in quadratura di fase
    if (sum == 0b0001 || sum == 0b0111 || sum == 0b1110 || sum == 0b1000) encoderTicks++;
    if (sum == 0b0010 || sum == 0b1011 || sum == 0b1101 || sum == 0b0100) encoderTicks--;

    lastEncoded = encoded;
}

// -------------------------------------------------------------------------------------------------
// 7. GENERATORE DI PASSI HARDWARE SU TIMER1 (Base Clock a 50 kHz / 20 us)
// -------------------------------------------------------------------------------------------------
volatile long targetStepSpeed = 0;      // Velocita' richiesta in microstep/secondo (+/-)
volatile long stepTimerAccum = 0;       // Accumulatore di fase (algoritmo DDA/Bresenham)
volatile long physicalStepsEmitted = 0; // Conteggio esatto dei passi eseguiti (Posizione odometrica)
volatile bool stepPinState = false;     // Stato logico del pin STEP
const long TIMER_FREQ_HZ = 50000;       // Frequenza base del timer hardware

void IRAM_ATTR onStepTimer() {
    if (targetStepSpeed == 0) {
        if (stepPinState) {
            stepPinState = false;
            digitalWrite(PIN_STEP, LOW);
        }
        return;
    }

    // Imposta la direzione fisica prima di emettere l'impulso (HIGH per concordanza con asta a sinistra/destra)
    if (targetStepSpeed > 0) {
        digitalWrite(PIN_DIR, HIGH);
        stepTimerAccum += (2 * targetStepSpeed);
    } else {
        digitalWrite(PIN_DIR, LOW);
        stepTimerAccum += (2 * (-targetStepSpeed));
    }

    // Quando l'accumulatore supera la soglia, inverte lo stato del pin STEP
    while (stepTimerAccum >= TIMER_FREQ_HZ) {
        stepTimerAccum -= TIMER_FREQ_HZ;
        stepPinState = !stepPinState;
        digitalWrite(PIN_STEP, stepPinState ? HIGH : LOW);
        if (stepPinState) {
            // Conta il passo solo sul fronte di salita (Rising Edge)
            physicalStepsEmitted += (targetStepSpeed > 0) ? 1 : -1;
        }
    }
}

// -------------------------------------------------------------------------------------------------
// 8. FILTRO DI KALMAN NONLINEARE DINAMICO (EKF a 3 stati @ 200 Hz)
// -------------------------------------------------------------------------------------------------
struct KalmanEstimator {
    float x_theta;  // Stima dell'angolo reale filtrato [rad]
    float x_omega;  // Stima della velocita' angolare pura [rad/s]
    float x_bias;   // Stima dell'errore di verticalita' dello zero meccanico [rad]

    // Guadagni di Kalman a regime stazionario calcolati via Riccati Discreto
    const float L_theta = 0.6845f;
    const float L_omega = 43.37f;
    const float L_bias  = -0.4291f;

    // Parametri dinamici della fisica del pendolo:
    // d2(theta)/dt2 = omega_0_sq * sin(theta) - alpha * cos(theta) * a_carrello
    const float omega_0_sq = 53.69f; // (m * g * l) / J
    const float alpha      = 5.47f;  // (m * l) / J

    void init(float init_th) {
        x_theta = init_th;
        x_omega = 0.0f;
        x_bias  = 0.0f;
    }

    void update(float z_theta, float u_accel, float dt) {
        // --- 1. Predizione dello Stato tramite Modello Fisico Nonlineare ---
        float theta_pred = x_theta + dt * x_omega;
        float omega_pred = x_omega + dt * (omega_0_sq * sin(x_theta - x_bias) - alpha * cos(x_theta) * u_accel);
        float bias_pred  = x_bias;

        // --- 2. Residuo di Misura tra Encoder e Modello (Normalizzato a [-PI, +PI]) ---
        float residual = z_theta - theta_pred;
        while (residual > PI)  residual -= 2.0f * PI;
        while (residual < -PI) residual += 2.0f * PI;

        // --- 3. Correzione Ottima Statistica (Fusione Sensore + Modello) ---
        x_theta = theta_pred + L_theta * residual;
        x_omega = omega_pred + L_omega * residual;
        
        // Stima adattiva lenta del bias solo quando l'asta e' vicina alla verticale (|theta| < 23 deg)
        if (abs(x_theta) < 0.40f) {
            x_bias = bias_pred + L_bias * residual * 0.04f;
            x_bias = constrain(x_bias, -0.15f, 0.15f); // Limita correzione max a +/- 8.5 gradi
        }

        // Normalizzazione circolare dell'angolo stimato
        while (x_theta > PI)  x_theta -= 2.0f * PI;
        while (x_theta < -PI) x_theta += 2.0f * PI;
    }
};

KalmanEstimator kf;

// -------------------------------------------------------------------------------------------------
// 9. VARIABILI GLOBALI DI CONTROLLO
// -------------------------------------------------------------------------------------------------
float cart_x = 0.0f;            // Posizione carrello calcolata [m]
float cart_v = 0.0f;            // Velocita' carrello comandata [m/s]
float cart_x_integral = 0.0f;   // Integrale dell'errore di posizione carrello [m*s]
float theta = PI;               // Angolo corrente pendolo (0 = ALTO, +/- PI = BASSO) [rad]
float theta_dot = 0.0f;         // Velocita' angolare stimata [rad/s]
float commanded_accel = 0.0f;   // Accelerazione inviata all'attuatore [m/s^2]

enum ControllerState {
    STATE_SWING_UP,             // Standby / Swing-up dal basso
    STATE_LQR_BALANCE           // Controllo attivo di bilanciamento in verticale
};
ControllerState currentState = STATE_SWING_UP;

// -------------------------------------------------------------------------------------------------
// 10. SETUP DI INIZIALIZZAZIONE HARDWARE
// -------------------------------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n===========================================================");
    Serial.println(">>> PENDOLO INVERSO: NONLINEAR ESTIMATION + LQR v3.4    <<<");
    Serial.println(">>> Loop di Controllo 200 Hz | Step Generator 50 kHz    <<<");
    Serial.println("===========================================================");

    // Configurazione GPIO
    pinMode(PIN_STEP, OUTPUT);
    pinMode(PIN_DIR, OUTPUT);
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_ENC_A, INPUT_PULLUP);
    pinMode(PIN_ENC_B, INPUT_PULLUP);
    pinMode(PIN_BTN_FLASH, INPUT_PULLUP);

    digitalWrite(PIN_STEP, LOW);
    digitalWrite(PIN_DIR, HIGH);
    digitalWrite(PIN_LED, HIGH);

    // Inizializza stato encoder
    int MSB = digitalRead(PIN_ENC_A);
    int LSB = digitalRead(PIN_ENC_B);
    lastEncoded = (MSB << 1) | LSB;

    // Collega interrupt esterni su entrambi i fronti dei canali A e B
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), isrEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), isrEncoder, CHANGE);

    // Configura Timer1 Hardware a 50 kHz (Clock CPU 80 MHz / 16 = 5 MHz -> 100 ticks = 20 us)
    timer1_attachInterrupt(onStepTimer);
    timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP);
    timer1_write(100);

    // Calibrazione di avvio a riposo (Pendolo verso il basso)
    Serial.println("-> Calibrazione completata. Zero posizionato in basso (theta = PI).");
    noInterrupts();
    encoderTicks = 0;
    physicalStepsEmitted = 0;
    interrupts();

    kf.init(PI);
    Serial.println("-> Sistema pronto. Solleva l'asta in alto per agganciare l'LQR!\n");
}

unsigned long lastControlTime = 0;
unsigned long lastTelemetryTime = 0;

// -------------------------------------------------------------------------------------------------
// 11. MAIN CONTROL LOOP (200 Hz = 5 ms)
// -------------------------------------------------------------------------------------------------
void loop() {
    // --- A. Gestione Tasto Fisico FLASH Onboard (GPIO 0: Start / Stop / Reset) ---
    int btnVal = digitalRead(PIN_BTN_FLASH);
    if (btnVal == LOW && prevBtnVal == HIGH && (millis() - lastBtnTime > 250)) {
        lastBtnTime = millis();
        isArmed = !isArmed;
        if (isArmed) {
            Serial.println("\n>>> [SISTEMA ARMATO / START]: Controllo attivo! <<<");
            noInterrupts();
            physicalStepsEmitted = 0;
            interrupts();
            cart_x = 0.0f;
            cart_v = 0.0f;
            cart_x_integral = 0.0f;
        } else {
            Serial.println("\n>>> [SISTEMA DISARMATO / STOP]: Motore Disabilitato in Sicurezza! <<<");
            cart_v = 0.0f;
            targetStepSpeed = 0;
            digitalWrite(PIN_STEP, LOW);
            digitalWrite(PIN_LED, HIGH);
        }
    }
    prevBtnVal = btnVal;

    // --- B. Gestione Comandi Seriali da Tastiera (Real-Time Tuning a Caldo) ---
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd == "s" || cmd == " ") {
            isArmed = !isArmed;
            Serial.printf("\n>>> [SISTEMA %s] <<<\n", isArmed ? "ARMATO / START" : "DISARMATO / STOP");
            if (isArmed) {
                noInterrupts();
                physicalStepsEmitted = 0;
                interrupts();
                cart_x = 0.0f;
                cart_v = 0.0f;
                cart_x_integral = 0.0f;
            } else {
                cart_v = 0.0f;
                targetStepSpeed = 0;
            }
        } else if (cmd == "r") {
            noInterrupts();
            physicalStepsEmitted = 0;
            encoderTicks = 0;
            interrupts();
            cart_x = 0.0f;
            cart_v = 0.0f;
            cart_x_integral = 0.0f;
            kf.init(PI);
            Serial.println("\n>>> [RESET ZERO]: Posizione carrello e sensori azzerati! <<<");
        } else if (cmd.startsWith("th ")) {
            K_theta = cmd.substring(3).toFloat();
            Serial.printf("-> [TUNING] K_theta = %.2f\n", K_theta);
        } else if (cmd.startsWith("w ")) {
            K_omega = cmd.substring(2).toFloat();
            Serial.printf("-> [TUNING] K_omega = %.2f\n", K_omega);
        } else if (cmd.startsWith("x ")) {
            K_x = cmd.substring(2).toFloat();
            Serial.printf("-> [TUNING] K_x = %.2f\n", K_x);
        } else if (cmd.startsWith("v ")) {
            K_v = cmd.substring(2).toFloat();
            Serial.printf("-> [TUNING] K_v = %.2f\n", K_v);
        } else if (cmd.startsWith("xi ")) {
            K_xi = cmd.substring(3).toFloat();
            Serial.printf("-> [TUNING] K_xi = %.2f\n", K_xi);
        } else if (cmd.startsWith("sw ")) {
            enableAutoSwingUp = (cmd.substring(3).toInt() == 1);
            Serial.printf("-> [TUNING] Auto Swing-Up = %s\n", enableAutoSwingUp ? "ABILITATO (ON)" : "DISABILITATO (OFF)");
        } else if (cmd.startsWith("ke ")) {
            K_E_swing = cmd.substring(3).toFloat();
            Serial.printf("-> [TUNING] K_E_swing = %.2f\n", K_E_swing);
        } else if (cmd.startsWith("asw ")) {
            MAX_ACCEL_SWING = cmd.substring(4).toFloat();
            Serial.printf("-> [TUNING] MAX_ACCEL_SWING = %.2f m/s2\n", MAX_ACCEL_SWING);
        } else if (cmd.startsWith("c ")) {
            catch_angle_deg = cmd.substring(2).toFloat();
            Serial.printf("-> [TUNING] Cono di cattura = %.1f deg\n", catch_angle_deg);
        } else if (cmd.startsWith("o ")) {
            theta_calib_offset_deg = cmd.substring(2).toFloat();
            Serial.printf("-> [TUNING] Offset tara = %.2f deg\n", theta_calib_offset_deg);
        }
    }

    unsigned long nowUs = micros();

    // --- C. Loop Real-Time a 200 Hz (Eseguito deterministicamente ogni 5000 microsecondi) ---
    if (nowUs - lastControlTime >= 5000) {
        float dt = (nowUs - lastControlTime) / 1000000.0f;
        lastControlTime = nowUs;

        // 1. Lettura atomica dei sensori fisici (Interrupt-Safe)
        long currentSteps;
        long currentTicks;
        noInterrupts();
        currentSteps = physicalStepsEmitted;
        currentTicks = encoderTicks;
        interrupts();

        // 2. Calcolo Posizione Odometrica del Carrello
        cart_x = (float)currentSteps / STEPS_PER_METER;

        // 3. Conversione Tic Encoder in Angolo Reale Tarato (0 = VERTICALE IN ALTO, +/- PI = IN BASSO)
        float theta_offset_rad = theta_calib_offset_deg * PI / 180.0f;
        float rawAngle = PI - ((float)currentTicks * (2.0f * PI / 4800.0f)) - theta_offset_rad;
        float z_theta = fmod(rawAngle + PI, 2.0f * PI);
        if (z_theta < 0) z_theta += 2.0f * PI;
        z_theta -= PI;

        // 4. Aggiornamento Filtro di Kalman (Filtra rumore e calcola velocita' angolare pura)
        kf.update(z_theta, commanded_accel, dt);
        theta = kf.x_theta;
        theta_dot = kf.x_omega;
        float eff_theta = theta;
        float angleDeg = eff_theta * 180.0f / PI;

        // 5. Calcolo Energia Meccanica Reale del Pendolo (0 = Vertice, -E_0 = Riposo in basso)
        float E_mech = 0.5f * J_pivot * theta_dot * theta_dot + m_pend * g_acc * l_com * (cos(eff_theta) - 1.0f);

        if (!isArmed) {
            // Modalità Disarmata: Motore fermo, LED lampeggia
            currentState = STATE_SWING_UP;
            cart_v = 0.0f;
            commanded_accel = 0.0f;
            targetStepSpeed = 0;
            digitalWrite(PIN_LED, ((millis() / 500) % 2 == 0) ? LOW : HIGH);
        } else {
            // --- 6. Macchina a Stati del Supervisore Ibrido (Swing-Up <-> LQR Balance) ---
            if (currentState == STATE_SWING_UP) {
                // Ingresso nel cono di cattura LQR: asta nel cono con velocità modesta e carrello in sicurezza
                if (abs(angleDeg) < catch_angle_deg && abs(theta_dot) < catch_max_omega && abs(cart_x) < (MAX_CART_X - 0.03f)) {
                    currentState = STATE_LQR_BALANCE;
                    digitalWrite(PIN_LED, LOW); // LED Fisso Acceso = LQR ATTIVO
                    
                    // Reset dell'integrale carrello all'ingresso nel cono LQR
                    cart_x_integral = 0.0f;
                }
            } else { // STATE_LQR_BALANCE
                // Uscita di sicurezza: se l'asta cade oltre la soglia di tolleranza, ritorna in swing-up
                if (abs(angleDeg) > (catch_angle_deg + 3.0f)) {
                    currentState = STATE_SWING_UP;
                    digitalWrite(PIN_LED, HIGH); // LED Spento = Swing-Up
                    cart_x_integral = 0.0f;
                }
            }

            // --- 7. Esecuzione Legge di Controllo in Base allo Stato ---
            if (currentState == STATE_LQR_BALANCE) {
                digitalWrite(PIN_LED, LOW);

                // A. Azione integrale per centrare il carrello rigorosamente a x = 0
                cart_x_integral += cart_x * dt;
                cart_x_integral = constrain(cart_x_integral, -0.15f, 0.15f);

                // B. Legge LQR 5-Stati Asintotica (stabilità globale senza cicli limite)
                float raw_a = (K_theta * eff_theta + K_omega * theta_dot) + (K_x * cart_x + K_v * cart_v + K_xi * cart_x_integral);
                raw_a = constrain(raw_a, -MAX_ACCEL, MAX_ACCEL);

                // C. Filtro passa-basso a bassissima latenza (75% reattivo, 25% memoria)
                commanded_accel = 0.25f * commanded_accel + 0.75f * raw_a;
                
                // Integrazione fluida della velocita'
                cart_v += commanded_accel * dt;
                cart_v = constrain(cart_v, -MAX_VEL, MAX_VEL);
            } else { 
                // --- STATE_SWING_UP: Algoritmo di Pompaggio Energetico Non Lineare (Lyapunov) ---
                if (enableAutoSwingUp) {
                    float raw_a = 0.0f;

                    // Se l'asta e' quasi ferma a riposo in basso (|theta| > 155 deg e |w| < 0.15 rad/s), impulso di avvio
                    if (abs(angleDeg) > 155.0f && abs(theta_dot) < 0.15f) {
                        raw_a = (cart_x <= 0.0f) ? 3.50f : -3.50f;
                    } else {
                        // Pompaggio energetico Åström:
                        float a_pump = 0.0f;
                        if (abs(angleDeg) > 70.0f) {
                            // Zona inferiore: spinta risonante proporzionale al deficit energetico E_mech < 0
                            float energy_deficit = constrain(-E_mech / E_0, 0.25f, 1.0f);
                            a_pump = (theta_dot >= 0.0f ? 1.0f : -1.0f) * (MAX_ACCEL_SWING * energy_deficit);
                        } else {
                            // Zona superiore: assistenza verso il cono LQR
                            a_pump = -(theta_dot >= 0.0f ? 1.0f : -1.0f) * (cos(eff_theta) >= 0.0f ? 1.0f : -1.0f) * (MAX_ACCEL_SWING * 0.60f);
                        }
                        // Controllo di centratura carrello per non urtare i finecorsa
                        raw_a = a_pump - (K_x_swing * cart_x + K_v_swing * cart_v);
                    }

                    raw_a = constrain(raw_a, -MAX_ACCEL_SWING, MAX_ACCEL_SWING);
                    commanded_accel = 0.30f * commanded_accel + 0.70f * raw_a;
                    cart_v += commanded_accel * dt;
                    cart_v = constrain(cart_v, -MAX_VEL_SWING, MAX_VEL_SWING);
                } else {
                    // Se lo swing-up automatico è disabilitato: carrello fermo a riposo
                    digitalWrite(PIN_LED, HIGH);
                    cart_v = 0.0f;
                    commanded_accel = 0.0f;
                    targetStepSpeed = 0;
                    cart_x_integral = 0.0f;

                    if (abs(angleDeg) > 90.0f) {
                        noInterrupts();
                        physicalStepsEmitted = 0;
                        interrupts();
                        cart_x = 0.0f;
                    }
                }
            }

            // --- 8. Limiti Morbidi di Finecorsa di Sicurezza (Soft Limits) ---
            if (cart_x >= MAX_CART_X && cart_v > 0.0f) {
                cart_v = 0.0f;
                commanded_accel = 0.0f;
                targetStepSpeed = 0;
                currentState = STATE_SWING_UP; // Sgancia istantaneamente per sicurezza
            }
            if (cart_x <= -MAX_CART_X && cart_v < 0.0f) {
                cart_v = 0.0f;
                commanded_accel = 0.0f;
                targetStepSpeed = 0;
                currentState = STATE_SWING_UP; // Sgancia istantaneamente per sicurezza
            }
        }

        // --- 9. Invio della Velocita' Target al Generatore di Passi Hardware ---
        long targetSteps = (long)(cart_v * STEPS_PER_METER);
        noInterrupts();
        targetStepSpeed = targetSteps;
        interrupts();
    }

    // --- D. Telemetria Seriale a 10 Hz (Compatibile con Visualizzatore Python) ---
    if (millis() - lastTelemetryTime >= 100) {
        lastTelemetryTime = millis();
        float angleDeg = theta * 180.0f / PI;
        float E_mech = 0.5f * J_pivot * theta_dot * theta_dot + m_pend * g_acc * l_com * (cos(theta) - 1.0f);
        const char* stateStr = (!isArmed) ? "STOP_SAFE" : ((currentState == STATE_LQR_BALANCE) ? "LQR_CATCH" : "SWING_UP ");

        Serial.printf("[%s] Th:%+6.1f° | w:%+5.1f r/s | E:%+6.3fJ | x:%+5.1fcm | a:%+5.2f\n",
                      stateStr, angleDeg, theta_dot, E_mech, cart_x * 100.0f, commanded_accel);
    }

    yield(); // Permette la gestione dei task background WiFi/System di ESP8266
}
