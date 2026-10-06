# 📐 Inverted Pendulum on a Linear Cart: Complete Mathematical Derivations, Control Theory & Firmware Architecture

**Author**: Karim Laiti  
**Project**: `pendolo_inverso`  
**Target Hardware**: NodeMCU ESP8266 (80 MHz) + TMC2209 Stepper Driver + OMCH 1200 P/R (4800 PPR) Optical Encoder + MGN12H Rail  
**Date**: September 2026  
**Status**: Experimental Validation Complete & Documented  

---

## 📑 Table of Contents
1. [Physical System Parameters & Hardware Identification](#1-physical-system-parameters--hardware-identification)
2. [First-Principles Lagrangian Dynamics](#2-first-principles-lagrangian-dynamics)
3. [Linearization & 5-State Space Representation](#3-linearization--5-state-space-representation)
4. [Spectral Analysis & Root Cause of the Swaying Oscillation](#4-spectral-analysis--root-cause-of-the-swaying-oscillation)
5. [Continuous CARE LQR Controller Synthesis](#5-continuous-care-lqr-controller-synthesis)
6. [3-State Nonlinear Dynamic Kalman Filter (EKF)](#6-3-state-nonlinear-dynamic-kalman-filter-ekf)
7. [Lyapunov Energy-Shaping Swing-Up Control (Åström-Furuta)](#7-lyapunov-energy-shaping-swing-up-control-åström-furuta)
8. [Firmware Architecture & Line-by-Line Code Mapping](#8-firmware-architecture--line-by-line-code-mapping)

---

## 1. Physical System Parameters & Hardware Identification

The physical system consists of an inverted pendulum pinned to a motorized linear cart driven by a GT2 timing belt and a NEMA 17 stepper motor.

```
                      +-------------------+ (Tip Mass: m_tip = 10g)
                      |   Rod Length L    |
                      |   (Carbon Fiber)  |
                      |                   |
                      |     l_com (COM)   |
                      |         o         |
                      |         |         |
                      |      \  |  /      | (Angle theta: 0 = Upright, +/-PI = Down)
                      +--------(O)--------+ (OMCH Optical Encoder: 4800 PPR)
                            [ Cart ]        (Mass M_cart = 380g)
                      ======[======]======  (MGN12H Linear Rail: 650 mm)
                           <-- x -->
```

### 📊 Physical Identifications Table
| Parameter | Symbol | Identified Value | Units | Description |
| :--- | :---: | :---: | :---: | :--- |
| **Cart Mass** | $M$ | $0.380$ | $\text{kg}$ | Cart platform + MGN12H carriage + encoder mount |
| **Pendulum Total Mass** | $m$ | $0.020$ | $\text{kg}$ | $7\text{g}$ rod ($m_r$) + $10\text{g}$ tip nut ($m_t$) + $3\text{g}$ hub ($m_h$) |
| **Total Rod Length** | $L$ | $0.200$ | $\text{m}$ | $200\text{ mm}$ pultruded carbon fiber tube |
| **Center of Mass (COM)** | $l$ | $0.135$ | $\text{m}$ | $l = \frac{m_r (L/2) + m_t L + m_h(0)}{m} = \frac{0.007(0.10) + 0.010(0.20)}{0.020}$ |
| **Moment of Inertia (Pivot)** | $J$ | $4.933 \times 10^{-4}$ | $\text{kg}\cdot\text{m}^2$ | $J = \frac{1}{3} m_r L^2 + m_t L^2 = \frac{1}{3}(0.007)(0.04) + (0.010)(0.04)$ |
| **Gravitational Acceleration**| $g$ | $9.81$ | $\text{m/s}^2$ | Earth standard gravity |
| **Natural Pendulum Frequency**| $\omega_0$ | $7.327$ | $\text{rad/s}$ | $\omega_0 = \sqrt{\frac{m g l}{J}} \implies T_0 \approx 0.857\text{ s}$ |
| **Input Coupling Coefficient** | $\alpha$ | $5.474$ | $\text{rad}/(\text{m}\cdot\text{s}^2)$ | $\alpha = \frac{m l}{J}$ |
| **Lyapunov Swing-Up Energy** | $E_0$ | $0.0530$ | $\text{J}$ | $E_0 = 2 m g l = 2 \times 0.020 \times 9.81 \times 0.135$ |
| **Belt Kinematics** | $K_{\text{step}}$ | $80\,000$ | $\text{steps/m}$ | GT2 20T ($40\text{ mm/rev}$), $1/16$ microstepping ($3200\text{ steps/rev}$) |

---

## 2. First-Principles Lagrangian Dynamics

We define the generalized coordinates $q = [x, \theta]^T$, where:
* $x(t) \in \mathbb{R}$: Horizontal position of the cart ($\text{m}$).
* $\theta(t) \in [-\pi, +\pi]$: Angular displacement of the rod from the **upright vertical equilibrium** ($\text{rad}$, clockwise positive).

### 2.1 Kinematics of System Components
1. **Cart Position**:
   $$\mathbf{r}_{\text{cart}} = \begin{bmatrix} x \\ 0 \end{bmatrix}$$
2. **Pendulum Center of Mass (COM) Position**:
   $$\mathbf{r}_{\text{com}} = \begin{bmatrix} x + l \sin\theta \\ l \cos\theta \end{bmatrix}$$
3. **Pendulum COM Velocity**:
   $$\mathbf{v}_{\text{com}} = \dot{\mathbf{r}}_{\text{com}} = \begin{bmatrix} \dot{x} + l \dot{\theta} \cos\theta \\ -l \dot{\theta} \sin\theta \end{bmatrix}$$
   $$\|\mathbf{v}_{\text{com}}\|^2 = (\dot{x} + l \dot{\theta} \cos\theta)^2 + (-l \dot{\theta} \sin\theta)^2 = \dot{x}^2 + 2 l \dot{x} \dot{\theta} \cos\theta + l^2 \dot{\theta}^2$$

### 2.2 Kinetic & Potential Energies
* **Total Kinetic Energy $T$**:
  $$T = T_{\text{cart}} + T_{\text{pend}} = \frac{1}{2} M \dot{x}^2 + \frac{1}{2} m \|\mathbf{v}_{\text{com}}\|^2 + \frac{1}{2} I_{\text{com}} \dot{\theta}^2$$
  Using Parallel Axis Theorem ($J = I_{\text{com}} + m l^2$):
  $$T = \frac{1}{2} (M + m) \dot{x}^2 + m l \dot{x} \dot{\theta} \cos\theta + \frac{1}{2} J \dot{\theta}^2$$

* **Total Potential Energy $V$** (Taking $y=0$ at the pivot axis):
  $$V = m g l \cos\theta$$

### 2.3 Euler-Lagrange Equations of Motion
The Lagrangian is $\mathcal{L}(q, \dot{q}) = T - V$:
$$\mathcal{L} = \frac{1}{2} (M + m) \dot{x}^2 + m l \dot{x} \dot{\theta} \cos\theta + \frac{1}{2} J \dot{\theta}^2 - m g l \cos\theta$$

Applying the Euler-Lagrange operator $\frac{d}{dt}\left(\frac{\partial \mathcal{L}}{\partial \dot{q}_i}\right) - \frac{\partial \mathcal{L}}{\partial q_i} = Q_i$:

1. **For $q_1 = x$ (Horizontal Force $F_x$)**:
   $$\frac{\partial \mathcal{L}}{\partial \dot{x}} = (M + m)\dot{x} + m l \dot{\theta} \cos\theta$$
   $$\frac{d}{dt}\left(\frac{\partial \mathcal{L}}{\partial \dot{x}}\right) = (M + m)\ddot{x} + m l \ddot{\theta} \cos\theta - m l \dot{\theta}^2 \sin\theta$$
   $$\frac{\partial \mathcal{L}}{\partial x} = 0$$
   $$\implies (M + m)\ddot{x} + m l \ddot{\theta} \cos\theta - m l \dot{\theta}^2 \sin\theta = F_x - b \dot{x}$$

2. **For $q_2 = \theta$ (Rotational Torque at Pivot $\tau_\theta = 0$)**:
   $$\frac{\partial \mathcal{L}}{\partial \dot{\theta}} = m l \dot{x} \cos\theta + J \dot{\theta}$$
   $$\frac{d}{dt}\left(\frac{\partial \mathcal{L}}{\partial \dot{\theta}}\right) = m l \ddot{x} \cos\theta - m l \dot{x} \dot{\theta} \sin\theta + J \ddot{\theta}$$
   $$\frac{\partial \mathcal{L}}{\partial \theta} = -m l \dot{x} \dot{\theta} \sin\theta + m g l \sin\theta$$
   $$\implies J \ddot{\theta} + m l \ddot{x} \cos\theta - m g l \sin\theta = -c \dot{\theta}$$

### 2.4 Explicit Acceleration Equations
Solving for the angular acceleration $\ddot{\theta}$:
$$\ddot{\theta} = \frac{m g l}{J} \sin\theta - \frac{m l}{J} \cos\theta \ddot{x} - \frac{c}{J} \dot{\theta}$$
Defining $\omega_0^2 \triangleq \frac{m g l}{J}$ and $\alpha \triangleq \frac{m l}{J}$:
$$\ddot{\theta} = \omega_0^2 \sin\theta - \alpha \cos\theta \cdot \ddot{x} - \beta \dot{\theta}$$

> **Key Insight**: The stepper motor driven via microstepping tracks commanded velocity $v_{\text{cmd}}(t)$ and acceleration $a_{\text{cmd}}(t) = \ddot{x}$ directly through the hardware pulse train. Hence, the control input to the pendulum rotational subsystem is the **cart acceleration $u \triangleq \ddot{x}$**.

---

## 3. Linearization & 5-State Space Representation

Linearizing around the vertical upright equilibrium point:
$$\theta \approx 0 \implies \sin\theta \approx \theta, \quad \cos\theta \approx 1, \quad \dot{\theta}^2 \approx 0$$
The linear rotational equation simplifies to:
$$\ddot{\theta} = \omega_0^2 \theta - \alpha u$$

### 3.1 Definition of the 5-State Vector
To ensure **zero steady-state position error ($x \to 0$)** in the presence of track inclination or mechanical offset, we augment the state vector with the position integral $x_I \triangleq \int x \, dt$:
$$z(t) \triangleq \begin{bmatrix} x_I(t) \\ x(t) \\ v(t) \\ \theta(t) \\ \omega(t) \end{bmatrix} \in \mathbb{R}^5, \quad u(t) = a_{\text{cmd}}(t) \in \mathbb{R}$$

The dynamic state derivatives are:
$$\begin{aligned}
\dot{x}_I &= x \\
\dot{x} &= v \\
\dot{v} &= u \\
\dot{\theta} &= \omega \\
\dot{\omega} &= \omega_0^2 \theta - \alpha u
\end{aligned}$$

### 3.2 Continuous State-Space Matrices $\dot{z} = A z + B u$
$$A = \begin{bmatrix}
0 & 1 & 0 & 0 & 0 \\
0 & 0 & 1 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 1 \\
0 & 0 & 0 & \omega_0^2 & 0
\end{bmatrix} = \begin{bmatrix}
0 & 1 & 0 & 0 & 0 \\
0 & 0 & 1 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 1 \\
0 & 0 & 0 & 53.69 & 0
\end{bmatrix}, \quad
B = \begin{bmatrix}
0 \\ 0 \\ 1 \\ 0 \\ -\alpha
\end{bmatrix} = \begin{bmatrix}
0 \\ 0 \\ 1 \\ 0 \\ -5.474
\end{bmatrix}$$

---

## 4. Spectral Analysis & Root Cause of the Swaying Oscillation

During early tests, the system exhibited violent, growing left-right oscillations. Here is the mathematical proof of the root cause:

### 4.1 Non-Minimum Phase Transfer Function
The transfer function from cart acceleration $U(s) = A(s)$ to pendulum angle $\Theta(s)$ is:
$$G_{\theta u}(s) = \frac{\Theta(s)}{U(s)} = \frac{-\alpha}{s^2 - \omega_0^2} = \frac{-5.474}{(s - 7.33)(s + 7.33)}$$
Notice the **negative sign ($-\alpha$)**:
* Accelerating the cart to the **right** ($u > 0$) causes the pendulum to tilt to the **left** ($\ddot{\theta} < 0$).
* To move the cart left ($x \to 0$ from $x > 0$), the controller must **FIRST tilt the rod left ($\theta < 0$)**, which requires an **initial positive acceleration ($u > 0$)** to create the lean, followed by a sustained negative acceleration.

### 4.2 Eigenvalue Comparison: The Unstable Sign Bug
In the erroneous implementation, the feedback law was written as:
$$u = (K_\theta \theta + K_\omega \omega) - (K_x x + K_v v)$$

Writing this in standard control form $u = -K_{\text{err}} z$ with $K_{\text{err}} = [0, K_x, K_v, -K_\theta, -K_\omega]$:
$$A_{\text{cl}} = A - B K_{\text{err}} = \begin{bmatrix}
0 & 1 & 0 & 0 \\
-K_x & -K_v & K_\theta & K_\omega \\
0 & 0 & 0 & 1 \\
\alpha K_x & \alpha K_v & \omega_0^2 - \alpha K_\theta & -\alpha K_\omega
\end{bmatrix}$$

Evaluating eigenvalues for $K_x = 3.65, K_v = 5.60, K_\theta = 43.0, K_\omega = 7.45$:
$$\lambda(A_{\text{cl}}) = \{-41.75, \; -5.70, \; \mathbf{+1.61}, \; -0.51\}$$

$$\begin{aligned}
\text{The eigenvalue } \lambda_3 = \mathbf{+1.61} \in \mathbb{C}^+ \text{ (Right Half Plane)} \implies \text{\textbf{UNSTABLE LIMIT CYCLE OSCILLATION!}}
\end{aligned}$$

When the cart was at $x > 0$, the controller commanded $u < 0$ (push left immediately). This pushed the pendulum right ($\theta > 0$). The angle stabilizer then commanded $u > 0$ (push right). The two loops actively fought each other, pumping energy into a destructive resonance!

---

## 5. Continuous CARE LQR Controller Synthesis

### 5.1 Optimization Objective
We minimize the infinite-horizon quadratic performance index:
$$J = \int_{0}^{\infty} \left( z(t)^T Q z(t) + R u(t)^2 \right) dt$$

We select the state penalty matrix $Q \in \mathbb{R}^{5 \times 5}$ and control cost $R \in \mathbb{R}$:
$$Q = \operatorname{diag}\Big(q_{xi} = 1.0, \; q_x = 5.0, \; q_v = 2.0, \; q_\theta = 200.0, \; q_\omega = 5.0\Big), \quad R = [0.10]$$

### 5.2 Continuous Algebraic Riccati Equation (CARE)
We solve for the unique symmetric positive-definite matrix $P = P^T > 0$:
$$A^T P + P A - P B R^{-1} B^T P + Q = 0$$

The optimal state feedback gain vector $K \in \mathbb{R}^{1 \times 5}$ is:
$$K = R^{-1} B^T P = \begin{bmatrix} K_{0} & K_{1} & K_{2} & K_{3} & K_{4} \end{bmatrix}$$
Because $B = [0, 0, 1, 0, -\alpha]^T$, the computed gains in $u = -K z$ are all negative:
$$K = \begin{bmatrix} -0.80 & -5.00 & -7.00 & -65.00 & -10.00 \end{bmatrix}$$

Therefore, the commanded acceleration in firmware is **additive (all positive signs)**:
$$a_{\text{cmd}} = (K_\theta \theta + K_\omega \omega) + (K_x x + K_v v + K_{xi} x_I)$$

### 5.3 Closed-Loop Stability Verification
$$A_{\text{cl}} = A - B K \implies \lambda(A_{\text{cl}}) = \begin{cases}
\lambda_1 = -40.65 & (\text{Fast angle convergence, } \zeta = 1.0) \\
\lambda_2 = -5.69 & (\text{Heavily damped cart velocity}) \\
\lambda_{3,4} = -0.59 \pm 0.72j & (\text{Well-damped cart centering, } \zeta = 0.63) \\
\lambda_5 = -0.21 & (\text{Slow asymptotic integral zero tracking})
\end{cases}$$

$$\text{Every single eigenvalue has } \operatorname{Re}(\lambda_i) < 0 \implies \mathbf{100\% \text{ Asymptotically Stable}}.$$

---

## 6. 3-State Nonlinear Dynamic Kalman Filter (EKF)

Direct finite-difference numerical differentiation ($\omega = \frac{\theta_k - \theta_{k-1}}{\Delta t}$) amplifies encoder quantization noise ($0.075^\circ$) into severe motor chatter. We implement a non-linear observer.

```
                    u_accel (Commanded Cart Accel)
                           |
                           v
  [ Non-linear Model ] ---> ( + ) ---> Prediction [ theta_pred, omega_pred ]
                                |
  z_theta (Raw Encoder) ----> ( - )
                                |
                             Residual (Circular [-PI, +PI])
                                |
                                v
                          [ L Matrix ] ---> State Correction [ theta, omega, bias ]
```

### 6.1 State Equations
$$\mathbf{x}_k = \begin{bmatrix} \theta_k \\ \omega_k \\ \theta_{\text{bias}, k} \end{bmatrix} \in \mathbb{R}^3$$

1. **Continuous Non-Linear Physics**:
   $$\begin{aligned}
   \dot{\theta} &= \omega \\
   \dot{\omega} &= \omega_0^2 \sin(\theta - \theta_{\text{bias}}) - \alpha \cos\theta \cdot u \\
   \dot{\theta}_{\text{bias}} &= 0
   \end{aligned}$$

2. **Discrete Euler Propagation ($\Delta t = 5\text{ ms}$)**:
   $$\begin{aligned}
   \theta_{k+1}^{-} &= \theta_k + \Delta t \cdot \omega_k \\
   \omega_{k+1}^{-} &= \omega_k + \Delta t \cdot \left( \omega_0^2 \sin(\theta_k - \theta_{\text{bias}, k}) - \alpha \cos(\theta_k) u_k \right) \\
   \theta_{\text{bias}, k+1}^{-} &= \theta_{\text{bias}, k}
   \end{aligned}$$

3. **Measurement Innovation with Phase Wrapping**:
   $$y_k = z_{\text{encoder}} - \theta_{k+1}^{-}$$
   $$y_k \leftarrow \operatorname{atan2}(\sin y_k, \cos y_k) \in [-\pi, +\pi]$$

4. **Optimal Measurement Update**:
   $$\begin{aligned}
   \theta_{k+1} &= \theta_{k+1}^{-} + L_\theta \cdot y_k \\
   \omega_{k+1} &= \omega_{k+1}^{-} + L_\omega \cdot y_k \\
   \theta_{\text{bias}, k+1} &= \theta_{\text{bias}, k+1}^{-} + L_{\text{bias}} \cdot y_k \cdot 0.04
   \end{aligned}$$

* **Tuned Kalman Gains**: $L_\theta = 0.6845, \; L_\omega = 43.37, \; L_{\text{bias}} = -0.4291$.

---

## 7. Lyapunov Energy-Shaping Swing-Up Control (Åström-Furuta)

When the pendulum hangs at rest ($\theta = \pm \pi$), the linear LQR is ineffective because $\cos(\pi) = -1$. We employ Lyapunov Energy-Shaping.

```
            Theta = 0 (Top Equilibrium, E = 0)
                 \     |     /
                  \    |    /   <- LQR Cono di Cattura (|theta| < 22 deg)
                   \   |   /
                    \  |  /
                     \ | /
                      (O)
                     / | \
                    /  |  \
                   /   |   \
                  /    |    \
                 /     |     \
            Theta = PI (Bottom Rest, E = -2mgl = -0.053 J)
            [ Cart pumps back and forth synchronously ]
```

### 7.1 Mechanical Energy Formulation
Setting the zero reference energy ($E = 0$) at the upright vertical equilibrium ($\theta = 0, \dot{\theta} = 0$):
$$E(\theta, \dot{\theta}) = E_{\text{kin}} + E_{\text{pot}} = \frac{1}{2} J \dot{\theta}^2 + m g l (\cos\theta - 1)$$

* At **Bottom Rest** ($\theta = \pm \pi, \dot{\theta} = 0$):
  $$E_{\text{bottom}} = m g l (-1 - 1) = -2 m g l = -0.0530\text{ J}$$
* At **Upright Vertice** ($\theta = 0, \dot{\theta} = 0$):
  $$E_{\text{target}} = 0.0000\text{ J}$$

### 7.2 Energy Time-Derivative along Trajectories
$$\dot{E} = \frac{d}{dt}\left( \frac{1}{2} J \dot{\theta}^2 + m g l (\cos\theta - 1) \right) = J \dot{\theta} \ddot{\theta} - m g l \dot{\theta} \sin\theta$$

Substituting the rotational dynamics $J \ddot{\theta} = m g l \sin\theta - m l \ddot{x} \cos\theta$:
$$\dot{E} = \dot{\theta} \left( m g l \sin\theta - m l \ddot{x} \cos\theta \right) - m g l \dot{\theta} \sin\theta = -m l \dot{\theta} \cos\theta \cdot u$$

### 7.3 Lyapunov Control Law
To drive $E(t) \to 0$, we construct the Lyapunov candidate $V(E) = \frac{1}{2} (E - E_{\text{target}})^2 = \frac{1}{2} E^2$:
$$\dot{V} = E \cdot \dot{E} = -m l E \dot{\theta} \cos\theta \cdot u$$

To ensure $\dot{V} \le 0$ (energy continuously converging to target), we choose:
$$u_{\text{swing}} = k_e \cdot E \cdot \operatorname{sgn}(\dot{\theta} \cos\theta)$$
where $k_e > 0$ is the energy pump gain.

### 7.4 Rail-Bounded Swing-Up Law
To prevent the cart from drifting into the hardware endstops during swing-up:
$$u_{\text{total}} = \operatorname{sat}_{a_{\text{max}}}\Big( k_e \cdot (E - E_{\text{target}}) \cdot \operatorname{sgn}(\dot{\theta} \cos\theta) - k_p x_{\text{cart}} - k_d v_{\text{cart}} \Big)$$

---

## 8. Firmware Architecture & Line-by-Line Code Mapping

The complete firmware lives in [`firmware/src/main.cpp`](firmware/src/main.cpp) and modular headers under [`firmware/include/`](firmware/include/).

The real-time execution model runs on the ESP8266 (80 MHz) organized into prioritized tasks:
- **Hardware Timer1 ISR ($50\text{ kHz}$)**: High-frequency Bresenham DDA step generator.
- **GPIO Change ISR ($4800\text{ PPR}$)**: 4-bit optical encoder quadrature state table.
- **Real-Time Control Loop ($200\text{ Hz}$, $\Delta t = 5\text{ ms}$)**:
  1. Atomic step and encoder position capture
  2. Nonlinear Extended Kalman Filter (EKF) state estimation
  3. Supervisory state machine transition (`SWING_UP` vs `LQR_BALANCE`)
  4. 5-state CARE LQR control law computation
  5. Low-phase-lag velocity integration

### 8.1 Key Firmware Code Excerpts

#### 1. Hardware Step Generator (`Timer1` @ 50 kHz)
```cpp
void IRAM_ATTR onStepTimer() {
    if (targetStepSpeed == 0) {
        if (stepPinState) {
            stepPinState = false;
            digitalWrite(PIN_STEP, LOW);
        }
        return;
    }
    // Direction assignment
    digitalWrite(PIN_DIR, (targetStepSpeed > 0) ? HIGH : LOW);
    stepTimerAccum += (2 * abs(targetStepSpeed));

    // Bresenham accumulator overflow
    while (stepTimerAccum >= TIMER_FREQ_HZ) {
        stepTimerAccum -= TIMER_FREQ_HZ;
        stepPinState = !stepPinState;
        digitalWrite(PIN_STEP, stepPinState ? HIGH : LOW);
        if (stepPinState) {
            physicalStepsEmitted += (targetStepSpeed > 0) ? 1 : -1;
        }
    }
}
```

#### 2. Quadrature 4x State Lookup Table
```cpp
void IRAM_ATTR isrEncoder() {
    int MSB = digitalRead(PIN_ENC_A);
    int LSB = digitalRead(PIN_ENC_B);
    int encoded = (MSB << 1) | LSB;
    int sum = (lastEncoded << 2) | encoded;

    if (sum == 0b0001 || sum == 0b0111 || sum == 0b1110 || sum == 0b1000) encoderTicks++;
    if (sum == 0b0010 || sum == 0b1011 || sum == 0b1101 || sum == 0b0100) encoderTicks--;

    lastEncoded = encoded;
}
```

#### 3. 5-State LQR Control Law Execution
```cpp
if (currentState == STATE_LQR_BALANCE) {
    // 1. Position Integral Action
    cart_x_integral += cart_x * dt;
    cart_x_integral = constrain(cart_x_integral, -0.15f, 0.15f);

    // 2. Additive 5-State LQR (All Poles in Left Half Plane)
    float raw_a = (K_theta * theta + K_omega * theta_dot) + 
                  (K_x * cart_x + K_v * cart_v + K_xi * cart_x_integral);
    raw_a = constrain(raw_a, -MAX_ACCEL, MAX_ACCEL);

    // 3. Ultra-Low Phase Lag Smoothing (75% reactive, 25% filter)
    commanded_accel = 0.25f * commanded_accel + 0.75f * raw_a;
    
    // 4. Cart Velocity Integration
    cart_v += commanded_accel * dt;
    cart_v = constrain(cart_v, -MAX_VEL, MAX_VEL);
}
```

---

## 9. Summary & Quick Reference Formulas
* **Cart Position**: $x = \frac{\text{steps}}{80\,000}\text{ m}$
* **Pendulum Angle**: $\theta = \pi - \text{ticks} \cdot \left(\frac{2\pi}{4800}\right)\text{ rad}$
* **LQR Acceleration**: $a_{\text{cmd}} = (60.0 \theta + 9.5 \omega) + (4.5 x + 6.5 v + 0.5 \int x dt)$
* **Lyapunov Energy**: $E = \frac{1}{2}(0.0004933)\omega^2 + 0.0265(\cos\theta - 1)$
* **Catch Transition**: $|\theta| < 22^\circ \implies \text{STATE\_LQR\_BALANCE}$
