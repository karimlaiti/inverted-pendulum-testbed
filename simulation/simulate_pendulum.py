"""
Inverted Pendulum on a Linear Cart: Nonlinear Simulation & Benchmark
Author: Karim Laiti (Sapienza University of Rome)
License: MIT
"""

import os
import yaml
import numpy as np
import scipy.linalg
import matplotlib.pyplot as plt

# Load system configuration
CONFIG_PATH = os.path.join(os.path.dirname(__file__), "..", "config", "system_params.yaml")
with open(CONFIG_PATH, "r") as f:
    config = yaml.safe_load(f)

phys = config["physical_system"]
M = float(phys["cart_mass_M"])
m = float(phys["pendulum_mass_m"])
L = float(phys["pendulum_length_L"])
l = float(phys["pendulum_com_l"])
J = float(phys["pendulum_inertia_J"])
g = float(phys["gravity_g"])
b_c = float(phys["cart_friction_b"])
b_p = float(phys["pivot_friction_c"])

lqr_cfg = config["control_parameters"]["lqr"]
K_th = float(lqr_cfg["K_theta"])   # 60.0
K_w = float(lqr_cfg["K_omega"])    # 9.50
K_x = float(lqr_cfg["K_x"])        # 4.50
K_v = float(lqr_cfg["K_v"])        # 6.50
K_xi = float(lqr_cfg["K_xi"])      # 0.50

omega_0_sq = (m * g * l) / J
alpha = (m * l) / J

print("=== Inverted Pendulum Identified Parameters ===")
print(f"Cart Mass M: {M:.3f} kg | Pendulum Mass m: {m:.3f} kg")
print(f"Center of Mass l: {l:.3f} m | Moment of Inertia J: {J:.7f} kg*m^2")
print(f"Natural Frequency omega_0: {np.sqrt(omega_0_sq):.3f} rad/s")
print(f"Input Coupling alpha: {alpha:.3f} rad/(m*s^2)")
print(f"Calibrated Gains: K_th={K_th}, K_w={K_w}, K_x={K_x}, K_v={K_v}, K_xi={K_xi}")

# Linearized Closed-Loop Matrix
# State vector: z = [xi, x, v, theta, omega]^T
# a_cmd = (K_th * theta + K_w * omega) + (K_x * x + K_v * v + K_xi * xi)
A_cl = np.array([
    [0.0, 1.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0, 0.0],
    [K_xi, K_x, K_v, K_th, K_w],
    [0.0, 0.0, 0.0, 0.0, 1.0],
    [-alpha * K_xi, -alpha * K_x, -alpha * K_v, omega_0_sq - alpha * K_th, -b_p / J - alpha * K_w]
])

closed_loop_poles = np.linalg.eigvals(A_cl)
print(f"Closed-Loop Poles: {np.round(closed_loop_poles, 2)}")
assert np.all(np.real(closed_loop_poles) < 0), "System is not asymptotically stable!"
print(">> Proof of Asymptotic Stability: ALL POLES IN LHP (Re(lambda) < 0)! <<")

# Nonlinear Runge-Kutta 4 Simulation
dt = 0.001  # 1 ms step
t_final = 8.0
steps = int(t_final / dt)

def nonlinear_derivatives(state, a_cmd):
    xi, x, v, th, w = state
    sin_th = np.sin(th)
    cos_th = np.cos(th)
    
    # Equation of motion:
    # J * w_dot = m * g * l * sin(th) - b_p * w - m * l * a_cmd * cos(th)
    w_dot = (m * g * l * sin_th - b_p * w - m * l * a_cmd * cos_th) / J
    
    return np.array([x, v, a_cmd, w, w_dot])

# Initial state: 12 degree angle perturbation from upright
state = np.array([0.0, 0.0, 0.0, np.radians(12.0), 0.0])

time_arr = np.linspace(0, t_final, steps)
state_arr = np.zeros((steps, 5))
u_arr = np.zeros(steps)

for i in range(steps):
    t = time_arr[i]
    state_arr[i] = state
    
    # External disturbance: tap adding 2.0 rad/s angular velocity at t = 3.5s
    if abs(t - 3.5) < dt / 2:
        state[4] += 2.0
        
    xi, x, v, th, w = state
    
    # Commanded acceleration from 5-state LQR law
    raw_a = (K_th * th + K_w * w) + (K_x * x + K_v * v + K_xi * xi)
    a_cmd = np.clip(raw_a, -9.5, 9.5)
    u_arr[i] = a_cmd
    
    # RK4 integration
    k1 = nonlinear_derivatives(state, a_cmd)
    k2 = nonlinear_derivatives(state + 0.5 * dt * k1, a_cmd)
    k3 = nonlinear_derivatives(state + 0.5 * dt * k2, a_cmd)
    k4 = nonlinear_derivatives(state + dt * k3, a_cmd)
    state = state + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

# Plotting results
fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True, dpi=150)

# Angle theta
axs[0].plot(time_arr, np.degrees(state_arr[:, 3]), color="#00E5FF", lw=2, label=r"$\theta(t)$ (deg)")
axs[0].axhline(0, color="gray", linestyle="--", alpha=0.5)
axs[0].axvline(3.5, color="#FF5252", linestyle=":", label="External Tap (+2 rad/s)")
axs[0].set_ylabel("Angle [deg]")
axs[0].grid(True, alpha=0.3)
axs[0].legend(loc="upper right")
axs[0].set_title("Inverted Pendulum Dynamics: 5-State LQR Balance & Disturbance Rejection")

# Cart Position x
axs[1].plot(time_arr, state_arr[:, 1] * 100.0, color="#76FF03", lw=2, label="Cart Position x(t) [cm]")
axs[1].axhline(0, color="gray", linestyle="--", alpha=0.5)
axs[1].axhline(22, color="#FF1744", linestyle="--", alpha=0.5, label="Soft Stroke Limit (+/- 22 cm)")
axs[1].axhline(-22, color="#FF1744", linestyle="--", alpha=0.5)
axs[1].set_ylabel("Position [cm]")
axs[1].grid(True, alpha=0.3)
axs[1].legend(loc="upper right")

# Command Acceleration a
axs[2].plot(time_arr, u_arr, color="#FFD600", lw=1.5, label=r"Commanded Acceleration $a_{\mathrm{cmd}}$ [m/s$^2$]")
axs[2].set_ylabel("Accel [m/s$^2$]")
axs[2].set_xlabel("Time [s]")
axs[2].grid(True, alpha=0.3)
axs[2].legend(loc="upper right")

output_plot = os.path.join(os.path.dirname(__file__), "benchmark_dynamics.png")
plt.tight_layout()
plt.savefig(output_plot)
print(f"\nBenchmark plot saved successfully to: {output_plot}")
