#!/usr/bin/env python3
"""
Interactive 2D Real-Time Simulation: Inverted Pendulum on Linear Rail
Author: Karim Laiti (Sapienza University of Rome)
Features:
- Physical system parameters loaded from config/system_params.yaml (m=20g, l=13.5cm)
- Full nonlinear Euler-Lagrange equations of motion (RK4)
- Åström-Furuta Lyapunov energy swing-up with resonant pumping
- Continuous CARE LQR balancing around upright vertical
- Keyboard controls: Left/Right to move target, Up/Down to push, 'S' for swing-up, 'R' to reset, Space to pause
"""

import os
import yaml
import numpy as np
import scipy.linalg
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle, Circle

# Load physical parameters from YAML
CONFIG_PATH = os.path.join(os.path.dirname(__file__), "..", "config", "system_params.yaml")
with open(CONFIG_PATH, "r") as f:
    config = yaml.safe_load(f)

phys = config["physical_system"]
M = float(phys["cart_mass_M"])          # 0.380 kg (cart + carriage + encoder mount)
m = float(phys["pendulum_mass_m"])      # 0.020 kg (7g rod + 10g tip nut + 3g hub)
L = float(phys["pendulum_length_L"])    # 0.200 m (rod length)
l = float(phys["pendulum_com_l"])       # 0.135 m (center of mass distance)
J = float(phys["pendulum_inertia_J"])   # 0.0004933 kg*m^2 (inertia at pivot)
g = float(phys["gravity_g"])            # 9.81 m/s^2
b_c = float(phys["cart_friction_b"])    # 0.12 N*s/m (rail friction)
b_p = float(phys["pivot_friction_c"])   # 0.0008 N*m*s/rad (encoder bearing friction)

RAIL_LIMIT = 0.25 # Safety rail stroke (+- 25 cm from center)
MAX_FORCE = 9.5   # NEMA 17 continuous force limit (N)

det0 = (M + m) * J - (m * l)**2

# Linearized continuous state-space around vertical upright (theta = 0)
A = np.array([
    [0.0, 1.0, 0.0, 0.0],
    [0.0, -b_c * J / det0, -(m**2 * g * l**2) / det0, b_p * m * l / det0],
    [0.0, 0.0, 0.0, 1.0],
    [0.0, m * l * b_c / det0, (M + m) * m * g * l / det0, -(M + m) * b_p / det0]
])

B = np.array([
    [0.0],
    [J / det0],
    [0.0],
    [-m * l / det0]
])

# Optimal LQR Weights
Q = np.diag([160.0, 18.0, 420.0, 22.0])
R = np.array([[0.05]])

# Solve Continuous Algebraic Riccati Equation (CARE)
P_care = scipy.linalg.solve_continuous_are(A, B, Q, R)
K = (np.linalg.inv(R) @ B.T @ P_care)[0]

E0_target = 2.0 * m * g * l # 0.0530 J

print("=== Inverted Pendulum Real-Time Simulator ===")
print(f"Cart Mass M: {M:.3f} kg | Pendulum Mass m: {m:.3f} kg")
print(f"Center of Mass l: {l:.3f} m | Moment of Inertia J: {J:.7f} kg*m^2")
print(f"Target Swing-Up Energy E0: {E0_target:.4f} J")
print(f"CARE Optimal Feedback K: {np.round(K, 2)}")

# Nonlinear Euler-Lagrange RK4 integrator
def rk4_step(state, u, dt):
    def deriv(s):
        x, v, th, w = s
        sin_th = np.sin(th)
        cos_th = np.cos(th)
        det = (M + m) * J - (m * l * cos_th)**2
        
        f1 = u - b_c * v + m * l * (w**2) * sin_th
        f2 = m * g * l * sin_th - b_p * w
        
        x_acc = (J * f1 - m * l * cos_th * f2) / det
        th_acc = (-m * l * cos_th * f1 + (M + m) * f2) / det
        return np.array([v, x_acc, w, th_acc])

    k1 = deriv(state)
    k2 = deriv(state + 0.5 * dt * k1)
    k3 = deriv(state + 0.5 * dt * k2)
    k4 = deriv(state + dt * k3)
    return state + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

# Simulation state
state = np.array([0.0, 0.0, np.radians(12.0), 0.0]) # Start with 12 deg tilt
target_x = 0.0
dt = 0.016 # ~60 FPS
time_elapsed = 0.0
active_mode = "LQR (Balancing)"
is_paused = False

# Visualization Setup
fig, ax = plt.subplots(figsize=(12, 6.5))
try:
    fig.canvas.manager.set_window_title("Inverted Pendulum Mechatronic Simulator")
except Exception:
    pass

ax.set_aspect('equal')
ax.set_xlim(-0.35, 0.35)
ax.set_ylim(-0.25, 0.35)

ax.set_facecolor('#0f172a')
fig.patch.set_facecolor('#020617')
ax.grid(True, linestyle=':', color='#334155', alpha=0.6)

# Rail & Base
rail_y = -0.04
ax.fill_between([-0.325, 0.325], rail_y - 0.02, rail_y, color='#475569', zorder=2)
ax.fill_between([-0.325, 0.325], rail_y, rail_y + 0.008, color='#94a3b8', zorder=3)

# Limit switches
ax.scatter([-RAIL_LIMIT, RAIL_LIMIT], [rail_y + 0.015, rail_y + 0.015], color='#ef4444', s=90, zorder=4, marker='s')
ax.axvline(-RAIL_LIMIT, color='#ef4444', linestyle='--', lw=1.2, alpha=0.5)
ax.axvline(RAIL_LIMIT, color='#ef4444', linestyle='--', lw=1.2, alpha=0.5)

target_line = ax.axvline(x=0, color='#10b981', linestyle='--', lw=2, alpha=0.85, zorder=1)

cart_w = 0.065
cart_h = 0.035
cart_base_y = rail_y + 0.008

cart_rect = Rectangle((-cart_w/2, cart_base_y), cart_w, cart_h, fc='#0284c7', ec='#38bdf8', lw=2, zorder=5)
ax.add_patch(cart_rect)

enc_circle = Circle((0, cart_base_y + cart_h/2), 0.016, fc='#1e293b', ec='#f59e0b', lw=1.5, zorder=6)
ax.add_patch(enc_circle)

pendulum_line, = ax.plot([], [], '-', color='#f8fafc', lw=4.5, zorder=7)
tip_circle = Circle((0, 0), 0.012, fc='#e11d48', ec='#fecdd3', lw=2, zorder=8)
ax.add_patch(tip_circle)

force_line, = ax.plot([], [], color='#a855f7', lw=3.5, zorder=9, solid_capstyle='round')

bbox_props = dict(boxstyle="round,pad=0.6", fc="#0f172a", ec="#3b82f6", lw=1.5, alpha=0.92)
info_text = ax.text(0.02, 0.96, '', transform=ax.transAxes, fontsize=10, 
                    verticalalignment='top', bbox=bbox_props, family='monospace', color='#f8fafc')

def on_key(event):
    global target_x, state, is_paused
    if event.key == 'right':
        target_x = min(target_x + 0.03, RAIL_LIMIT - 0.04)
    elif event.key == 'left':
        target_x = max(target_x - 0.03, -RAIL_LIMIT + 0.04)
    elif event.key == 'up':
        state[3] += 3.0 # Perturbation tap
    elif event.key == 'down':
        state[3] -= 3.0
    elif event.key == 'r':
        state = np.array([0.0, 0.0, np.radians(12.0), 0.0])
        target_x = 0.0
    elif event.key == 's':
        # Start swing-up test from bottom
        state = np.array([0.0, 0.0, np.pi, 0.0])
        target_x = 0.0
    elif event.key == ' ':
        is_paused = not is_paused

fig.canvas.mpl_connect('key_press_event', on_key)

def init():
    pendulum_line.set_data([], [])
    force_line.set_data([], [])
    info_text.set_text('')
    return cart_rect, enc_circle, pendulum_line, tip_circle, force_line, target_line, info_text

def animate(frame):
    global state, time_elapsed, target_x, active_mode
    
    if not is_paused:
        x, v, th, w = state
        th_norm = (th + np.pi) % (2 * np.pi) - np.pi
        
        # Catch condition: angle within 22 deg and angular velocity reasonable
        if abs(th_norm) < 0.384 and abs(w) < 4.2:
            active_mode = "LQR (Balancing)"
            err_state = np.array([x - target_x, v, th_norm, w])
            u = - float(np.dot(K, err_state))
        else:
            active_mode = "SWING_UP (Lyapunov Energy)"
            if abs(np.degrees(th_norm)) > 155.0 and abs(w) < 0.2:
                # Initial kick to break symmetry at rest
                u_pump = 4.0 if (x - target_x) <= 0 else -4.0
            else:
                E = 0.5 * J * (w**2) + m * g * l * (1.0 - np.cos(th_norm))
                deficit = np.clip((E0_target - E) / E0_target, 0.2, 1.0)
                # Resonant pumping
                u_pump = (1.0 if w >= 0 else -1.0) * (5.5 * deficit)
            # Cart soft centering spring
            u = u_pump - 6.0 * (x - target_x) - 2.0 * v
            
        u = float(np.clip(u, -MAX_FORCE, MAX_FORCE))
        
        # Substepped RK4 for numerical stability
        substeps = 8
        dt_sub = dt / substeps
        for _ in range(substeps):
            state = rk4_step(state, u, dt_sub)
            
        if state[0] > RAIL_LIMIT:
            state[0] = RAIL_LIMIT
            state[1] = 0.0
        elif state[0] < -RAIL_LIMIT:
            state[0] = -RAIL_LIMIT
            state[1] = 0.0
            
        time_elapsed += dt
    else:
        u = 0.0
        
    x = state[0]
    th = state[2]
    w = state[3]
    th_deg = np.degrees((th + np.pi) % (2 * np.pi) - np.pi)
    
    pivot_x = x
    pivot_y = cart_base_y + cart_h / 2
    top_x = pivot_x + L * np.sin(th)
    top_y = pivot_y + L * np.cos(th)
    
    cart_rect.set_x(x - cart_w / 2)
    enc_circle.center = (pivot_x, pivot_y)
    pendulum_line.set_data([pivot_x, top_x], [pivot_y, top_y])
    tip_circle.center = (top_x, top_y)
    target_line.set_xdata([target_x, target_x])
    
    f_len = u * 0.006
    force_line.set_data([pivot_x, pivot_x + f_len], [pivot_y, pivot_y])
    
    info_txt = (
        f"INVERTED PENDULUM MECHATRONIC SIMULATOR\n"
        f"---------------------------------------\n"
        f"Mode          : {active_mode}\n"
        f"Time Elapsed  : {time_elapsed:5.1f} s\n"
        f"Cart Pos (x)  : {x*100:+5.1f} cm (Target: {target_x*100:+5.1f} cm)\n"
        f"Angle (theta) : {th_deg:+5.1f} deg\n"
        f"Ang Vel (w)   : {w:+5.2f} rad/s\n"
        f"Motor Force   : {u:+5.2f} N (Max +- {MAX_FORCE} N)\n"
        f"---------------------------------------\n"
        f"Left / Right  : Move cart position target\n"
        f"Up / Down     : Apply impulse push disturbance\n"
        f"S             : Test Lyapunov Swing-Up from rest\n"
        f"R             : Reset initial tilt (12 deg)\n"
        f"Space         : Pause / Resume"
    )
    info_text.set_text(info_txt)
    return cart_rect, enc_circle, pendulum_line, tip_circle, force_line, target_line, info_text

ani = animation.FuncAnimation(fig, animate, init_func=init, interval=16, blit=True)

if __name__ == "__main__":
    plt.show()
