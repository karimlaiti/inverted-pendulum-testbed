#!/usr/bin/env python3
"""
Publication-Grade Inverted Pendulum LQR Simulation Video
Author: Karim Laiti (Sapienza University of Rome)
Generates 1080p 60fps MP4 combining:
1. Top: Full physical 2D animation with rail, cart, pendulum rod, tip mass, and live force vector
2. Bottom: 3 live-updating oscilloscopes (Angle, Cart Position, Actuator Force) + HUD
Based directly on Karim's benchmark in lqr_simulation.py and plot_dynamics.py
"""

import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
import matplotlib.animation as animation
import scipy.linalg

# --- 1. SYSTEM CONSTANTS & LQR SYNTHESIS ---
g = 9.81
M = 0.380       # Cart mass (kg)
m = 0.045       # Pendulum mass (kg)
L = 0.200       # Rod length (m)
l = 0.130       # COM distance (m)
J = 0.00105     # Pivot inertia (kg*m^2)
b_c = 0.12      # Cart friction (N*s/m)
b_p = 0.0008    # Pivot damping (N*m*s/rad)
MAX_FORCE = 9.5 # NEMA 17 limit (N)
RAIL_LIMIT = 0.25 # Safety stroke (m)

det0 = (M + m) * J - (m * l)**2

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

Q = np.diag([120.0, 15.0, 500.0, 25.0])
R = np.array([[0.05]])
S = scipy.linalg.solve_continuous_are(A, B, Q, R)
K = (np.linalg.inv(R) @ B.T @ S)[0]

# --- 2. PRE-COMPUTE TRAJECTORY (60 FPS, 6.0 SECONDS) ---
FPS = 60
T_MAX = 6.0
N_FRAMES = int(T_MAX * FPS)
dt_frame = 1.0 / FPS
substeps = 20
dt_sub = dt_frame / substeps

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

# Initial State: Inverted equilibrium with an initial +12.0° tilt disturbance
state = np.array([0.0, 0.0, np.radians(12.0), 0.0])

t_arr = np.zeros(N_FRAMES)
x_arr = np.zeros(N_FRAMES)
th_deg_arr = np.zeros(N_FRAMES)
w_arr = np.zeros(N_FRAMES)
u_arr = np.zeros(N_FRAMES)

for frame in range(N_FRAMES):
    t = frame * dt_frame
    t_arr[frame] = t
    x_arr[frame] = state[0]
    th_deg_arr[frame] = np.degrees(state[2])
    w_arr[frame] = state[3]
    
    # External disturbance: tap adding +2.5 rad/s at t = 3.0s
    if 3.0 <= t < 3.0 + dt_frame:
        state[3] += 2.5
        
    u = - float(np.dot(K, state))
    u = float(np.clip(u, -MAX_FORCE, MAX_FORCE))
    u_arr[frame] = u
    
    for _ in range(substeps):
        state = rk4_step(state, u, dt_sub)
        if state[0] > RAIL_LIMIT:
            state[0] = RAIL_LIMIT
            state[1] = 0.0
        elif state[0] < -RAIL_LIMIT:
            state[0] = -RAIL_LIMIT
            state[1] = 0.0

print("Trajectory pre-computed cleanly. Building high-end visualization...")

# --- 3. ADVANCED VISUALIZATION SETUP (1920x1080 @ 60 FPS) ---
plt.style.use('dark_background')
fig = plt.figure(figsize=(16, 9), dpi=120)
fig.patch.set_facecolor('#020617')

# Layout: Top = Physical animation (58%), Bottom = 3 Oscilloscopes (42%)
gs = fig.add_gridspec(2, 3, height_ratios=[1.25, 0.95], hspace=0.32, wspace=0.25,
                       left=0.06, right=0.96, top=0.93, bottom=0.08)

ax_anim = fig.add_subplot(gs[0, :])
ax1 = fig.add_subplot(gs[1, 0])
ax2 = fig.add_subplot(gs[1, 1])
ax3 = fig.add_subplot(gs[1, 2])

# Styling of plots
for ax in [ax1, ax2, ax3]:
    ax.set_facecolor('#0f172a')
    ax.grid(True, color='#334155', linestyle=':', alpha=0.6)
    ax.tick_params(colors='#94a3b8', labelsize=9)
    for spine in ax.spines.values():
        spine.set_color('#334155')

# Physical Animation Axes
ax_anim.set_facecolor('#0f172a')
ax_anim.set_xlim(-0.35, 0.35)
ax_anim.set_ylim(-0.25, 0.35)
ax_anim.set_aspect('equal')
ax_anim.axis('off')

# Title
ax_anim.text(0.0, 0.32, "INVERTED PENDULUM BENCHMARK: CONTINUOUS LQR & DISTURBANCE REJECTION", 
             ha='center', va='center', color='#00E5FF', fontsize=13, weight='bold', fontfamily='monospace')

# Static Elements: 2020 Rail & MGN12 Base
rail_y = -0.04
ax_anim.fill_between([-0.325, 0.325], rail_y - 0.02, rail_y, color='#475569', zorder=2) # 2020 Extrusion
ax_anim.fill_between([-0.325, 0.325], rail_y, rail_y + 0.008, color='#94a3b8', zorder=3) # MGN12 Rail

# Limit switches
ax_anim.scatter([-RAIL_LIMIT, RAIL_LIMIT], [rail_y + 0.015, rail_y + 0.015], color='#ef4444', s=90, zorder=4, marker='s')
ax_anim.axvline(-RAIL_LIMIT, color='#ef4444', linestyle='--', lw=1.2, alpha=0.4)
ax_anim.axvline(RAIL_LIMIT, color='#ef4444', linestyle='--', lw=1.2, alpha=0.4)

# Center Target Line
ax_anim.axvline(x=0, color='#10b981', linestyle='--', lw=1.5, alpha=0.7, zorder=1)

# Dynamic Objects
cart_w = 0.065
cart_h = 0.035
cart_base_y = rail_y + 0.008

cart_rect = Rectangle((-cart_w/2, cart_base_y), cart_w, cart_h, fc='#0284c7', ec='#38bdf8', lw=2, zorder=5)
ax_anim.add_patch(cart_rect)

enc_circle = Circle((0, cart_base_y + cart_h/2), 0.016, fc='#1e293b', ec='#f59e0b', lw=1.5, zorder=6)
ax_anim.add_patch(enc_circle)

pendulum_line, = ax_anim.plot([], [], '-', color='#f8fafc', lw=4.5, zorder=7, solid_capstyle='round')

tip_circle = Circle((0, 0), 0.012, fc='#e11d48', ec='#fecdd3', lw=2, zorder=8)
ax_anim.add_patch(tip_circle)

force_line, = ax_anim.plot([], [], color='#a855f7', lw=3.5, zorder=9, solid_capstyle='round')

# HUD text
bbox_props = dict(boxstyle="round,pad=0.5", fc="#020617", ec="#38bdf8", lw=1.2, alpha=0.92)
info_text = ax_anim.text(0.02, 0.96, '', transform=ax_anim.transAxes, fontsize=10, 
                         verticalalignment='top', bbox=bbox_props, family='monospace', color='#f8fafc')

# --- 4. OSCILLOSCOPES SETUP ---
# 1. Angle
line_th, = ax1.plot([], [], color='#FF5252', lw=2.0, label=r'$\theta(t)$')
ax1.axhline(0, color='#00E5FF', linestyle='--', alpha=0.8, label='Target (0°)')
ax1.axvline(3.0, color='#FFD700', linestyle=':', lw=1.8, label='Tap Disturbance (3.0s)')
ax1.set_xlim(0, T_MAX)
ax1.set_ylim(-16, 16)
ax1.set_title("1. Rod Angle: Rapid Settling (< 0.7s)", fontsize=10, color='#FF5252', weight='bold')
ax1.set_xlabel("Time (s)", color='#94a3b8', fontsize=9)
ax1.set_ylabel("Angle (deg)", color='#94a3b8', fontsize=9)
ax1.legend(loc='upper right', fontsize=8, framealpha=0.4)

# 2. Cart position
line_x, = ax2.plot([], [], color='#69F0AE', lw=2.0, label=r'$x(t)$')
ax2.axhline(+22.0, color='#FF1744', linestyle='--', alpha=0.6, label='Safety Limits (±22cm)')
ax2.axhline(-22.0, color='#FF1744', linestyle='--', alpha=0.6)
ax2.axhline(0, color='#94a3b8', linestyle=':', alpha=0.4)
ax2.set_xlim(0, T_MAX)
ax2.set_ylim(-25, 25)
ax2.set_title("2. Cart Linear Position x(t)", fontsize=10, color='#69F0AE', weight='bold')
ax2.set_xlabel("Time (s)", color='#94a3b8', fontsize=9)
ax2.set_ylabel("Position (cm)", color='#94a3b8', fontsize=9)
ax2.legend(loc='upper right', fontsize=8, framealpha=0.4)

# 3. Control Force
line_u, = ax3.plot([], [], color='#00E5FF', lw=2.0, label=r'$u(t)$')
ax3.axhline(+9.5, color='#FF9100', linestyle='--', alpha=0.6, label='NEMA 17 Limit (±9.5 N)')
ax3.axhline(-9.5, color='#FF9100', linestyle='--', alpha=0.6)
ax3.axhline(0, color='#94a3b8', linestyle=':', alpha=0.4)
ax3.set_xlim(0, T_MAX)
ax3.set_ylim(-12, 12)
ax3.set_title("3. Actuator Force u(t) (N)", fontsize=10, color='#00E5FF', weight='bold')
ax3.set_xlabel("Time (s)", color='#94a3b8', fontsize=9)
ax3.set_ylabel("Force (N)", color='#94a3b8', fontsize=9)
ax3.legend(loc='upper right', fontsize=8, framealpha=0.4)

# Trackers
dot_th, = ax1.plot([], [], 'o', color='#FF5252', markersize=5)
dot_x, = ax2.plot([], [], 'o', color='#69F0AE', markersize=5)
dot_u, = ax3.plot([], [], 'o', color='#00E5FF', markersize=5)

def update(frame):
    t_c = t_arr[frame]
    x_c = x_arr[frame]
    th_c_deg = th_deg_arr[frame]
    th_rad = np.radians(th_c_deg)
    w_c = w_arr[frame]
    u_c = u_arr[frame]
    
    # Geometric coordinates
    pivot_x = x_c
    pivot_y = cart_base_y + cart_h / 2
    top_x = pivot_x + L * np.sin(th_rad)
    top_y = pivot_y + L * np.cos(th_rad)
    
    # Update visuals
    cart_rect.set_x(x_c - cart_w / 2)
    enc_circle.center = (pivot_x, pivot_y)
    pendulum_line.set_data([pivot_x, top_x], [pivot_y, top_y])
    tip_circle.center = (top_x, top_y)
    
    # Force vector arrow
    f_len = u_c * 0.007
    force_line.set_data([pivot_x, pivot_x + f_len], [pivot_y, pivot_y])
    
    # Status mode string
    if 3.0 <= t_c < 3.8:
        mode_str = "REJECTING DISTURBANCE"
    elif t_c < 1.0:
        mode_str = "SETTLING INITIAL PERTURBATION"
    else:
        mode_str = "LQR STEADY-STATE BALANCE"
        
    info_txt = (
        f"⚡ LQR BENCHMARK SIMULATOR\n"
        f"STATE: {mode_str}\n"
        f"t    : {t_c:4.2f} s\n"
        f"θ    : {th_c_deg:+5.2f} °\n"
        f"ω    : {w_c:+5.2f} rad/s\n"
        f"x    : {x_c*100:+5.2f} cm\n"
        f"u    : {u_c:+5.2f} N"
    )
    info_text.set_text(info_txt)
    
    # Oscilloscopes
    sub_t = t_arr[:frame+1]
    line_th.set_data(sub_t, th_deg_arr[:frame+1])
    line_x.set_data(sub_t, x_arr[:frame+1] * 100.0)
    line_u.set_data(sub_t, u_arr[:frame+1])
    
    dot_th.set_data([t_c], [th_c_deg])
    dot_x.set_data([t_c], [x_c * 100.0])
    dot_u.set_data([t_c], [u_c])
    
    return [cart_rect, enc_circle, pendulum_line, tip_circle, force_line, info_text,
            line_th, line_x, line_u, dot_th, dot_x, dot_u]

OUTPUT_VIDEO = "/home/karim/Documents/pendulum_simulation_lqr_benchmark.mp4"
print(f"Rendering {N_FRAMES} frames to {OUTPUT_VIDEO} via FFMpegWriter...")

writer = animation.FFMpegWriter(fps=FPS, metadata=dict(artist='Karim Laiti'),
                                extra_args=['-vcodec', 'libx264', '-pix_fmt', 'yuv420p', '-preset', 'fast'])

ani = animation.FuncAnimation(fig, update, frames=N_FRAMES, blit=True)
ani.save(OUTPUT_VIDEO, writer=writer)
plt.close(fig)

print("Video rendered successfully!")
