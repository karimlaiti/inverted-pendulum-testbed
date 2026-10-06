#!/usr/bin/env python3
"""
High-Resolution Simulation Video Renderer: Inverted Pendulum on Linear Rail
Author: Karim Laiti (Sapienza University of Rome)
Generates a 1080p 60fps MP4 showcasing:
- Autonomous resonant swing-up from bottom rest (theta = pi)
- Dynamic handoff into 5-State LQR balance
- External disturbance rejection
- Synchronized multi-channel telemetry oscilloscope
"""

import os
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation

# Physical & Control Parameters
M = 0.380        # Cart mass [kg]
m = 0.020        # Pendulum mass [kg]
L = 0.200        # Pendulum full length [m]
l = 0.135        # Center of mass distance [m]
J = 0.0004933    # Pivot inertia [kg*m^2]
g = 9.81         # Gravity [m/s^2]
b_p = 0.0008     # Pivot friction [N*m*s/rad]

# LQR Gains (5-state: [theta, omega, x, v, integral_x])
K_theta = 62.43
K_omega = 9.50
K_x = 11.58
K_v = 12.09
K_xi = 1.41

MAX_A = 11.0
RAIL_LIMIT = 0.25 # m

# Simulation settings
FPS = 60
T_FINAL = 7.0
N_FRAMES = int(T_FINAL * FPS)
DT = 1.0 / FPS
SUBSTEPS = 20
DT_SUB = DT / SUBSTEPS

def normalize_angle(th):
    return (th + math.pi) % (2.0 * math.pi) - math.pi

# Pre-compute simulation trajectory
state = np.array([0.0, 0.0, 0.0, math.pi, 0.0]) # [xi, x, v, theta, omega]
mode = "SWING_UP"

t_hist = []
x_hist = []
v_hist = []
th_hist = []
w_hist = []
a_hist = []
mode_hist = []

for frame in range(N_FRAMES):
    t_curr = frame * DT
    
    # Disturbance tap at t = 4.8s (+2.2 rad/s)
    if 4.8 <= t_curr < 4.8 + DT:
        state[4] += 2.2
        
    for _ in range(SUBSTEPS):
        xi, x, v, th, w = state
        th_norm = normalize_angle(th)
        th_deg = abs(math.degrees(th_norm))
        
        # State machine transition
        if mode == "LQR_BALANCE":
            if th_deg > 28.0:
                mode = "SWING_UP"
                state[0] = 0.0
        else:
            if th_deg < 22.0 and abs(w) < 4.0:
                mode = "LQR_BALANCE"
                state[0] = 0.0
                
        # Control computation
        if mode == "LQR_BALANCE":
            a_cmd = (K_theta * th_norm + K_omega * w + K_x * x + K_v * v + K_xi * xi)
        else:
            if th_deg > 170.0 and abs(w) < 0.15:
                a_cmd = 3.5 if x <= 0 else -3.5
            else:
                sgn = 1.0 if (w * math.cos(th_norm) >= 0.0) else -1.0
                a_pump = -6.5 * sgn
                a_cmd = a_pump - 4.5 * x - 2.2 * v
                
        a_cmd = float(np.clip(a_cmd, -MAX_A, MAX_A))
        
        # RK4
        def deriv(s):
            _xi, _x, _v, _th, _w = s
            _sin = math.sin(_th)
            _cos = math.cos(_th)
            _w_dot = (m * g * l * _sin - b_p * _w - m * l * a_cmd * _cos) / J
            return np.array([_x, _v, a_cmd, _w, _w_dot])
            
        k1 = deriv(state)
        k2 = deriv(state + 0.5 * DT_SUB * k1)
        k3 = deriv(state + 0.5 * DT_SUB * k2)
        k4 = deriv(state + DT_SUB * k3)
        state = state + (DT_SUB / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
        state[0] = np.clip(state[0], -0.4, 0.4)
        
        if state[1] > RAIL_LIMIT:
            state[1] = RAIL_LIMIT
            state[2] = 0.0
        elif state[1] < -RAIL_LIMIT:
            state[1] = -RAIL_LIMIT
            state[2] = 0.0

    t_hist.append(t_curr)
    x_hist.append(state[1])
    v_hist.append(state[2])
    th_hist.append(normalize_angle(state[3]))
    w_hist.append(state[4])
    a_hist.append(a_cmd)
    mode_hist.append(mode)

t_hist = np.array(t_hist)
x_hist = np.array(x_hist)
th_hist = np.array(th_hist)
w_hist = np.array(w_hist)
a_hist = np.array(a_hist)

print(f"Simulation completed: {N_FRAMES} frames ({T_FINAL}s). Preparing animation...")

# Setup Figure & Axes
plt.style.use('dark_background')
fig = plt.figure(figsize=(16, 9), dpi=120)
fig.patch.set_facecolor('#0d1117')

# Grid layout: Top = Physical Animation (60%), Bottom = 3 Oscilloscopes (40%)
gs = fig.add_gridspec(2, 3, height_ratios=[1.3, 0.9], hspace=0.35, wspace=0.25,
                       left=0.06, right=0.96, top=0.92, bottom=0.08)

ax_anim = fig.add_subplot(gs[0, :])
ax_th = fig.add_subplot(gs[1, 0])
ax_x = fig.add_subplot(gs[1, 1])
ax_a = fig.add_subplot(gs[1, 2])

for ax in [ax_th, ax_x, ax_a]:
    ax.set_facecolor('#161b22')
    ax.grid(True, color='#30363d', linestyle='--', alpha=0.6)
    ax.tick_params(colors='#8b949e', labelsize=9)
    for spine in ax.spines.values():
        spine.set_color('#30363d')

# Setup Animation Axis
ax_anim.set_facecolor('#161b22')
ax_anim.set_xlim(-0.35, 0.35)
ax_anim.set_ylim(-0.28, 0.32)
ax_anim.set_aspect('equal')
ax_anim.axis('off')

# Static elements: Rail
rail_rect = patches.Rectangle((-0.32, -0.015), 0.64, 0.03, facecolor='#21262d', edgecolor='#484f58', lw=1.5, zorder=1)
ax_anim.add_patch(rail_rect)

# Limit stops
ax_anim.plot([-0.30, -0.30], [-0.04, 0.04], color='#f85149', lw=3, zorder=2)
ax_anim.plot([0.30, 0.30], [-0.04, 0.04], color='#f85149', lw=3, zorder=2)

# Dynamic elements in animation
cart_patch = patches.Rectangle((-0.035, -0.02), 0.07, 0.04, facecolor='#1f6feb', edgecolor='#58a6ff', lw=2, zorder=3)
ax_anim.add_patch(cart_patch)

rod_line, = ax_anim.plot([], [], color='#f0f6fc', lw=3.5, solid_capstyle='round', zorder=4)
pivot_dot, = ax_anim.plot([], [], 'o', color='#58a6ff', markersize=7, zorder=5)
tip_dot, = ax_anim.plot([], [], 'o', color='#3fb950', markersize=9, zorder=5)

# Text HUD
title_txt = ax_anim.text(0.0, 0.28, "INVERTED PENDULUM TESTBED: NONLINEAR SIMULATION BENCHMARK", 
                         ha='center', va='center', color='#f0f6fc', fontsize=13, weight='bold', fontfamily='monospace')
status_badge = ax_anim.text(-0.32, 0.24, "MODE: SWING_UP", ha='left', va='center',
                            color='#d29922', fontsize=11, weight='bold', fontfamily='monospace',
                            bbox=dict(boxstyle='round,pad=0.4', facecolor='#2d2206', edgecolor='#d29922', lw=1.5))
telemetry_txt = ax_anim.text(0.32, 0.24, "", ha='right', va='center',
                             color='#8b949e', fontsize=10, fontfamily='monospace')

# Telemetry plots setup
line_th, = ax_th.plot([], [], color='#58a6ff', lw=1.8, label=r'$\theta$ (deg)')
ax_th.axhline(0, color='#3fb950', linestyle=':', alpha=0.7)
ax_th.axhspan(-22, 22, color='#3fb950', alpha=0.08, label='Catch Cone')
ax_th.set_xlim(0, T_FINAL)
ax_th.set_ylim(-190, 190)
ax_th.set_title("Rod Angle θ [deg]", color='#58a6ff', fontsize=10, weight='bold')
ax_th.set_xlabel("Time [s]", color='#8b949e', fontsize=9)

line_x, = ax_x.plot([], [], color='#3fb950', lw=1.8, label='x (cm)')
ax_x.axhline(0, color='#8b949e', linestyle=':', alpha=0.5)
ax_x.axhline(-25, color='#f85149', linestyle='--', alpha=0.5)
ax_x.axhline(25, color='#f85149', linestyle='--', alpha=0.5)
ax_x.set_xlim(0, T_FINAL)
ax_x.set_ylim(-30, 30)
ax_x.set_title("Cart Position x [cm]", color='#3fb950', fontsize=10, weight='bold')
ax_x.set_xlabel("Time [s]", color='#8b949e', fontsize=9)

line_a, = ax_a.plot([], [], color='#e3b341', lw=1.8, label='a_cmd (m/s²)')
ax_a.axhline(0, color='#8b949e', linestyle=':', alpha=0.5)
ax_a.set_xlim(0, T_FINAL)
ax_a.set_ylim(-13, 13)
ax_a.set_title("Cart Acceleration [m/s²]", color='#e3b341', fontsize=10, weight='bold')
ax_a.set_xlabel("Time [s]", color='#8b949e', fontsize=9)

# Cursor points on plots
cursor_th, = ax_th.plot([], [], 'o', color='#58a6ff', markersize=5)
cursor_x, = ax_x.plot([], [], 'o', color='#3fb950', markersize=5)
cursor_a, = ax_a.plot([], [], 'o', color='#e3b341', markersize=5)

def update(frame):
    t_c = t_hist[frame]
    x_c = x_hist[frame]
    th_c = th_hist[frame]
    w_c = w_hist[frame]
    a_c = a_hist[frame]
    m_c = mode_hist[frame]
    
    # Update cart patch
    cart_patch.set_x(x_c - 0.035)
    
    # Compute pendulum rod coordinates (theta=0 is upright)
    tip_x = x_c + L * math.sin(th_c)
    tip_y = L * math.cos(th_c)
    
    rod_line.set_data([x_c, tip_x], [0.0, tip_y])
    pivot_dot.set_data([x_c], [0.0])
    tip_dot.set_data([tip_x], [tip_y])
    
    # Color badge
    if m_c == "LQR_BALANCE":
        status_badge.set_text("MODE: 5-STATE LQR BALANCE")
        status_badge.set_color('#3fb950')
        status_badge.set_bbox(dict(boxstyle='round,pad=0.4', facecolor='#0d2818', edgecolor='#3fb950', lw=1.5))
        tip_dot.set_color('#3fb950')
    else:
        status_badge.set_text("MODE: LYAPUNOV SWING-UP")
        status_badge.set_color('#d29922')
        status_badge.set_bbox(dict(boxstyle='round,pad=0.4', facecolor='#2d2206', edgecolor='#d29922', lw=1.5))
        tip_dot.set_color('#d29922')
        
    th_deg = math.degrees(th_c)
    telemetry_txt.set_text(f"θ: {th_deg:+6.1f}° | ω: {w_c:+5.1f} rad/s\nx: {x_c*100:+5.1f} cm | a: {a_c:+5.1f} m/s²")
    
    # Update oscilloscope lines
    sub_t = t_hist[:frame+1]
    line_th.set_data(sub_t, np.degrees(th_hist[:frame+1]))
    line_x.set_data(sub_t, x_hist[:frame+1] * 100.0)
    line_a.set_data(sub_t, a_hist[:frame+1])
    
    cursor_th.set_data([t_c], [th_deg])
    cursor_x.set_data([t_c], [x_c * 100.0])
    cursor_a.set_data([t_c], [a_c])
    
    return [cart_patch, rod_line, pivot_dot, tip_dot, status_badge, telemetry_txt,
            line_th, line_x, line_a, cursor_th, cursor_x, cursor_a]

print("Rendering video via FFMpegWriter (1080p @ 60fps)...")
OUTPUT_PATH = "/home/karim/Documents/pendulum_simulation_demo.mp4"
REPO_OUTPUT = "/home/karim/Documents/Projects/inverted-pendulum-testbed/simulation/pendulum_simulation_demo.mp4"

writer = animation.FFMpegWriter(fps=FPS, metadata=dict(artist='Karim Laiti'), 
                                extra_args=['-vcodec', 'libx264', '-pix_fmt', 'yuv420p', '-preset', 'fast'])

ani = animation.FuncAnimation(fig, update, frames=N_FRAMES, blit=True)
ani.save(OUTPUT_PATH, writer=writer)
plt.close(fig)

# Copy to repo simulation folder
import shutil
shutil.copy2(OUTPUT_PATH, REPO_OUTPUT)

print(f"SUCCESS: Video saved to {OUTPUT_PATH} and {REPO_OUTPUT}")
