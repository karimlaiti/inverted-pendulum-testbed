#!/usr/bin/env python3
"""
Complete Swing-Up from Bottom + 5-State LQR Balance Simulation Video
Author: Karim Laiti (Sapienza University of Rome)
Starts at bottom at rest (theta = 180 deg / pi rad).
Performs resonant Lyapunov energy swing-up.
Catches into LQR at vertical (theta = 0 deg).
Stabilizes cart at x = 0 cm with zero steady-state error.
Renders 1080p 60fps MP4 with glitch-free Matplotlib curves.
"""

import sys, os, math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
import matplotlib.animation as animation

# Physical & Control Parameters from interactive_gui.py
M = 0.380        # Cart mass [kg]
m = 0.020        # Pendulum mass [kg]
L = 0.200        # Pendulum length [m]
l = 0.135        # COM distance [m]
J = 0.0004933    # Pivot inertia [kg*m^2]
g = 9.81         # Gravity [m/s^2]
b_p = 0.0008     # Pivot damping
RAIL_LIMIT = 0.28 # Safety rail limit

# LQR gains
K_theta = 62.43
K_omega = 9.50
K_x = 11.58
K_v = 12.09
K_xi = 1.41
MAX_A = 12.0

# 60 FPS Simulation
FPS = 60
T_TOTAL = 6.5
N_FRAMES = int(T_TOTAL * FPS)
dt = 1.0 / FPS

# System state: [x_integral, x, v, theta, omega]
# START AT BOTTOM AT REST (theta = pi, omega = 0)
state = np.array([0.0, 0.0, 0.0, math.pi, 0.0], dtype=float)
mode = "SWING_UP"

def normalize_angle(theta: float) -> float:
    return (theta + math.pi) % (2.0 * math.pi) - math.pi

# Trajectory data buffers
t_arr = np.zeros(N_FRAMES)
x_arr = np.zeros(N_FRAMES)
v_arr = np.zeros(N_FRAMES)
th_raw_arr = np.zeros(N_FRAMES)
th_deg_arr = np.zeros(N_FRAMES)
w_arr = np.zeros(N_FRAMES)
a_arr = np.zeros(N_FRAMES)
e_arr = np.zeros(N_FRAMES)
mode_arr = []

# Pre-compute physics using substepped RK4
for frame in range(N_FRAMES):
    t_curr = frame * dt
    
    # 1. Compute control
    xi, x, v, th, w = state
    th_norm = normalize_angle(th)
    th_deg = abs(math.degrees(th_norm))
    e_pot = m * g * l * (math.cos(th_norm) - 1.0)
    e_kin = 0.5 * J * (w ** 2)
    e_total = e_kin + e_pot
    
    # Supervisor logic with catch condition
    if mode == "LQR_BALANCE":
        if th_deg > 28.0:
            mode = "SWING_UP"
            state[0] = 0.0
    else:
        if th_deg < 22.0 and abs(w) < 4.0:
            mode = "LQR_BALANCE"
            state[0] = 0.0 # reset integral on catch
            
    if mode == "LQR_BALANCE":
        a_cmd = (K_theta * th_norm + K_omega * w + K_x * x + K_v * v + K_xi * xi)
    else:
        if th_deg > 170.0 and abs(w) < 0.15:
            a_cmd = 3.5 if x <= 0 else -3.5
        else:
            sgn = 1.0 if (w * math.cos(th_norm) >= 0.0) else -1.0
            a_pump = - 6.5 * sgn
            a_cmd = a_pump - 4.5 * x - 2.2 * v
            
    a_cmd = float(np.clip(a_cmd, -MAX_A, MAX_A))
    
    # Substepped RK4
    substeps = 8
    dt_sub = dt / substeps
    for _ in range(substeps):
        def deriv(s):
            _xi, _x, _v, _th, _w = s
            _sin = math.sin(_th); _cos = math.cos(_th)
            _w_dot = (m * g * l * _sin - b_p * _w - m * l * a_cmd * _cos) / J
            return np.array([_x, _v, a_cmd, _w, _w_dot])
            
        s = state
        k1 = deriv(s)
        k2 = deriv(s + 0.5 * dt_sub * k1)
        k3 = deriv(s + 0.5 * dt_sub * k2)
        k4 = deriv(s + dt_sub * k3)
        state = s + (dt_sub / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
        state[0] = np.clip(state[0], -0.4, 0.4)
        if state[1] > RAIL_LIMIT:
            state[1] = RAIL_LIMIT; state[2] = 0.0
        elif state[1] < -RAIL_LIMIT:
            state[1] = -RAIL_LIMIT; state[2] = 0.0

    t_arr[frame] = t_curr
    x_arr[frame] = state[1]
    v_arr[frame] = state[2]
    th_raw_arr[frame] = state[3]
    th_deg_arr[frame] = math.degrees(normalize_angle(state[3]))
    w_arr[frame] = state[4]
    a_arr[frame] = a_cmd
    e_arr[frame] = e_total
    mode_arr.append(mode)

print("Pre-computation completed! Preparing Matplotlib 1080p 60fps layout...")

# Prepare clean, non-glitching theta curve for plotting:
# Insert NaN where the angle crosses the +-180 branch cut to prevent vertical line artifacts
th_clean_deg = np.copy(th_deg_arr)
for i in range(1, len(th_clean_deg)):
    if abs(th_deg_arr[i] - th_deg_arr[i-1]) > 140.0:
        th_clean_deg[i] = np.nan

# --- 2. ADVANCED MATPLOTLIB DASHBOARD SETUP ---
plt.style.use('dark_background')
fig = plt.figure(figsize=(16, 9), dpi=120)
fig.patch.set_facecolor('#090d16')

# Grid: Top 58% = Physical Animation, Bottom 42% = 3 Oscilloscopes
gs = fig.add_gridspec(2, 3, height_ratios=[1.3, 0.9], hspace=0.32, wspace=0.25,
                       left=0.06, right=0.96, top=0.93, bottom=0.08)

ax_anim = fig.add_subplot(gs[0, :])
ax1 = fig.add_subplot(gs[1, 0])
ax2 = fig.add_subplot(gs[1, 1])
ax3 = fig.add_subplot(gs[1, 2])

# Styling plots
for ax in [ax1, ax2, ax3]:
    ax.set_facecolor('#111827')
    ax.grid(True, color='#374151', linestyle=':', alpha=0.6)
    ax.tick_params(colors='#9ca3af', labelsize=9)
    for spine in ax.spines.values():
        spine.set_color('#374151')

# Setup Physical Animation Axes
ax_anim.set_facecolor('#0f172a')
ax_anim.set_xlim(-0.35, 0.35)
ax_anim.set_ylim(-0.27, 0.30)
ax_anim.set_aspect('equal')
ax_anim.axis('off')

# Title
ax_anim.text(0.0, 0.28, "INVERTED PENDULUM: AUTONOMOUS SWING-UP (FROM REST) & 5-STATE LQR", 
             ha='center', va='center', color='#38bdf8', fontsize=13, weight='bold', fontfamily='monospace')

# 2020 Aluminum Rail & MGN12 Profile
rail_y = 0.0
ax_anim.fill_between([-0.325, 0.325], rail_y - 0.025, rail_y, color='#334155', zorder=2) # 2020 extrusion
ax_anim.fill_between([-0.325, 0.325], rail_y, rail_y + 0.008, color='#94a3b8', zorder=3) # MGN12 rail

# Limit switches (red stoppers)
ax_anim.scatter([-0.25, 0.25], [rail_y + 0.015, rail_y + 0.015], color='#ef4444', s=100, zorder=4, marker='s')
ax_anim.axvline(-0.25, color='#ef4444', linestyle='--', lw=1.2, alpha=0.4)
ax_anim.axvline(0.25, color='#ef4444', linestyle='--', lw=1.2, alpha=0.4)

# Center Target (Emerald Dashed Line)
ax_anim.axvline(x=0, color='#10b981', linestyle='--', lw=1.5, alpha=0.7, zorder=1)

# Cart
cart_w = 0.065
cart_h = 0.035
cart_base_y = rail_y + 0.008

cart_rect = Rectangle((-cart_w/2, cart_base_y), cart_w, cart_h, fc='#0284c7', ec='#38bdf8', lw=2, zorder=5)
ax_anim.add_patch(cart_rect)

# Encoder body on cart
enc_circle = Circle((0, cart_base_y + cart_h/2), 0.016, fc='#1e293b', ec='#f59e0b', lw=1.5, zorder=6)
ax_anim.add_patch(enc_circle)

# Pendulum Rod (Carbon Fiber)
pendulum_line, = ax_anim.plot([], [], '-', color='#f8fafc', lw=4.5, zorder=7, solid_capstyle='round')

# Tip Mass (Brass weight)
tip_circle = Circle((0, 0), 0.013, fc='#e11d48', ec='#fecdd3', lw=2, zorder=8)
ax_anim.add_patch(tip_circle)

# Force vector arrow
force_line, = ax_anim.plot([], [], color='#a855f7', lw=3.5, zorder=9, solid_capstyle='round')

# Status Mode Badge in top left
status_badge = ax_anim.text(-0.33, 0.23, "MODE: LYAPUNOV SWING-UP", ha='left', va='center',
                            color='#f59e0b', fontsize=11, weight='bold', fontfamily='monospace',
                            bbox=dict(boxstyle='round,pad=0.4', facecolor='#2d2206', edgecolor='#f59e0b', lw=1.5))

# HUD Text in top right
bbox_hud = dict(boxstyle="round,pad=0.5", fc="#020617", ec="#38bdf8", lw=1.2, alpha=0.92)
info_text = ax_anim.text(0.33, 0.23, '', ha='right', va='center', fontsize=10, 
                         bbox=bbox_hud, family='monospace', color='#f8fafc')

# --- 3. OSCILLOSCOPES SETUP ---
# 1. Rod Angle theta
line_th, = ax1.plot([], [], color='#38bdf8', lw=2.0, label=r'$\theta(t)$')
ax1.axhline(0, color='#10b981', linestyle='--', alpha=0.8, label='Upright Target (0°)')
ax1.axhline(180, color='#f59e0b', linestyle=':', alpha=0.5, label='Bottom Rest (±180°)')
ax1.axhline(-180, color='#f59e0b', linestyle=':', alpha=0.5)
ax1.axhspan(-22, 22, color='#10b981', alpha=0.12, label='LQR Catch Cone (±22°)')
ax1.set_xlim(0, T_TOTAL)
ax1.set_ylim(-195, 195)
ax1.set_title("1. Rod Angle θ(t) [deg]: 180° -> 0°", fontsize=10, color='#38bdf8', weight='bold')
ax1.set_xlabel("Time (s)", color='#9ca3af', fontsize=9)
ax1.set_ylabel("Angle (deg)", color='#9ca3af', fontsize=9)
ax1.legend(loc='upper right', fontsize=8, framealpha=0.4)

# 2. Cart Linear Position x
line_x, = ax2.plot([], [], color='#10b981', lw=2.0, label=r'$x(t)$')
ax2.axhline(+25.0, color='#ef4444', linestyle='--', alpha=0.7, label='Safety Limits (±25 cm)')
ax2.axhline(-25.0, color='#ef4444', linestyle='--', alpha=0.7)
ax2.axhline(0, color='#9ca3af', linestyle=':', alpha=0.4, label='Center (x = 0)')
ax2.set_xlim(0, T_TOTAL)
ax2.set_ylim(-30, 30)
ax2.set_title("2. Cart Position x(t) [cm]", fontsize=10, color='#10b981', weight='bold')
ax2.set_xlabel("Time (s)", color='#9ca3af', fontsize=9)
ax2.set_ylabel("Position (cm)", color='#9ca3af', fontsize=9)
ax2.legend(loc='upper right', fontsize=8, framealpha=0.4)

# 3. Mechanical Energy E_total
line_e, = ax3.plot([], [], color='#a855f7', lw=2.0, label=r'$E_{mech}(t)$')
ax3.axhline(0, color='#10b981', linestyle='--', alpha=0.8, label='Target Energy E=0 (Upright)')
E_bottom = - 2.0 * m * g * l
ax3.axhline(E_bottom, color='#f59e0b', linestyle=':', alpha=0.6, label=f'Bottom Energy ({E_bottom:.3f} J)')
ax3.set_xlim(0, T_TOTAL)
ax3.set_ylim(-0.065, 0.015)
ax3.set_title("3. Total Mechanical Energy E(t) [J]", fontsize=10, color='#a855f7', weight='bold')
ax3.set_xlabel("Time (s)", color='#9ca3af', fontsize=9)
ax3.set_ylabel("Energy (J)", color='#9ca3af', fontsize=9)
ax3.legend(loc='lower right', fontsize=8, framealpha=0.4)

# Trackers
dot_th, = ax1.plot([], [], 'o', color='#38bdf8', markersize=5)
dot_x, = ax2.plot([], [], 'o', color='#10b981', markersize=5)
dot_e, = ax3.plot([], [], 'o', color='#a855f7', markersize=5)

def update(frame):
    t_c = t_arr[frame]
    x_c = x_arr[frame]
    th_c_rad = th_raw_arr[frame]
    th_c_deg = th_deg_arr[frame]
    w_c = w_arr[frame]
    a_c = a_arr[frame]
    e_c = e_arr[frame]
    m_c = mode_arr[frame]
    
    # Geometric coordinates (pivot is on cart)
    pivot_x = x_c
    pivot_y = cart_base_y + cart_h / 2
    top_x = pivot_x + L * math.sin(th_c_rad)
    top_y = pivot_y + L * math.cos(th_c_rad)
    
    # Update visuals
    cart_rect.set_x(x_c - cart_w / 2)
    enc_circle.center = (pivot_x, pivot_y)
    pendulum_line.set_data([pivot_x, top_x], [pivot_y, top_y])
    tip_circle.center = (top_x, top_y)
    
    # Force vector arrow
    f_len = a_c * 0.007
    force_line.set_data([pivot_x, pivot_x + f_len], [pivot_y, pivot_y])
    
    # Update Mode Badge
    if m_c == "LQR_BALANCE":
        status_badge.set_text("MODE: 5-STATE LQR BALANCE")
        status_badge.set_color('#10b981')
        status_badge.set_bbox(dict(boxstyle='round,pad=0.4', facecolor='#064e3b', edgecolor='#10b981', lw=1.5))
        tip_circle.set_facecolor('#10b981')
    else:
        status_badge.set_text("MODE: LYAPUNOV SWING-UP")
        status_badge.set_color('#f59e0b')
        status_badge.set_bbox(dict(boxstyle='round,pad=0.4', facecolor='#2d2206', edgecolor='#f59e0b', lw=1.5))
        tip_circle.set_facecolor('#e11d48')
        
    info_txt = (
        f"t : {t_c:4.2f}s | θ: {th_c_deg:+5.1f}°\n"
        f"x : {x_c*100:+5.1f}cm | E: {e_c:+5.3f}J\n"
        f"a : {a_c:+5.1f}m/s²"
    )
    info_text.set_text(info_txt)
    
    # Oscilloscopes progressive data
    sub_t = t_arr[:frame+1]
    line_th.set_data(sub_t, th_clean_deg[:frame+1])
    line_x.set_data(sub_t, x_arr[:frame+1] * 100.0)
    line_e.set_data(sub_t, e_arr[:frame+1])
    
    dot_th.set_data([t_c], [th_c_deg])
    dot_x.set_data([t_c], [x_c * 100.0])
    dot_e.set_data([t_c], [e_c])
    
    return [cart_rect, enc_circle, pendulum_line, tip_circle, force_line, status_badge, info_text,
            line_th, line_x, line_e, dot_th, dot_x, dot_e]

OUTPUT_VIDEO = "/home/karim/Documents/pendulum_swingup_lqr_simulation.mp4"
print(f"Rendering {N_FRAMES} frames (T={T_TOTAL}s) to {OUTPUT_VIDEO} via FFMpegWriter (60fps)...")

writer = animation.FFMpegWriter(fps=FPS, metadata=dict(artist='Karim Laiti'),
                                extra_args=['-vcodec', 'libx264', '-pix_fmt', 'yuv420p', '-preset', 'fast'])

ani = animation.FuncAnimation(fig, update, frames=N_FRAMES, blit=True)
ani.save(OUTPUT_VIDEO, writer=writer)
plt.close(fig)

print("SUCCESS: Swing-up simulation video rendered perfectly!")
