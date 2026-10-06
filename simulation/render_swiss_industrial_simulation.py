#!/usr/bin/env python3
"""
Publication-Grade Inverted Pendulum Simulation Video (Swiss / IEEE Academic Style)
Author: Karim Laiti (Sapienza University of Rome)
Design Read: High-end scientific engineering visualization, pure white/slate palette,
zero neon, zero gaming slop, realistic material tones, rigorous typography.
"""

import sys, os, math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
import matplotlib.animation as animation

# --- 1. PHYSICAL & SIMULATION PARAMETERS ---
M = 0.380        # Cart mass [kg]
m = 0.020        # Pendulum mass [kg]
L = 0.200        # Pendulum length [m]
l = 0.135        # COM distance [m]
J = 0.0004933    # Pivot inertia [kg*m^2]
g = 9.81         # Gravity [m/s^2]
b_p = 0.0008     # Pivot damping
RAIL_LIMIT = 0.28

K_theta = 62.43
K_omega = 9.50
K_x = 11.58
K_v = 12.09
K_xi = 1.41
MAX_A = 12.0

FPS = 60
T_TOTAL = 6.5
N_FRAMES = int(T_TOTAL * FPS)
dt = 1.0 / FPS

state = np.array([0.0, 0.0, 0.0, math.pi, 0.0], dtype=float)
mode = "SWING_UP"

def normalize_angle(theta: float) -> float:
    return (theta + math.pi) % (2.0 * math.pi) - math.pi

t_arr = np.zeros(N_FRAMES)
x_arr = np.zeros(N_FRAMES)
th_raw_arr = np.zeros(N_FRAMES)
th_deg_arr = np.zeros(N_FRAMES)
w_arr = np.zeros(N_FRAMES)
a_arr = np.zeros(N_FRAMES)
e_arr = np.zeros(N_FRAMES)
mode_arr = []

# Pre-compute physics using substepped RK4
for frame in range(N_FRAMES):
    t_curr = frame * dt
    xi, x, v, th, w = state
    th_norm = normalize_angle(th)
    th_deg = abs(math.degrees(th_norm))
    e_pot = m * g * l * (math.cos(th_norm) - 1.0)
    e_kin = 0.5 * J * (w ** 2)
    e_total = e_kin + e_pot
    
    if mode == "LQR_BALANCE":
        if th_deg > 28.0:
            mode = "SWING_UP"
            state[0] = 0.0
    else:
        if th_deg < 22.0 and abs(w) < 4.0:
            mode = "LQR_BALANCE"
            state[0] = 0.0
            
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
    
    substeps = 8
    dt_sub = dt / substeps
    for _ in range(substeps):
        def deriv(s):
            _xi, _x, _v, _th, _w = s
            _sin = math.sin(_th); _cos = math.cos(_th)
            _w_dot = (m * g * l * _sin - b_p * _w - m * l * a_cmd * _cos) / J
            return np.array([_x, _v, a_cmd, _w, _w_dot])
        s = state
        k1 = deriv(s); k2 = deriv(s + 0.5 * dt_sub * k1); k3 = deriv(s + 0.5 * dt_sub * k2); k4 = deriv(s + dt_sub * k3)
        state = s + (dt_sub / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
        state[0] = np.clip(state[0], -0.4, 0.4)
        if state[1] > RAIL_LIMIT:
            state[1] = RAIL_LIMIT; state[2] = 0.0
        elif state[1] < -RAIL_LIMIT:
            state[1] = -RAIL_LIMIT; state[2] = 0.0

    t_arr[frame] = t_curr
    x_arr[frame] = state[1]
    th_raw_arr[frame] = state[3]
    th_deg_arr[frame] = math.degrees(normalize_angle(state[3]))
    w_arr[frame] = state[4]
    a_arr[frame] = a_cmd
    e_arr[frame] = e_total
    mode_arr.append(mode)

# Clean discontinuous jumps for angle plot
th_clean_deg = np.copy(th_deg_arr)
for i in range(1, len(th_clean_deg)):
    if abs(th_deg_arr[i] - th_deg_arr[i-1]) > 140.0:
        th_clean_deg[i] = np.nan

# --- 2. SOBER SCIENTIFIC VISUALIZATION (IEEE/SWISS STYLE) ---
# White / Clean Off-White Palette
fig = plt.figure(figsize=(16, 9), dpi=120)
fig.patch.set_facecolor('#F8FAFC') # Crisp technical paper white

gs = fig.add_gridspec(2, 3, height_ratios=[1.3, 0.9], hspace=0.36, wspace=0.25,
                       left=0.06, right=0.96, top=0.92, bottom=0.08)

ax_anim = fig.add_subplot(gs[0, :])
ax1 = fig.add_subplot(gs[1, 0])
ax2 = fig.add_subplot(gs[1, 1])
ax3 = fig.add_subplot(gs[1, 2])

# Subdued sober styling for plots
for ax in [ax1, ax2, ax3]:
    ax.set_facecolor('#FFFFFF')
    ax.grid(True, color='#E2E8F0', linestyle='-', linewidth=0.8)
    ax.tick_params(colors='#475569', labelsize=9)
    for spine in ax.spines.values():
        spine.set_color('#CBD5E1')
        spine.set_linewidth(1.0)

# Physical Animation Axes
ax_anim.set_facecolor('#FFFFFF')
ax_anim.set_xlim(-0.35, 0.35)
ax_anim.set_ylim(-0.27, 0.30)
ax_anim.set_aspect('equal')
ax_anim.axis('off')

# Title (Sober Technical Typography)
ax_anim.text(0.0, 0.28, "LINEAR INVERTED PENDULUM: NONLINEAR SWING-UP & LQR REGULATION", 
             ha='center', va='center', color='#0F172A', fontsize=12, weight='bold', fontfamily='sans-serif')

# Realistic Mechanical Elements
rail_y = 0.0
# 2020 Extrusion (Anodized matte grey)
ax_anim.fill_between([-0.325, 0.325], rail_y - 0.022, rail_y, color='#94A3B8', zorder=2)
# MGN12 Ground Steel Rail
ax_anim.fill_between([-0.325, 0.325], rail_y, rail_y + 0.006, color='#CBD5E1', zorder=3)

# Limit stoppers (Muted industrial crimson)
ax_anim.scatter([-0.25, 0.25], [rail_y + 0.012, rail_y + 0.012], color='#B91C1C', s=70, zorder=4, marker='s')
ax_anim.axvline(-0.25, color='#B91C1C', linestyle=':', lw=1.0, alpha=0.5)
ax_anim.axvline(0.25, color='#B91C1C', linestyle=':', lw=1.0, alpha=0.5)

# Center Target Reference Line (Subtle slate dashed)
ax_anim.axvline(x=0, color='#64748B', linestyle='--', lw=1.2, alpha=0.6, zorder=1)

# Cart (Black anodized aluminum MGN12 carriage block)
cart_w = 0.065
cart_h = 0.032
cart_base_y = rail_y + 0.006

cart_rect = Rectangle((-cart_w/2, cart_base_y), cart_w, cart_h, fc='#1E293B', ec='#0F172A', lw=1.5, zorder=5)
ax_anim.add_patch(cart_rect)

# Encoder hub (Muted steel/graphite)
enc_circle = Circle((0, cart_base_y + cart_h/2), 0.015, fc='#334155', ec='#64748B', lw=1.2, zorder=6)
ax_anim.add_patch(enc_circle)

# Carbon Fiber Rod (Matte graphite black)
pendulum_line, = ax_anim.plot([], [], '-', color='#09090B', lw=4.0, zorder=7, solid_capstyle='round')

# Brass Tip Mass (Natural muted brass/ochre)
tip_circle = Circle((0, 0), 0.012, fc='#B45309', ec='#78350F', lw=1.5, zorder=8)
ax_anim.add_patch(tip_circle)

# Force Vector Arrow (Muted cobalt blue)
force_line, = ax_anim.plot([], [], color='#2563EB', lw=3.0, zorder=9, solid_capstyle='round')

# Status Badge (Minimalist technical label, no rounded neon pills)
status_badge = ax_anim.text(-0.33, 0.23, "STATE: ENERGY SWING-UP", ha='left', va='center',
                            color='#92400E', fontsize=10, weight='bold', fontfamily='monospace',
                            bbox=dict(boxstyle='square,pad=0.3', facecolor='#FEF3C7', edgecolor='#D97706', lw=1.0))

# HUD Text (Muted Slate)
bbox_hud = dict(boxstyle="square,pad=0.4", facecolor='#F1F5F9', edgecolor='#CBD5E1', lw=1.0)
info_text = ax_anim.text(0.33, 0.23, '', ha='right', va='center', fontsize=9.5, 
                         bbox=bbox_hud, family='monospace', color='#1E293B')

# --- 3. PLOTS SETUP (SOBER ENGINEERING CURVES) ---
# 1. Rod Angle (Deep Navy #1E3A8A)
line_th, = ax1.plot([], [], color='#1E40AF', lw=1.8, label=r'$\theta(t)$')
ax1.axhline(0, color='#047857', linestyle='--', lw=1.2, alpha=0.8, label='Target (0°)')
ax1.axhline(180, color='#94A3B8', linestyle=':', lw=1.0, alpha=0.7, label='Rest (±180°)')
ax1.axhline(-180, color='#94A3B8', linestyle=':', lw=1.0, alpha=0.7)
ax1.axhspan(-22, 22, color='#047857', alpha=0.08, label='LQR Basin (±22°)')
ax1.set_xlim(0, T_TOTAL)
ax1.set_ylim(-195, 195)
ax1.set_title("Rod Angle θ(t) [deg]", fontsize=10, color='#0F172A', weight='bold')
ax1.set_xlabel("Time (s)", color='#475569', fontsize=9)
ax1.set_ylabel("Angle (deg)", color='#475569', fontsize=9)
ax1.legend(loc='upper right', fontsize=8, framealpha=0.8, facecolor='#FFFFFF', edgecolor='#E2E8F0')

# 2. Cart Linear Position (Forest Green #047857)
line_x, = ax2.plot([], [], color='#047857', lw=1.8, label=r'$x(t)$')
ax2.axhline(+25.0, color='#B91C1C', linestyle='--', lw=1.0, alpha=0.7, label='Rail Limits (±25 cm)')
ax2.axhline(-25.0, color='#B91C1C', linestyle='--', lw=1.0, alpha=0.7)
ax2.axhline(0, color='#94A3B8', linestyle=':', lw=1.0, alpha=0.6, label='Center (x = 0)')
ax2.set_xlim(0, T_TOTAL)
ax2.set_ylim(-30, 30)
ax2.set_title("Cart Position x(t) [cm]", fontsize=10, color='#0F172A', weight='bold')
ax2.set_xlabel("Time (s)", color='#475569', fontsize=9)
ax2.set_ylabel("Position (cm)", color='#475569', fontsize=9)
ax2.legend(loc='upper right', fontsize=8, framealpha=0.8, facecolor='#FFFFFF', edgecolor='#E2E8F0')

# 3. Total Mechanical Energy (Burgundy/Slate #991B1B)
line_e, = ax3.plot([], [], color='#991B1B', lw=1.8, label=r'$E_{mech}(t)$')
ax3.axhline(0, color='#047857', linestyle='--', lw=1.2, alpha=0.8, label='Upright Energy E=0')
E_bottom = - 2.0 * m * g * l
ax3.axhline(E_bottom, color='#94A3B8', linestyle=':', lw=1.0, alpha=0.7, label=f'Rest Energy ({E_bottom:.3f} J)')
ax3.set_xlim(0, T_TOTAL)
ax3.set_ylim(-0.065, 0.015)
ax3.set_title("Mechanical Energy E(t) [J]", fontsize=10, color='#0F172A', weight='bold')
ax3.set_xlabel("Time (s)", color='#475569', fontsize=9)
ax3.set_ylabel("Energy (J)", color='#475569', fontsize=9)
ax3.legend(loc='lower right', fontsize=8, framealpha=0.8, facecolor='#FFFFFF', edgecolor='#E2E8F0')

# Trackers (Solid understated dots)
dot_th, = ax1.plot([], [], 'o', color='#1E40AF', markersize=4.5)
dot_x, = ax2.plot([], [], 'o', color='#047857', markersize=4.5)
dot_e, = ax3.plot([], [], 'o', color='#991B1B', markersize=4.5)

def update(frame):
    t_c = t_arr[frame]
    x_c = x_arr[frame]
    th_c_rad = th_raw_arr[frame]
    th_c_deg = th_deg_arr[frame]
    a_c = a_arr[frame]
    e_c = e_arr[frame]
    m_c = mode_arr[frame]
    
    pivot_x = x_c
    pivot_y = cart_base_y + cart_h / 2
    top_x = pivot_x + L * math.sin(th_c_rad)
    top_y = pivot_y + L * math.cos(th_c_rad)
    
    cart_rect.set_x(x_c - cart_w / 2)
    enc_circle.center = (pivot_x, pivot_y)
    pendulum_line.set_data([pivot_x, top_x], [pivot_y, top_y])
    tip_circle.center = (top_x, top_y)
    
    f_len = a_c * 0.007
    force_line.set_data([pivot_x, pivot_x + f_len], [pivot_y, pivot_y])
    
    if m_c == "LQR_BALANCE":
        status_badge.set_text("STATE: 5-STATE LQR BALANCE")
        status_badge.set_color('#065F46')
        status_badge.set_bbox(dict(boxstyle='square,pad=0.3', facecolor='#D1FAE5', edgecolor='#059669', lw=1.0))
        tip_circle.set_facecolor('#059669')
    else:
        status_badge.set_text("STATE: ENERGY SWING-UP")
        status_badge.set_color('#92400E')
        status_badge.set_bbox(dict(boxstyle='square,pad=0.3', facecolor='#FEF3C7', edgecolor='#D97706', lw=1.0))
        tip_circle.set_facecolor('#B45309')
        
    info_txt = (
        f"t : {t_c:4.2f} s | θ: {th_c_deg:+5.1f}°\n"
        f"x : {x_c*100:+5.1f} cm | E: {e_c:+5.3f} J\n"
        f"a : {a_c:+5.1f} m/s²"
    )
    info_text.set_text(info_txt)
    
    sub_t = t_arr[:frame+1]
    line_th.set_data(sub_t, th_clean_deg[:frame+1])
    line_x.set_data(sub_t, x_arr[:frame+1] * 100.0)
    line_e.set_data(sub_t, e_arr[:frame+1])
    
    dot_th.set_data([t_c], [th_c_deg])
    dot_x.set_data([t_c], [x_c * 100.0])
    dot_e.set_data([t_c], [e_c])
    
    return [cart_rect, enc_circle, pendulum_line, tip_circle, force_line, status_badge, info_text,
            line_th, line_x, line_e, dot_th, dot_x, dot_e]

OUTPUT_VIDEO = "/home/karim/Documents/pendulum_swiss_simulation.mp4"
print(f"Rendering {N_FRAMES} frames (T={T_TOTAL}s) to {OUTPUT_VIDEO} in clean Swiss academic style...")

writer = animation.FFMpegWriter(fps=FPS, metadata=dict(artist='Karim Laiti'),
                                extra_args=['-vcodec', 'libx264', '-pix_fmt', 'yuv420p', '-preset', 'fast'])

ani = animation.FuncAnimation(fig, update, frames=N_FRAMES, blit=True)
ani.save(OUTPUT_VIDEO, writer=writer)
plt.close(fig)

print("SUCCESS: Sober Swiss academic video rendered cleanly!")
