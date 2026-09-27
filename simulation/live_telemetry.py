#!/usr/bin/env python3
"""
Real-Time Inverted Pendulum Telemetry & Visualizer Dashboard
Connects to ESP8266 via USB Serial (/dev/ttyUSB0 or /dev/ttyUSB1) and displays live:
1. Physical 2D Cart-Pole Animation
2. Pendulum Angle & LQR Catch Cone (+/-22 deg)
3. Angular Velocity (rad/s)
4. Cart Position (cm) & Rail Limits
5. Control Acceleration / Force & Mode Status
"""

import sys
import os
import re
import time
import threading
from collections import deque
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Configure Matplotlib dark theme
plt.style.use('dark_background')
plt.rcParams['font.family'] = 'sans-serif'
plt.rcParams['font.size'] = 9

PORT = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
BAUD = 115200
HISTORY_LEN = 300  # ~15 seconds of history at 20 Hz

time_buf = deque(maxlen=HISTORY_LEN)
theta_buf = deque(maxlen=HISTORY_LEN)
omega_buf = deque(maxlen=HISTORY_LEN)
x_buf = deque(maxlen=HISTORY_LEN)
accel_buf = deque(maxlen=HISTORY_LEN)
state_buf = deque(maxlen=HISTORY_LEN)

start_time = time.time()
latest_data = {
    'mode': 'STANDBY',
    'theta': 0.0,
    'omega': 0.0,
    'x': 0.0,
    'a': 0.0
}

data_lock = threading.Lock()
running = True

# Regex parser for firmware telemetry:
# [SWING_UP ] Th: -178.7 deg | w:  +0.2 rad/s | x:  -0.2 cm | a: -0.02 m/s2
# [LQR_CATCH] Th:   +2.1 deg | w:  -0.1 rad/s | x:  +0.5 cm | a: +1.20 m/s2
pattern = re.compile(
    r'\[(?P<mode>[^\]]+)\]\s+Th:\s*(?P<th>[+-]?\d+\.?\d*)\s*deg\s*\|\s*w:\s*(?P<w>[+-]?\d+\.?\d*)\s*rad/s\s*\|\s*x:\s*(?P<x>[+-]?\d+\.?\d*)\s*cm\s*\|\s*a:\s*(?P<a>[+-]?\d+\.?\d*)'
)

def serial_reader_thread():
    global running, latest_data
    try:
        import serial
    except ImportError:
        print("[ERROR] pyserial is not installed. Run: pip install pyserial")
        return

    try:
        ser = serial.Serial(PORT, BAUD, timeout=1.0)
        print(f"[OK] Connected to hardware testbed on {PORT} at {BAUD} baud.")
    except Exception as e:
        print(f"[WARN] Could not open {PORT}: {e}")
        print("Please check your USB connection or specify port: python live_telemetry.py /dev/ttyUSB1")
        return

    while running:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue

            match = pattern.search(line)
            if match:
                mode_str = match.group('mode').strip()
                th_val = float(match.group('th'))
                w_val = float(match.group('w'))
                x_val = float(match.group('x'))
                a_val = float(match.group('a'))

                t_now = time.time() - start_time

                with data_lock:
                    latest_data['mode'] = mode_str
                    latest_data['theta'] = th_val
                    latest_data['omega'] = w_val
                    latest_data['x'] = x_val
                    latest_data['a'] = a_val

                    time_buf.append(t_now)
                    theta_buf.append(th_val)
                    omega_buf.append(w_val)
                    x_buf.append(x_val)
                    accel_buf.append(a_val)
                    state_buf.append(mode_str)
        except Exception:
            break

    try:
        ser.close()
    except Exception:
        pass

# Start Serial Thread
t_serial = threading.Thread(target=serial_reader_thread, daemon=True)
t_serial.start()

# Setup Plot
fig = plt.figure(figsize=(14, 8))
try:
    fig.canvas.manager.set_window_title(f"Inverted Pendulum - Live Telemetry ({PORT})")
except Exception:
    pass

gs = fig.add_gridspec(3, 2, width_ratios=[1.2, 1.0])

ax_anim = fig.add_subplot(gs[:, 0])
ax_theta = fig.add_subplot(gs[0, 1])
ax_x = fig.add_subplot(gs[1, 1])
ax_accel = fig.add_subplot(gs[2, 1])

fig.patch.set_facecolor('#0b0f19')
for ax in [ax_anim, ax_theta, ax_x, ax_accel]:
    ax.set_facecolor('#111827')
    ax.grid(True, linestyle=':', color='#374151', alpha=0.6)

# Animation elements
ax_anim.set_xlim(-0.35, 0.35)
ax_anim.set_ylim(-0.25, 0.30)
ax_anim.set_aspect('equal')
ax_anim.set_title("Real-Time Kinematics (Physical Testbed)", fontsize=11, fontweight='bold', color='#38bdf8', pad=10)

# Rail
ax_anim.fill_between([-0.325, 0.325], -0.06, -0.04, color='#4b5563', zorder=1)
ax_anim.fill_between([-0.325, 0.325], -0.04, -0.035, color='#9ca3af', zorder=2)
ax_anim.axvline(-0.25, color='#ef4444', linestyle='--', alpha=0.7)
ax_anim.axvline(0.25, color='#ef4444', linestyle='--', alpha=0.7)

from matplotlib.patches import Rectangle, Circle
cart_rect = Rectangle((-0.03, -0.035), 0.06, 0.03, fc='#2563eb', ec='#60a5fa', lw=1.5, zorder=3)
ax_anim.add_patch(cart_rect)
enc_circle = Circle((0, -0.02), 0.012, fc='#1f2937', ec='#f59e0b', lw=1.2, zorder=4)
ax_anim.add_patch(enc_circle)

rod_line, = ax_anim.plot([], [], lw=4, color='#f3f4f6', zorder=5)
tip_circle = Circle((0, 0), 0.009, fc='#e11d48', ec='#fda4af', lw=1.5, zorder=6)
ax_anim.add_patch(tip_circle)

hud_text = ax_anim.text(0.04, 0.95, '', transform=ax_anim.transAxes, fontsize=10,
                        verticalalignment='top', family='monospace', color='#f9fafb',
                        bbox=dict(boxstyle='round,pad=0.5', fc='#1f2937', ec='#3b82f6', alpha=0.85))

# Subplots lines
line_theta, = ax_theta.plot([], [], color='#00E5FF', lw=1.8, label=r'$\theta$ (deg)')
ax_theta.axhline(22, color='#eab308', linestyle=':', alpha=0.8, label=r'Catch Cone ($\pm 22^\circ$)')
ax_theta.axhline(-22, color='#eab308', linestyle=':', alpha=0.8)
ax_theta.set_ylabel(r'$\theta$ [deg]', color='#00E5FF')
ax_theta.set_ylim(-190, 190)
ax_theta.legend(loc='upper right', fontsize=8)

line_x, = ax_x.plot([], [], color='#76FF03', lw=1.8, label=r'$x$ (cm)')
ax_x.axhline(22, color='#ef4444', linestyle='--', alpha=0.7, label=r'Soft Limit ($\pm 22$ cm)')
ax_x.axhline(-22, color='#ef4444', linestyle='--', alpha=0.7)
ax_x.set_ylabel(r'$x$ [cm]', color='#76FF03')
ax_x.set_ylim(-26, 26)
ax_x.legend(loc='upper right', fontsize=8)

line_accel, = ax_accel.plot([], [], color='#FFD600', lw=1.8, label=r'$a_{\mathrm{cmd}}$ (m/s$^2$)')
ax_accel.set_ylabel(r'$a$ [m/s$^2$]', color='#FFD600')
ax_accel.set_xlabel('Time [s]')
ax_accel.set_ylim(-10, 10)
ax_accel.legend(loc='upper right', fontsize=8)

L_ROD = 0.200

def update_gui(frame):
    with data_lock:
        x_m = latest_data['x'] / 100.0
        th_deg = latest_data['theta']
        th_rad = np.radians(th_deg)
        w_val = latest_data['omega']
        a_val = latest_data['a']
        mode_str = latest_data['mode']

        times = list(time_buf)
        thetas = list(theta_buf)
        xs = list(x_buf)
        accels = list(accel_buf)

    # Kinematics
    pivot_x = x_m
    pivot_y = -0.02
    tip_x = pivot_x + L_ROD * np.sin(th_rad)
    tip_y = pivot_y + L_ROD * np.cos(th_rad)

    cart_rect.set_x(pivot_x - 0.03)
    enc_circle.center = (pivot_x, pivot_y)
    rod_line.set_data([pivot_x, tip_x], [pivot_y, tip_y])
    tip_circle.center = (tip_x, tip_y)

    hud_text.set_text(
        f"HARDWARE TELEMETRY\n"
        f"-------------------\n"
        f"Mode  : {mode_str}\n"
        f"Theta : {th_deg:+6.1f} deg\n"
        f"Omega : {w_val:+6.2f} rad/s\n"
        f"Cart x: {x_m*100:+6.1f} cm\n"
        f"Accel : {a_val:+6.2f} m/s^2"
    )

    if times:
        t_max = max(times)
        t_min = max(0, t_max - 12)

        for ax in [ax_theta, ax_x, ax_accel]:
            ax.set_xlim(t_min, t_max + 0.5)

        line_theta.set_data(times, thetas)
        line_x.set_data(times, xs)
        line_accel.set_data(times, accels)

    return cart_rect, enc_circle, rod_line, tip_circle, hud_text, line_theta, line_x, line_accel

ani = animation.FuncAnimation(fig, update_gui, interval=25, blit=False)

if __name__ == '__main__':
    try:
        plt.tight_layout()
        plt.show()
    finally:
        running = False
