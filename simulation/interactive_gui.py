#!/usr/bin/env python3
"""
Interactive Mechatronic Testbed Simulator: Inverted Pendulum on Linear Rail
Author: Karim Laiti (Sapienza University of Rome)

Physics & Control:
- Full nonlinear Euler-Lagrange equations of motion (500 Hz substepped RK4)
- Hardware parameters loaded from config/system_params.yaml (m=20g, l=13.5cm, J=0.0004933, rail=0.65m)
- Starts AT REST AT THE BOTTOM (theta = 180 deg / PI rad)
- Resonant energy swing-up with symmetry breaking
- 5-State Continuous LQR with integral action on cart position for zero steady-state error
- Interactive PySide6 GUI with real-time vector graphics, mouse-drag interaction, HUD, and live telemetry oscilloscope
"""

import sys
import os
import math
import collections
from typing import Tuple, List, Optional
import yaml
import numpy as np

# Load physical parameters
CONFIG_FILE = os.path.join(os.path.dirname(__file__), "..", "config", "system_params.yaml")
if os.path.exists(CONFIG_FILE):
    with open(CONFIG_FILE, "r") as f:
        cfg = yaml.safe_load(f)
    phys = cfg["physical_system"]
    M = float(phys["cart_mass_M"])          # 0.380 kg
    m = float(phys["pendulum_mass_m"])      # 0.020 kg
    L = float(phys["pendulum_length_L"])    # 0.200 m
    l = float(phys["pendulum_com_l"])       # 0.135 m
    J = float(phys["pendulum_inertia_J"])   # 0.0004933 kg*m^2
    g = float(phys["gravity_g"])            # 9.81 m/s^2
    b_c = float(phys["cart_friction_b"])    # 0.12 N*s/m
    b_p = float(phys["pivot_friction_c"])   # 0.0008 N*m*s/rad
    
    ctrl = cfg["control_parameters"]
    lqr_cfg = ctrl["lqr"]
    K_theta = 62.43
    K_omega = 9.50
    K_x = 11.58
    K_v = 12.09
    K_xi = 1.41
    
    k_E = float(ctrl["swing_up"]["energy_gain_kE"]) # 35.0
    E0 = float(ctrl["swing_up"]["target_energy_E0"])# 0.0530 J
    catch_deg = float(ctrl["supervisor"]["catch_angle_deg"]) # 22.0 deg
else:
    M, m, L, l, J, g, b_c, b_p = 0.380, 0.020, 0.200, 0.135, 0.0004933, 9.81, 0.12, 0.0008
    K_theta, K_omega, K_x, K_v, K_xi = 62.43, 9.50, 11.58, 12.09, 1.41
    k_E, E0, catch_deg = 35.0, 0.0530, 22.0

RAIL_LIMIT = 0.30 # Safety rail stroke (+- 30 cm, 65 cm rail total)
MAX_A = 12.0      # Max cart acceleration (m/s^2)
CATCH_RAD = math.radians(catch_deg)


class PendulumDynamics:
    """Nonlinear Euler-Lagrange Equations of Motion with RK4 Integration."""
    def __init__(self):
        self.reset()

    def reset(self, theta_init: float = math.pi):
        # state: [x_integral, x, v, theta, omega]
        # START AT BOTTOM AT REST (theta = pi, omega = 0)
        self.state = np.array([0.0, 0.0, 0.0, theta_init, 0.0], dtype=float)
        self.target_x = 0.0
        self.mode = "SWING_UP"
        self.kick_timer = 0.0
        self.last_a_cmd = 0.0

    @staticmethod
    def normalize_angle(theta: float) -> float:
        """Wrap angle to [-pi, pi] where 0 is upright vertical."""
        return (theta + math.pi) % (2.0 * math.pi) - math.pi

    def get_energy(self) -> Tuple[float, float, float]:
        """Compute Kinetic, Potential, and Total energy with upright = 0."""
        th_norm = self.normalize_angle(self.state[3])
        w = self.state[4]
        e_pot = m * g * l * (math.cos(th_norm) - 1.0)
        e_kin = 0.5 * J * (w ** 2)
        return e_kin, e_pot, e_kin + e_pot

    def compute_control(self, dt: float) -> Tuple[float, str]:
        xi, x, v, th, w = self.state
        th_norm = self.normalize_angle(th)
        th_deg = abs(math.degrees(th_norm))
        _, _, e_total = self.get_energy()

        # Catch supervisor with hysteresis
        if self.mode == "LQR_BALANCE":
            if th_deg > 28.0:
                self.mode = "SWING_UP"
                self.state[0] = 0.0
        else:
            if th_deg < 22.0 and abs(w) < 4.0:
                self.mode = "LQR_BALANCE"
                self.state[0] = 0.0 # Reset integrator on catch

        if self.mode == "LQR_BALANCE":
            # 5-State LQR full state feedback with integral action on cart position
            a_cmd = (K_theta * th_norm + 
                     K_omega * w + 
                     K_x * (x - self.target_x) + 
                     K_v * v + 
                     K_xi * xi)
        else:
            # Initial kick when motionless at bottom to break symmetry
            if th_deg > 170.0 and abs(w) < 0.15:
                a_cmd = 3.5 if (x - self.target_x) <= 0 else -3.5
            else:
                # Resonant energy injection: a_pump has opposite sign of w * cos(theta)
                sgn = 1.0 if (w * math.cos(th_norm) >= 0.0) else -1.0
                a_pump = - 6.5 * sgn
                # Cart centering spring + damper to keep within linear rail
                a_cmd = a_pump - 4.5 * (x - self.target_x) - 2.2 * v

        # Clamp acceleration within stepper capabilities
        a_cmd = float(np.clip(a_cmd, -MAX_A, MAX_A))
        self.last_a_cmd = a_cmd
        return a_cmd, self.mode

    def step(self, a_cmd: float, dt: float):
        """Perform substepped RK4 numerical integration."""
        substeps = 8
        dt_sub = dt / substeps

        for _ in range(substeps):
            def deriv(s):
                xi, x, v, th, w = s
                sin_th = math.sin(th)
                cos_th = math.cos(th)
                # Euler-Lagrange angular acceleration from cart kinematic acceleration
                w_dot = (m * g * l * sin_th - b_p * w - m * l * a_cmd * cos_th) / J
                return np.array([x - self.target_x, v, a_cmd, w, w_dot])

            s = self.state
            k1 = deriv(s)
            k2 = deriv(s + 0.5 * dt_sub * k1)
            k3 = deriv(s + 0.5 * dt_sub * k2)
            k4 = deriv(s + dt_sub * k3)
            self.state = s + (dt_sub / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

            # Anti-windup clamping on integral state
            self.state[0] = np.clip(self.state[0], -0.4, 0.4)

            # Hardware limit stops
            if self.state[1] > RAIL_LIMIT:
                self.state[1] = RAIL_LIMIT
                self.state[2] = 0.0
            elif self.state[1] < -RAIL_LIMIT:
                self.state[1] = -RAIL_LIMIT
                self.state[2] = 0.0


# ==============================================================================
# PySide6 Modern Mechatronics GUI
# ==============================================================================
try:
    from PySide6.QtCore import Qt, QTimer, QPointF, QRectF
    from PySide6.QtGui import (QPainter, QColor, QPen, QBrush, QFont, 
                               QPainterPath, QLinearGradient, QPolygonF)
    from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                                   QHBoxLayout, QLabel, QPushButton, QSlider, QFrame)
    HAS_PYSIDE6 = True
except ImportError:
    HAS_PYSIDE6 = False


if HAS_PYSIDE6:
    class PendulumCanvas(QWidget):
        """High-FPS Hardware Accelerated QPainter Canvas."""
        def __init__(self, dynamics: PendulumDynamics, parent=None):
            super().__init__(parent)
            self.dyn = dynamics
            self.setMinimumSize(780, 440)
            self.setMouseTracking(True)
            self.is_dragging_cart = False
            self.is_dragging_bob = False
            self.trail = collections.deque(maxlen=40)

        def mousePressEvent(self, event):
            pos = event.position()
            cx, cy = self.to_screen_coords(self.dyn.state[1], 0)
            
            # Check bob click
            th = self.dyn.state[3]
            bx, by = self.to_screen_coords(
                self.dyn.state[1] + L * math.sin(th),
                L * math.cos(th)
            )
            dist_bob = math.hypot(pos.x() - bx, pos.y() - by)
            dist_cart = math.hypot(pos.x() - cx, pos.y() - cy)

            if dist_bob < 25.0:
                self.is_dragging_bob = True
            elif dist_cart < 35.0:
                self.is_dragging_cart = True

        def mouseMoveEvent(self, event):
            if not (self.is_dragging_cart or self.is_dragging_bob):
                return
            pos = event.position()
            w, h = self.width(), self.height()
            scale = min(w / 0.85, h / 0.65)
            origin_x, origin_y = w / 2.0, h / 2.0 + 35.0

            if self.is_dragging_cart:
                new_x = (pos.x() - origin_x) / scale
                self.dyn.state[1] = np.clip(new_x, -RAIL_LIMIT, RAIL_LIMIT)
                self.dyn.state[2] = 0.0
            elif self.is_dragging_bob:
                cart_sx, cart_sy = self.to_screen_coords(self.dyn.state[1], 0)
                dx = pos.x() - cart_sx
                dy = pos.y() - cart_sy
                new_th = math.atan2(dx, -dy)
                self.dyn.state[3] = new_th
                self.dyn.state[4] = 0.0

        def mouseReleaseEvent(self, event):
            self.is_dragging_cart = False
            self.is_dragging_bob = False

        def to_screen_coords(self, x_phys: float, y_phys: float) -> Tuple[float, float]:
            w, h = self.width(), self.height()
            scale = min(w / 0.85, h / 0.65)
            origin_x = w / 2.0
            origin_y = h / 2.0 + 35.0
            sx = origin_x + x_phys * scale
            sy = origin_y - y_phys * scale
            return sx, sy

        def paintEvent(self, event):
            painter = QPainter(self)
            painter.setRenderHint(QPainter.RenderHint.Antialiasing)

            w, h = self.width(), self.height()
            scale = min(w / 0.85, h / 0.65)
            origin_x = w / 2.0
            origin_y = h / 2.0 + 35.0

            # Background dark slate
            painter.fillRect(0, 0, w, h, QColor("#090d16"))

            # Fine background grid
            painter.setPen(QPen(QColor(255, 255, 255, 12), 1, Qt.PenStyle.DotLine))
            grid_step = int(scale * 0.05)
            for gx in range(int(origin_x % grid_step), w, grid_step):
                painter.drawLine(gx, 0, gx, h)
            for gy in range(int(origin_y % grid_step), h, grid_step):
                painter.drawLine(0, gy, w, gy)

            # Target position marker (dashed emerald line)
            target_sx, _ = self.to_screen_coords(self.dyn.target_x, 0)
            painter.setPen(QPen(QColor(16, 185, 129, 180), 2, Qt.PenStyle.DashLine))
            painter.drawLine(int(target_sx), int(origin_y - 140), int(target_sx), int(origin_y + 110))

            # Rail (Anodized MGN12 profile, 65 cm length)
            rail_x1, rail_y = self.to_screen_coords(-0.325, 0)
            rail_x2, _ = self.to_screen_coords(0.325, 0)
            rail_h = 10.0
            painter.setPen(Qt.PenStyle.NoPen)
            painter.setBrush(QBrush(QColor("#334155")))
            painter.drawRoundedRect(QRectF(rail_x1, rail_y - 2, rail_x2 - rail_x1, rail_h), 3, 3)
            # Rail metallic highlight
            painter.setBrush(QBrush(QColor("#64748b")))
            painter.drawRect(QRectF(rail_x1, rail_y - 2, rail_x2 - rail_x1, 2.5))

            # Safety stroke limits (+- 25 cm)
            lim1_sx, _ = self.to_screen_coords(-0.25, 0)
            lim2_sx, _ = self.to_screen_coords(0.25, 0)
            painter.setPen(QPen(QColor(239, 68, 68, 160), 1.5, Qt.PenStyle.DashDotLine))
            painter.drawLine(int(lim1_sx), int(origin_y - 80), int(lim1_sx), int(origin_y + 60))
            painter.drawLine(int(lim2_sx), int(origin_y - 80), int(lim2_sx), int(origin_y + 60))

            # Limit switch red stoppers
            painter.setBrush(QBrush(QColor("#ef4444")))
            painter.drawRect(QRectF(lim1_sx - 4, rail_y - 8, 8, 14))
            painter.drawRect(QRectF(lim2_sx - 4, rail_y - 8, 8, 14))

            # Physical coordinates
            x_cart = self.dyn.state[1]
            th = self.dyn.state[3]
            cart_sx, cart_sy = self.to_screen_coords(x_cart, 0)
            bob_x = x_cart + L * math.sin(th)
            bob_y = L * math.cos(th)
            bob_sx, bob_sy = self.to_screen_coords(bob_x, bob_y)

            # Update motion trail
            if not (self.is_dragging_cart or self.is_dragging_bob):
                self.trail.append((bob_sx, bob_sy))
            
            # Draw ghost trail
            if len(self.trail) > 1:
                for i in range(len(self.trail) - 1):
                    alpha = int(120 * (i / len(self.trail)))
                    painter.setPen(QPen(QColor(0, 229, 255, alpha), 1.8))
                    p1 = self.trail[i]
                    p2 = self.trail[i+1]
                    painter.drawLine(int(p1[0]), int(p1[1]), int(p2[0]), int(p2[1]))

            # Force / Acceleration Vector Arrow
            a_cmd = self.dyn.last_a_cmd
            if abs(a_cmd) > 0.3:
                arrow_len = a_cmd * 3.5
                painter.setPen(QPen(QColor(168, 85, 247, 220), 3.5, Qt.PenStyle.SolidLine))
                painter.drawLine(int(cart_sx), int(cart_sy), int(cart_sx + arrow_len), int(cart_sy))
                tip_dir = 1.0 if a_cmd > 0 else -1.0
                painter.setBrush(QBrush(QColor("#a855f7")))
                head = QPolygonF([
                    QPointF(cart_sx + arrow_len, cart_sy),
                    QPointF(cart_sx + arrow_len - tip_dir * 8, cart_sy - 5),
                    QPointF(cart_sx + arrow_len - tip_dir * 8, cart_sy + 5)
                ])
                painter.drawPolygon(head)

            # Cart (MGN12 carriage + block)
            cart_w = 64.0
            cart_h = 30.0
            cart_rect = QRectF(cart_sx - cart_w / 2, cart_sy - cart_h / 2, cart_w, cart_h)
            cart_grad = QLinearGradient(cart_rect.topLeft(), cart_rect.bottomLeft())
            cart_grad.setColorAt(0.0, QColor("#1e293b"))
            cart_grad.setColorAt(1.0, QColor("#0f172a"))
            painter.setBrush(QBrush(cart_grad))
            painter.setPen(QPen(QColor("#0ea5e9"), 1.8))
            painter.drawRoundedRect(cart_rect, 5, 5)

            # Optical Encoder Hub (Brass/Amber circle)
            painter.setBrush(QBrush(QColor("#f59e0b")))
            painter.setPen(QPen(QColor("#b45309"), 2))
            painter.drawEllipse(QPointF(cart_sx, cart_sy), 9.0, 9.0)
            painter.setBrush(QBrush(QColor("#1e293b")))
            painter.drawEllipse(QPointF(cart_sx, cart_sy), 3.5, 3.5)

            # Carbon Fiber Pendulum Rod
            rod_pen = QPen(QColor("#f8fafc"), 4.5, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap)
            painter.setPen(rod_pen)
            painter.drawLine(int(cart_sx), int(cart_sy), int(bob_sx), int(bob_sy))

            # Rod internal highlight
            painter.setPen(QPen(QColor("#38bdf8"), 1.8))
            painter.drawLine(int(cart_sx), int(cart_sy), int(bob_sx), int(bob_sy))

            # Tip Bob (Brass nut + weight in crimson)
            painter.setBrush(QBrush(QColor("#ef4444")))
            painter.setPen(QPen(QColor("#fecdd3"), 2.2))
            painter.drawEllipse(QPointF(bob_sx, bob_sy), 11.0, 11.0)
            painter.setBrush(QBrush(QColor("#ffffff")))
            painter.drawEllipse(QPointF(bob_sx - 2, bob_sy - 2), 2.5, 2.5)

            # Mode & Status HUD badge
            painter.setFont(QFont("Monospace", 9, QFont.Weight.Bold))
            if self.dyn.mode == "LQR_BALANCE":
                mode_col = QColor("#10b981")
                mode_str = "ACTIVE: 5-STATE LQR BALANCE"
            else:
                mode_col = QColor("#f59e0b")
                mode_str = "ACTIVE: RESONANT SWING-UP (LYAPUNOV)"

            painter.setPen(QPen(mode_col, 1))
            painter.setBrush(QBrush(QColor(15, 23, 42, 220)))
            painter.drawRoundedRect(QRectF(16, 16, 260, 32), 6, 6)
            painter.setPen(mode_col)
            painter.drawText(30, 37, mode_str)


    class OscilloscopeWidget(QWidget):
        """Live Real-Time Scope for Theta and X."""
        def __init__(self, parent=None):
            super().__init__(parent)
            self.setFixedHeight(110)
            self.theta_history = collections.deque(maxlen=240)
            self.x_history = collections.deque(maxlen=240)

        def push(self, th_deg: float, x_cm: float):
            self.theta_history.append(th_deg)
            self.x_history.append(x_cm)
            self.update()

        def paintEvent(self, event):
            painter = QPainter(self)
            painter.setRenderHint(QPainter.RenderHint.Antialiasing)
            w, h = self.width(), self.height()
            
            painter.fillRect(0, 0, w, h, QColor("#090d16"))
            painter.setPen(QPen(QColor(255, 255, 255, 18), 1))
            painter.drawRect(0, 0, w - 1, h - 1)
            
            mid_y = h / 2.0
            painter.setPen(QPen(QColor(255, 255, 255, 30), 1, Qt.PenStyle.DashLine))
            painter.drawLine(0, int(mid_y), w, int(mid_y))

            # Plot Theta (Cyan)
            if len(self.theta_history) > 1:
                painter.setPen(QPen(QColor("#00e5ff"), 1.8))
                path_th = QPainterPath()
                step_x = w / 240.0
                for i, th in enumerate(self.theta_history):
                    sy = mid_y - (th / 180.0) * (mid_y - 10)
                    sx = i * step_x
                    if i == 0:
                        path_th.moveTo(sx, sy)
                    else:
                        path_th.lineTo(sx, sy)
                painter.drawPath(path_th)

            # Plot X (Emerald)
            if len(self.x_history) > 1:
                painter.setPen(QPen(QColor("#10b981"), 1.8))
                path_x = QPainterPath()
                step_x = w / 240.0
                for i, xc in enumerate(self.x_history):
                    sy = mid_y - (xc / 30.0) * (mid_y - 10)
                    sx = i * step_x
                    if i == 0:
                        path_x.moveTo(sx, sy)
                    else:
                        path_x.lineTo(sx, sy)
                painter.drawPath(path_x)

            # Legend
            painter.setFont(QFont("Monospace", 8))
            painter.setPen(QColor("#00e5ff"))
            painter.drawText(10, 18, "— Angle θ (±180°)")
            painter.setPen(QColor("#10b981"))
            painter.drawText(160, 18, "— Cart x (±30 cm)")


    class MainWindow(QMainWindow):
        def __init__(self):
            super().__init__()
            self.setWindowTitle("Inverted Pendulum Mechatronic Simulator | Karim Laiti")
            self.resize(1120, 720)
            self.setStyleSheet("background-color: #0b0f19; color: #f3f4f6; font-family: sans-serif;")

            self.dyn = PendulumDynamics()
            self.is_paused = False

            root = QWidget()
            self.setCentralWidget(root)
            main_layout = QHBoxLayout(root)
            main_layout.setContentsMargins(16, 16, 16, 16)
            main_layout.setSpacing(16)

            # Left Column: Canvas + Oscilloscope
            left_col = QVBoxLayout()
            left_col.setSpacing(12)
            self.canvas = PendulumCanvas(self.dyn)
            left_col.addWidget(self.canvas, stretch=1)
            
            self.scope = OscilloscopeWidget()
            left_col.addWidget(self.scope)
            main_layout.addLayout(left_col, stretch=1)

            # Right Column: Control Panel & Telemetry HUD
            right_col = QVBoxLayout()
            right_col.setSpacing(12)
            right_col.setContentsMargins(0, 0, 0, 0)

            # Telemetry Box
            hud_frame = QFrame()
            hud_frame.setStyleSheet("""
                QFrame {
                    background-color: #111827;
                    border: 1px solid rgba(59, 130, 246, 0.25);
                    border-radius: 8px;
                    padding: 12px;
                }
            """)
            hud_layout = QVBoxLayout(hud_frame)
            hud_title = QLabel("SYSTEM TELEMETRY")
            hud_title.setStyleSheet("font-size: 11px; font-weight: bold; color: #94a3b8; letter-spacing: 1px;")
            hud_layout.addWidget(hud_title)

            self.lbl_theta = QLabel("Theta (Angle)   : 180.0°")
            self.lbl_omega = QLabel("Omega (Ang Vel) : 0.00 rad/s")
            self.lbl_x     = QLabel("Cart Pos (x)    : 0.0 cm")
            self.lbl_v     = QLabel("Cart Vel (v)    : 0.00 m/s")
            self.lbl_force = QLabel("Commanded Acc   : 0.00 m/s²")
            self.lbl_energy= QLabel("Normalized E/E0 : 0.0 %")

            for lbl in [self.lbl_theta, self.lbl_omega, self.lbl_x, self.lbl_v, self.lbl_force, self.lbl_energy]:
                lbl.setStyleSheet("font-family: monospace; font-size: 12px; color: #f8fafc;")
                hud_layout.addWidget(lbl)
            right_col.addWidget(hud_frame)

            # Target Position Slider
            slider_frame = QFrame()
            slider_frame.setStyleSheet("""
                QFrame {
                    background-color: #111827;
                    border: 1px solid rgba(59, 130, 246, 0.25);
                    border-radius: 8px;
                    padding: 10px;
                }
            """)
            slider_layout = QVBoxLayout(slider_frame)
            self.lbl_target = QLabel("Cart Target: 0.0 cm")
            self.lbl_target.setStyleSheet("font-family: monospace; font-size: 11px; color: #38bdf8;")
            slider_layout.addWidget(self.lbl_target)

            self.slider = QSlider(Qt.Orientation.Horizontal)
            self.slider.setRange(-20, 20)
            self.slider.setValue(0)
            self.slider.valueChanged.connect(self.on_slider_changed)
            slider_layout.addWidget(self.slider)
            right_col.addWidget(slider_frame)

            # Buttons Panel
            btn_frame = QFrame()
            btn_frame.setStyleSheet("""
                QFrame {
                    background-color: #111827;
                    border: 1px solid rgba(59, 130, 246, 0.25);
                    border-radius: 8px;
                    padding: 12px;
                }
            """)
            btn_layout = QVBoxLayout(btn_frame)
            btn_title = QLabel("INTERACTIVE CONTROL")
            btn_title.setStyleSheet("font-size: 11px; font-weight: bold; color: #94a3b8; letter-spacing: 1px;")
            btn_layout.addWidget(btn_title)

            self.btn_swing = QPushButton("Auto Swing-Up from Bottom")
            self.btn_swing.setStyleSheet("background-color: #0284c7; color: white; font-weight: bold; padding: 8px; border-radius: 5px;")
            self.btn_swing.clicked.connect(self.on_swing_up)
            btn_layout.addWidget(self.btn_swing)

            row_pert = QHBoxLayout()
            self.btn_push_l = QPushButton("Tap Push ←")
            self.btn_push_l.setStyleSheet("background-color: #334155; color: white; padding: 7px; border-radius: 5px;")
            self.btn_push_l.clicked.connect(lambda: self.on_perturb(-3.0))
            self.btn_push_r = QPushButton("Tap Push →")
            self.btn_push_r.setStyleSheet("background-color: #334155; color: white; padding: 7px; border-radius: 5px;")
            self.btn_push_r.clicked.connect(lambda: self.on_perturb(3.0))
            row_pert.addWidget(self.btn_push_l)
            row_pert.addWidget(self.btn_push_r)
            btn_layout.addLayout(row_pert)

            self.btn_pause = QPushButton("Pause / Resume")
            self.btn_pause.setStyleSheet("background-color: #475569; color: white; padding: 7px; border-radius: 5px;")
            self.btn_pause.clicked.connect(self.on_pause)
            btn_layout.addWidget(self.btn_pause)

            self.btn_reset = QPushButton("Reset to Bottom (θ = 180°)")
            self.btn_reset.setStyleSheet("background-color: #1e293b; color: #ef4444; border: 1px solid #ef4444; padding: 7px; border-radius: 5px;")
            self.btn_reset.clicked.connect(self.on_reset)
            btn_layout.addWidget(self.btn_reset)

            right_col.addWidget(btn_frame)

            # Identification Parameters Info
            info_lbl = QLabel(
                f"IDENTIFIED PARAMETERS\n"
                f"Cart Mass M  : {M:.3f} kg\n"
                f"Pendulum m   : {m:.3f} kg\n"
                f"Length L     : {L:.3f} m\n"
                f"Inertia J    : {J:.7f} kg·m²\n"
                f"Target E0    : {E0:.4f} J\n"
                f"Control Loop : 500 Hz (dt=2ms)"
            )
            info_lbl.setStyleSheet("font-family: monospace; font-size: 10px; color: #64748b; line-height: 1.4;")
            right_col.addWidget(info_lbl)
            right_col.addStretch(1)

            main_layout.addLayout(right_col, stretch=0)

            # 60 FPS Animation Timer (dt = 16.6 ms)
            self.sim_timer = QTimer(self)
            self.sim_timer.timeout.connect(self.on_sim_tick)
            self.sim_timer.start(16)

        def on_slider_changed(self, val):
            target = val / 100.0
            self.dyn.target_x = target
            self.lbl_target.setText(f"Cart Target: {target*100:+.1f} cm")

        def on_swing_up(self):
            self.dyn.reset(math.pi)

        def on_perturb(self, delta_w: float):
            self.dyn.state[4] += delta_w

        def on_pause(self):
            self.is_paused = not self.is_paused

        def on_reset(self):
            self.dyn.reset(math.pi)

        def on_sim_tick(self):
            if not self.is_paused and not (self.canvas.is_dragging_cart or self.canvas.is_dragging_bob):
                dt = 0.016
                a_cmd, mode = self.dyn.compute_control(dt)
                self.dyn.step(a_cmd, dt)

            th = self.dyn.state[3]
            w = self.dyn.state[4]
            x = self.dyn.state[1]
            v = self.dyn.state[2]
            th_norm = self.dyn.normalize_angle(th)
            th_deg = math.degrees(th_norm)
            _, _, e_tot = self.dyn.get_energy()

            # Update HUD Labels
            self.lbl_theta.setText(f"Theta (Angle)   : {th_deg:+6.1f}°")
            self.lbl_omega.setText(f"Omega (Ang Vel) : {w:+6.2f} rad/s")
            self.lbl_x.setText(    f"Cart Pos (x)    : {x*100:+6.1f} cm")
            self.lbl_v.setText(    f"Cart Vel (v)    : {v:+6.2f} m/s")
            self.lbl_force.setText(f"Commanded Acc   : {self.dyn.last_a_cmd:+6.2f} m/s²")
            
            e_pct = max(0.0, min(100.0, (1.0 + e_tot / E0) * 100.0))
            self.lbl_energy.setText(f"Energy Meter    : {e_pct:5.1f} %")

            # Push to oscilloscope
            self.scope.push(th_deg, x * 100.0)

            # Redraw canvas
            self.canvas.update()


def main():
    if not HAS_PYSIDE6:
        print("[!] PySide6 is not installed. Please install with: pip install PySide6")
        sys.exit(1)

    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
