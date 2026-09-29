"""
ELITA Robot — Python Controller (Laptop) v5
===========================================
Tema    : gelap (latar hitam), tombol START / STOP / RESET / LOG, input W & H
Koneksi : WebSocket ke ESP32 ws://192.168.4.1:81  (Uno v4.1 + ESP32 v4.3)
Metode  : Coverage path boustrophedon, kontrol gerak open-loop terkalibrasi

KALIBRASI (lantai datar, baterai penuh, 30-09-2026):
  Maju 156 cm  : 4,77 / 4,81 / 4,84 s  -> V_FWD = 0,3245 m/s
  U-turn kanan : 7,80 s                -> diameter 195 cm (diukur)
  U-turn kiri  : 8,45 s (1x)           -> diameter 195 cm (diukur)
  => Radius belok R = 0,975 m, jarak lajur minimum = 2R = 1,95 m.
  Setiap gerak dimulai dari DIAM dengan perintah yang sama seperti tombol
  dashboard saat kalibrasi (servo + motor dikirim bersamaan), lalu STOP.

POLA:
  Start pojok kiri bawah (0,0), menghadap +Y (ke atas, sejajar sisi H).
  Lajur ke-i di x = i*RS, panjang H. Belokan dilakukan DI LUAR area
  (butuh ruang ~R = 1 m di atas & di bawah area).
    RS == 2R : U-turn full-lock (kanan di atas, kiri di bawah)
    RS >  2R : belok 90 -> maju (RS - 2R) -> belok 90   ("belokan Pi")
    RS <  2R : ditolak (robot tidak bisa belok setajam itu)
  Cutter ON hanya di lajur lurus, OFF saat belok.

CATATAN:
  Jejak di layar adalah odometri dead-reckoning dari parameter kalibrasi
  (V_FWD, T_UTURN, R_TURN). Verifikasi ketepatan: tandai posisi akhir robot
  di lantai lalu bandingkan dengan titik target di peta.

Instalasi:
  pip install websocket-client matplotlib numpy
"""

import threading
import json
import time
import math
import csv
from datetime import datetime
from collections import deque

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
plt.style.use("dark_background")
import matplotlib.patches as patches
import matplotlib.transforms as transforms
from matplotlib.widgets import Button, TextBox
from matplotlib.animation import FuncAnimation

from websocket import WebSocketApp

# ──────────────────────────────────────────────────────────────
# KONFIGURASI & KALIBRASI
# ──────────────────────────────────────────────────────────────
DEFAULT_IP     = "192.168.4.1"
WS_PORT        = 81

SERVO_CENTER   = 75
SERVO_LEFT     = 45
SERVO_RIGHT    = 110

V_FWD          = 1.56 / ((4.77 + 4.81 + 4.84) / 3.0)   # m/s, maju lurus
T_UTURN = {+1: 7.80,          # detik, U-turn 180 derajat KANAN (servo 110) - kalibrasi ulang
           -1: 8.45}          # detik, U-turn 180 derajat KIRI  (servo 45)
R_TURN         = 1.95 / 2.0   # m, radius belok hasil ukur (kiri = kanan)
MOTOR_ON       = 255          # Uno v4: motor ON/OFF saja

PAUSE_S        = 0.4          # diam di antara gerak (robot benar-benar berhenti)
HEARTBEAT_S    = 0.2
TEL_STALE_S    = 1.5
SIM_SPEEDUP    = 3.0

# Dimensi robot (m)
WHEELBASE      = 0.25
TRACK          = 0.18
WHEEL_D        = 0.06
WHEEL_W        = 0.03
BODY_LEN       = 0.34
BODY_W         = 0.21
REAR_OVERHANG  = 0.045
STEER_DRAW_DEG = math.degrees(math.atan(WHEELBASE / R_TURN))   # sudut efektif ~14 derajat

BG_BTN         = "#1f2430"
BG_HOVER       = "#2d3446"
GRID_C         = "#2a2f3a"
COL_CUT        = "#ff4040"   # lurus / cutter ON  (merah)
COL_TURN       = "#3b82f6"   # belok / U-turn     (biru)

AREA_W_DEF     = 4.0
AREA_H_DEF     = 3.0
ROW_SP_DEF     = 1.95

CHART_LEN      = 150


def ws_url(ip):
    return f"ws://{ip}:{WS_PORT}"


# ──────────────────────────────────────────────────────────────
# TERMINAL STATUS
# ──────────────────────────────────────────────────────────────
_status_lock = threading.Lock()
STATUS_LINES = deque(maxlen=18)


def push_status(line):
    ts = datetime.now().strftime("%H:%M:%S")
    with _status_lock:
        STATUS_LINES.append(f"[{ts}] {line}")
    print(line, flush=True)


def status_snapshot(n=12):
    with _status_lock:
        return list(STATUS_LINES)[-n:]


# ──────────────────────────────────────────────────────────────
# LINK WEBSOCKET
# ──────────────────────────────────────────────────────────────
TEL_KEYS = ("mk", "mr", "ms", "sv", "ct", "wd", "bf", "el", "er", "up")


class RobotLink:
    def __init__(self, url):
        self.url = url
        self._app = None
        self._lock = threading.Lock()
        self._send_lock = threading.Lock()
        self._closing = False
        self._fail = 0
        self.connected = False
        self.uno_flag = False
        self.last_tel = 0.0
        self.tel = {k: 0 for k in TEL_KEYS}
        self.tel["sv"] = SERVO_CENTER
        threading.Thread(target=self._run, daemon=True, name="ws-link").start()

    def _run(self):
        last_url = None
        while not self._closing:
            url = self.url
            if url != last_url or self._fail % 10 == 0:
                push_status(f"[WS] Menghubungkan ke {url} ...")
            last_url = url
            app = WebSocketApp(url, on_open=self._on_open, on_message=self._on_message,
                               on_close=self._on_close, on_error=self._on_error)
            self._app = app
            try:
                app.run_forever(ping_interval=5, ping_timeout=3)
            except Exception as e:
                push_status(f"[WS ERR] {e}")
            was = self.connected
            self.connected = False
            self.uno_flag = False
            self._fail = 0 if was else self._fail + 1
            if self._closing:
                break
            time.sleep(2.0)

    def _on_open(self, ws):
        self.connected = True
        self._fail = 0
        push_status(f"[WS] Terhubung ke {self.url}")

    def _on_close(self, ws, *a):
        if self.connected:
            push_status("[WS] Terputus, reconnect otomatis...")
        self.connected = False

    def _on_error(self, ws, err):
        if self._fail % 10 == 0:
            push_status(f"[WS ERR] {err}")

    def _on_message(self, ws, message):
        try:
            d = json.loads(message)
        except ValueError:
            return
        if not isinstance(d, dict):
            return
        t = d.get("t")
        if t == "tel" and d.get("dg"):
            return
        if t == "tel":
            with self._lock:
                for k in TEL_KEYS:
                    if k in d:
                        try:
                            self.tel[k] = int(d[k])
                        except (TypeError, ValueError):
                            pass
                self.last_tel = time.monotonic()
            self.uno_flag = True
        elif t == "st":
            online = bool(d.get("uno"))
            if online != self.uno_flag:
                push_status("[ESP32] Uno ONLINE" if online else "[ESP32] Uno OFFLINE")
            self.uno_flag = online
        elif t == "err":
            push_status(f"[ESP32] {d.get('m', 'error')}")

    def get_tel(self):
        with self._lock:
            return dict(self.tel)

    @property
    def robot_online(self):
        with self._lock:
            fresh = (time.monotonic() - self.last_tel) < TEL_STALE_S
        return self.connected and self.uno_flag and fresh

    def watchdog_active(self):
        with self._lock:
            return bool(self.tel.get("wd"))

    def send(self, obj):
        app = self._app
        if not (self.connected and app):
            return False
        try:
            with self._send_lock:
                app.send(json.dumps(obj, separators=(",", ":")))
            return True
        except Exception as e:
            push_status(f"[SEND ERR] {e}")
            return False

    def set_url(self, url):
        if url == self.url:
            return
        push_status(f"[WS] Ganti target -> {url}")
        self.url = url
        app = self._app
        if app:
            app.close()

    def close(self):
        self._closing = True
        app = self._app
        if app:
            app.close()


# ──────────────────────────────────────────────────────────────
# DRIVER + HEARTBEAT
# ──────────────────────────────────────────────────────────────
class Driver:
    """Simpan state perintah, heartbeat ping tiap 0,2 s (watchdog Uno 1 s),
    drive penuh tiap 1 s untuk koreksi bila ada perintah hilang."""

    def __init__(self, link):
        self.link = link
        self._lock = threading.Lock()
        self._state = {"mk": 0, "mr": 0, "ms": 0, "sv": SERVO_CENTER, "ct": 0}
        self._active = False
        threading.Thread(target=self._heartbeat, daemon=True, name="heartbeat").start()

    def drive(self, mk, mr, sv, ct):
        st = {"mk": int(mk), "mr": int(mr), "ms": MOTOR_ON if (mk or mr) else 0,
              "sv": int(max(SERVO_LEFT, min(SERVO_RIGHT, sv))), "ct": 1 if ct else 0}
        with self._lock:
            self._state = st
            self._active = bool(st["mk"] or st["mr"] or st["ct"])
        return self.link.send({"cmd": "drive", **st})

    def halt(self, bursts=3):
        with self._lock:
            self._state = {"mk": 0, "mr": 0, "ms": 0, "sv": self._state["sv"], "ct": 0}
            self._active = False
        ok = False
        for i in range(bursts):
            ok = self.link.send({"cmd": "stop"}) or ok
            if i < bursts - 1:
                time.sleep(0.03)
        return ok

    def _heartbeat(self):
        last_full = 0.0
        while True:
            time.sleep(HEARTBEAT_S)
            with self._lock:
                act = self._active
                st = dict(self._state)
            if not act:
                continue
            now = time.monotonic()
            if now - last_full >= 1.0:
                self.link.send({"cmd": "drive", **st})
                last_full = now
            else:
                self.link.send({"cmd": "ping"})


# ──────────────────────────────────────────────────────────────
# POSE (x, y [m], yaw [rad]: 0 = +Y, positif = searah jarum jam)
# ──────────────────────────────────────────────────────────────
class RobotState:
    def __init__(self):
        self._lock = threading.Lock()
        self.x = self.y = self.yaw = 0.0
        self.steer = 0            # -1 kiri, 0 lurus, +1 kanan (untuk gambar)
        self.cut = 0
        self.traj = deque(maxlen=30000)          # (x, y, cutter)
        self.traj.append((0.0, 0.0, 0))

    def reset(self):
        with self._lock:
            self.x = self.y = self.yaw = 0.0
            self.steer = 0
            self.cut = 0
            self.traj.clear()
            self.traj.append((0.0, 0.0, 0))

    def set_cmd(self, steer, cut):
        with self._lock:
            self.steer, self.cut = steer, cut

    def integrate(self, v, omega, dt):
        with self._lock:
            ym = self.yaw + omega * dt / 2.0
            self.x += v * dt * math.sin(ym)
            self.y += v * dt * math.cos(ym)
            self.yaw += omega * dt
            lx, ly, lc = self.traj[-1]
            if math.hypot(self.x - lx, self.y - ly) >= 0.01 or lc != self.cut:
                self.traj.append((self.x, self.y, self.cut))

    def snapshot(self):
        with self._lock:
            return self.x, self.y, self.yaw, self.steer, self.cut, list(self.traj)


# ──────────────────────────────────────────────────────────────
# PATH PLANNING
# ──────────────────────────────────────────────────────────────
def plan_path(area_w, area_h, row_sp):
    """Kembalikan (segmen, polyline_rencana, info) atau raise ValueError.
    segmen: list dict {type: 'lane'|'turn'|'gap', ...}"""
    D = 2.0 * R_TURN
    if row_sp < D - 1e-6:
        raise ValueError(f"RS {row_sp:.2f} m < diameter U-turn {D:.2f} m (robot tidak bisa)")
    n = int(area_w / row_sp + 1e-9) + 1
    gap = row_sp - D
    segs = []
    for i in range(n):
        segs.append({"type": "lane", "row": i, "len": area_h})
        if i < n - 1:
            side = +1 if i % 2 == 0 else -1          # atas: kanan, bawah: kiri
            if gap < 1e-3:
                segs.append({"type": "turn", "side": side, "deg": 180.0, "row": i})
            else:
                segs.append({"type": "turn", "side": side, "deg": 90.0, "row": i})
                segs.append({"type": "gap", "len": gap, "row": i})
                segs.append({"type": "turn", "side": side, "deg": 90.0, "row": i})

    # polyline rencana (kinematik ideal)
    pts, x, y, yaw = [(0.0, 0.0)], 0.0, 0.0, 0.0
    polys = []                                  # (warna, np.array titik) per segmen
    for s in segs:
        start = len(pts) - 1
        if s["type"] in ("lane", "gap"):
            x += s["len"] * math.sin(yaw)
            y += s["len"] * math.cos(yaw)
            pts.append((x, y))
        else:
            steps = max(8, int(s["deg"] / 5))
            dth = math.radians(s["deg"]) * s["side"] / steps
            for _ in range(steps):
                chord = 2 * R_TURN * math.sin(abs(dth) / 2)
                ym = yaw + dth / 2
                x += chord * math.sin(ym)
                y += chord * math.cos(ym)
                yaw += dth
                pts.append((x, y))
        polys.append((COL_CUT if s["type"] == "lane" else COL_TURN, np.array(pts[start:])))

    t_total = 0.0
    for s in segs:
        if s["type"] in ("lane", "gap"):
            t_total += s["len"] / V_FWD + PAUSE_S
        else:
            t_total += T_UTURN[s["side"]] * s["deg"] / 180.0 + PAUSE_S
    info = {"n_lanes": n, "gap": gap, "end": (x, y, yaw), "t_total": t_total, "polys": polys}
    return segs, np.array(pts), info


# ──────────────────────────────────────────────────────────────
# MISI
# ──────────────────────────────────────────────────────────────
class MissionAborted(Exception):
    pass


class Mission:
    def __init__(self, link, driver, state, segs, hw, logger, on_done):
        self.link, self.driver, self.state = link, driver, state
        self.segs = list(segs)
        self.hw = hw
        self.logger = logger
        self.on_done = on_done
        self.abort_evt = threading.Event()
        self.phase = "IDLE"
        self.seg_idx = 0
        self.t_start = time.monotonic()
        self.thread = threading.Thread(target=self._run, daemon=True, name="mission")

    def start(self):
        self.thread.start()

    def abort(self):
        self.abort_evt.set()

    def running(self):
        return self.thread.is_alive()

    def _check(self):
        if self.abort_evt.is_set():
            raise MissionAborted("dihentikan operator")
        if self.hw:
            if not self.link.robot_online:
                raise MissionAborted("robot OFFLINE")
            if self.link.watchdog_active():
                raise MissionAborted("watchdog Uno aktif")

    def _wait(self, sec):
        end = time.monotonic() + (sec if self.hw else sec / SIM_SPEEDUP)
        while time.monotonic() < end:
            self._check()
            time.sleep(0.02)

    def _stop(self, sv):
        self.state.set_cmd(0, 0)
        if self.hw:
            self.driver.drive(0, 0, sv, 0)

    def _move(self, dur, v, omega, sv, steer, cut, label):
        """Satu gerak dari diam: kirim servo+motor bersamaan (sama seperti
        tombol dashboard saat kalibrasi), jalan 'dur' detik, lalu berhenti."""
        self.phase = label
        self.state.set_cmd(steer, cut)
        if self.hw:
            self.driver.drive(1, 1, sv, cut)
        el, t_prev = 0.0, time.monotonic()
        try:
            while el < dur:
                time.sleep(0.02)
                now = time.monotonic()
                dt = (now - t_prev) * (1.0 if self.hw else SIM_SPEEDUP)
                t_prev = now
                dt = min(dt, dur - el)
                el += dt
                self.state.integrate(v, omega, dt)
                self.logger(self, sv, cut)
                self._check()
        finally:
            self._stop(sv)
        self.phase = "PAUSE"
        self._wait(PAUSE_S)

    def _run(self):
        tag = "HW" if self.hw else "SIM"
        result = "selesai"
        try:
            self.state.reset()
            self._wait(0.3)
            for i, s in enumerate(self.segs):
                self.seg_idx = i
                self._check()
                if s["type"] == "lane":
                    push_status(f"[{tag}] Lajur {s['row'] + 1}: {s['len']:.2f} m, cutter ON")
                    self._move(s["len"] / V_FWD, V_FWD, 0.0, SERVO_CENTER, 0, 1,
                               f"LAJUR {s['row'] + 1}")
                elif s["type"] == "gap":
                    push_status(f"[{tag}] Maju sela {s['len']:.2f} m")
                    self._move(s["len"] / V_FWD, V_FWD, 0.0, SERVO_CENTER, 0, 0, "SELA")
                else:
                    side = s["side"]
                    T = T_UTURN[side] * s["deg"] / 180.0
                    omega = side * math.pi / T_UTURN[side]
                    v = abs(omega) * R_TURN
                    sv = SERVO_RIGHT if side > 0 else SERVO_LEFT
                    name = "KANAN" if side > 0 else "KIRI"
                    push_status(f"[{tag}] Belok {name} {s['deg']:.0f} deg ({T:.2f} s)")
                    self._move(T, v, omega, sv, side, 0, f"BELOK {name} {s['deg']:.0f}")
            self.seg_idx = len(self.segs)
            push_status(f"[{tag}] Selesai! Waktu {time.monotonic() - self.t_start:.1f} s")
        except MissionAborted as e:
            result = f"dibatalkan: {e}"
            push_status(f"[{tag}] DIBATALKAN - {e}")
        except Exception as e:
            result = f"error: {e}"
            push_status(f"[{tag}] ERROR - {e}")
        finally:
            self.phase = "SELESAI" if result == "selesai" else "BATAL"
            if self.hw:
                self.driver.halt()
            self.on_done(result)


# ──────────────────────────────────────────────────────────────
# GUI (tema terang gaya NISA)
# ──────────────────────────────────────────────────────────────
class ElitaGUI:
    def __init__(self, link, driver, state):
        self.link, self.driver, self.state = link, driver, state
        self.mission = None
        self.mode = "IDLE"
        self.ip = DEFAULT_IP
        self.area_w, self.area_h, self.row_sp = AREA_W_DEF, AREA_H_DEF, ROW_SP_DEF
        self.segs, self.plan_pts, self.info = [], np.zeros((1, 2)), {}
        self.log_on = False
        self.log_rows = []
        self.log_t0 = 0.0

        self.h_sv = deque([SERVO_CENTER] * CHART_LEN, maxlen=CHART_LEN)
        self.h_mot = deque([0] * CHART_LEN, maxlen=CHART_LEN)
        self.h_ct = deque([0] * CHART_LEN, maxlen=CHART_LEN)

        self.fig = plt.figure(figsize=(14, 8))
        self.fig.canvas.manager.set_window_title("ELITA - Coverage Path Test v5")
        self.ax = self.fig.add_axes([0.03, 0.05, 0.55, 0.68])
        self.ax.set_aspect("equal")

        self.status_txt = self.fig.text(0.03, 0.965, "ROBOT STATUS: IDLE", fontsize=11,
                                        fontweight="bold", va="center")

        self._build_controls()

        self.ax_term = self.fig.add_axes([0.61, 0.52, 0.37, 0.44])
        self.ax_term.axis("off")
        self.term = self.ax_term.text(0, 1, "", va="top", ha="left", fontsize=8.5, color="#7ee787",
                                      family="monospace", transform=self.ax_term.transAxes)

        xs = np.arange(CHART_LEN)
        self.ax_sv = self.fig.add_axes([0.61, 0.37, 0.37, 0.11])
        self.ax_sv.set_title("Servo steering (deg) - telemetri HW / perintah SIM", fontsize=9)
        self.ln_sv, = self.ax_sv.plot(xs, list(self.h_sv), "-", color="#3fb950", lw=1.5)
        self.ax_sv.set_ylim(SERVO_LEFT - 5, SERVO_RIGHT + 5)
        self.ax_mot = self.fig.add_axes([0.61, 0.21, 0.37, 0.11])
        self.ax_mot.set_title("Motor (1 = maju, 0 = stop)", fontsize=9)
        self.ln_mot, = self.ax_mot.plot(xs, list(self.h_mot), "-", color="#58a6ff", lw=1.5)
        self.ax_mot.set_ylim(-1.2, 1.2)
        self.ax_ct = self.fig.add_axes([0.61, 0.05, 0.37, 0.11])
        self.ax_ct.set_title("Cutter (1 = ON)", fontsize=9)
        self.ln_ct, = self.ax_ct.plot(xs, list(self.h_ct), "-", color="#ff7b72", lw=1.5)
        self.ax_ct.set_ylim(-0.2, 1.3)
        for a in (self.ax_sv, self.ax_mot, self.ax_ct):
            a.set_xlim(0, CHART_LEN - 1)
            a.set_xticklabels([])
            a.grid(True, linestyle="--", color=GRID_C)
            a.tick_params(labelsize=7)

        self._replan(initial=True)

        self.fig.canvas.mpl_connect("close_event", self._on_close)
        self.anim = FuncAnimation(self.fig, self._update, interval=50,
                                  blit=False, cache_frame_data=False)

    # ── kontrol ─────────────────────────────────────────────
    def _build_controls(self):
        def btn(rect, label, color, cb):
            b = Button(self.fig.add_axes(rect), label, color=BG_BTN, hovercolor=BG_HOVER)
            b.label.set_fontsize(10)
            b.label.set_color(color)
            b.label.set_fontweight("bold")
            b.on_clicked(cb)
            return b

        self.b_start = btn([0.03, 0.87, 0.10, 0.05], "START", "#3fb950", self._cb_start)
        self.b_stop = btn([0.14, 0.87, 0.10, 0.05], "STOP", "#f85149", self._cb_stop)
        self.b_reset = btn([0.25, 0.87, 0.10, 0.05], "RESET", "#d29922", self._cb_reset)
        self.b_log = btn([0.36, 0.87, 0.10, 0.05], "LOG: OFF", "#8b949e", self._cb_log)

        def tb(rect, label, init):
            t = TextBox(self.fig.add_axes(rect), label, initial=init,
                        color=BG_BTN, hovercolor=BG_HOVER)
            t.label.set_fontsize(10)
            t.label.set_color("white")
            t.text_disp.set_color("white")
            t.on_submit(lambda _t: self._cb_apply(None))
            t.on_text_change(lambda _t: self._live_apply())   # update peta saat mengetik
            return t

        self.tb_w = tb([0.06, 0.80, 0.07, 0.045], "W:", str(AREA_W_DEF))
        self.tb_h = tb([0.17, 0.80, 0.07, 0.045], "H:", str(AREA_H_DEF))
        self.fig.text(0.26, 0.822, f"meter  |  jarak lajur {ROW_SP_DEF:.2f} m (= diameter U-turn)  |  "
                      "peta update otomatis saat mengetik", fontsize=8, color="#8b949e", va="center")

    def _read_config(self):
        try:
            w, h = float(self.tb_w.text), float(self.tb_h.text)
        except ValueError:
            push_status("[CFG] W/H harus angka")
            return False
        if not (0 < w <= 50 and 0.3 <= h <= 50):
            push_status("[CFG] W 0-50 m, H 0.3-50 m")
            return False
        self.area_w, self.area_h, self.row_sp = w, h, ROW_SP_DEF
        return True

    def _replan(self, initial=False):
        try:
            segs, pts, info = plan_path(self.area_w, self.area_h, self.row_sp)
        except ValueError as e:
            push_status(f"[PLAN] {e}")
            return False
        self.segs, self.plan_pts, self.info = segs, pts, info
        self.state.reset()
        self._draw_map()
        n_turn = sum(1 for s in segs if s["type"] == "turn")
        push_status(f"[PLAN] {info['n_lanes']} lajur, {n_turn} belokan, "
                    f"durasi rencana {info['t_total']:.1f} s")
        if info["gap"] > 1e-3:
            push_status(f"[PLAN] Belokan Pi: 90 + maju {info['gap']:.2f} m + 90")
        return True

    def _busy(self):
        return self.mission is not None and self.mission.running()

    def _start(self, hw):
        if self._busy():
            push_status("[GUI] Misi masih berjalan")
            return
        if not self._read_config() or not self._replan():
            return
        if hw:
            if not self.link.robot_online:
                push_status("[GUI] Robot belum ONLINE (cek WiFi ElitaRobot & badge UNO)")
                return
            if self.link.watchdog_active():
                push_status("[GUI] Watchdog Uno aktif -> tekan STOP dulu")
                return
        self.mode = "HW" if hw else "SIM"
        self.log_rows = []
        self.log_t0 = time.monotonic()
        self.mission = Mission(self.link, self.driver, self.state, self.segs, hw,
                               self._log_sample, self._on_done)
        push_status(f"[GUI] START {self.mode}")
        self.mission.start()

    def _on_done(self, result):
        self.mode = "SELESAI" if result == "selesai" else "STOP"
        if self.log_on:
            self._save_log()

    def _cb_start(self, e):
        # Robot ONLINE -> jalankan robot sungguhan; tidak terhubung -> simulasi
        hw = self.link.robot_online
        if not hw:
            push_status("[GUI] Robot tidak terhubung -> mode SIMULASI")
        self._start(hw)

    def _cb_stop(self, e):
        # STOP = berhenti total (motor + cutter mati, reset latch watchdog Uno)
        if self.mission:
            self.mission.abort()
        self.driver.halt(bursts=5)
        self.mode = "STOP"
        push_status("[GUI] STOP")

    def _cb_reset(self, e):
        if self._busy():
            push_status("[GUI] Stop dulu sebelum reset")
            return
        if self._read_config():
            self._replan()
        self.mode = "IDLE"
        push_status("[GUI] Reset: posisi (0, 0), peta diperbarui")

    def _live_apply(self):
        """Dipanggil setiap isi W/H berubah: replan diam-diam bila angkanya valid."""
        if self._busy() or not hasattr(self, "tb_h"):
            return
        try:
            w, h = float(self.tb_w.text), float(self.tb_h.text)
        except ValueError:
            return                                  # masih mengetik (mis. "2.")
        if not (0 < w <= 50 and 0.3 <= h <= 50):
            return
        if (w, h) == (self.area_w, self.area_h):
            return
        self.area_w, self.area_h, self.row_sp = w, h, ROW_SP_DEF
        self._replan()
        self.fig.canvas.draw_idle()

    def _cb_apply(self, e):
        if self._busy():
            push_status("[GUI] Stop dulu sebelum APPLY")
            return
        if self._read_config():
            self._replan()

    def _cb_log(self, e):
        self.log_on = not self.log_on
        self.b_log.label.set_text("LOG: ON" if self.log_on else "LOG: OFF")
        self.b_log.label.set_color("#ff7b72" if self.log_on else "#8b949e")
        push_status(f"[LOG] {'ON - disimpan saat misi selesai/stop' if self.log_on else 'OFF'}")

    def _on_close(self, e):
        if self.mission:
            self.mission.abort()
        self.driver.halt()
        self.link.close()

    # ── log ────────────────────────────────────────────────
    def _log_sample(self, mission, sv_cmd, cut):
        if not self.log_on:
            return
        x, y, yaw, *_ = self.state.snapshot()
        tel = self.link.get_tel()
        self.log_rows.append([round(time.monotonic() - self.log_t0, 3), mission.phase,
                              round(x, 4), round(y, 4), round(math.degrees(yaw), 2),
                              sv_cmd, tel.get("sv"), tel.get("mk"), tel.get("mr"),
                              cut, tel.get("ct")])

    def _save_log(self):
        if not self.log_rows:
            return
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        fcsv, fpng = f"elita_log_{ts}.csv", f"elita_log_{ts}.png"
        try:
            with open(fcsv, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["t_s", "fase", "x_m", "y_m", "yaw_deg", "servo_cmd",
                            "servo_uno", "mk_uno", "mr_uno", "cutter_cmd", "cutter_uno"])
                w.writerows(self.log_rows)
            d = np.array([[r[0], r[2], r[3], r[5], r[9]] for r in self.log_rows], dtype=float)
            fl, axs = plt.subplots(1, 2, figsize=(13, 5.5))
            fl.suptitle(f"ELITA Log {ts} ({self.mode})", fontweight="bold")
            for col, pp in self.info["polys"]:
                axs[0].plot(pp[:, 0], pp[:, 1], "-", color=col, lw=5, alpha=0.25)
            cut = d[:, 4] > 0
            axs[0].plot(np.where(cut, d[:, 1], np.nan), np.where(cut, d[:, 2], np.nan), "-",
                        color=COL_CUT, lw=1.5, label="Lurus / cutter ON")
            axs[0].plot(np.where(~cut, d[:, 1], np.nan), np.where(~cut, d[:, 2], np.nan), "-",
                        color=COL_TURN, lw=1.5, label="Belok")
            axs[0].add_patch(patches.Rectangle((0, 0), self.area_w, self.area_h, fill=False,
                                               ls="--", color="#8b949e"))
            axs[0].set_aspect("equal")
            axs[0].legend()
            axs[0].grid(alpha=0.4)
            axs[1].plot(d[:, 0], d[:, 3], "g-", label="Servo cmd (deg)")
            axs[1].plot(d[:, 0], d[:, 4] * 100, "r-", label="Cutter x100")
            axs[1].set_xlabel("t (s)")
            axs[1].legend()
            axs[1].grid(alpha=0.4)
            fl.savefig(fpng, dpi=130, facecolor="black")
            plt.close(fl)
            push_status(f"[LOG] Disimpan: {fcsv}, {fpng}")
        except Exception as e:
            push_status(f"[LOG ERR] {e}")
        self.log_rows = []

    # ── gambar ─────────────────────────────────────────────
    def _draw_map(self):
        ax = self.ax
        ax.cla()
        R = R_TURN
        W, H = self.area_w, self.area_h
        xs = [0, W] + list(self.plan_pts[:, 0])
        ys = [0, H] + list(self.plan_pts[:, 1])
        m = 0.4
        ax.set_xlim(min(xs) - m, max(xs) + m)
        ax.set_ylim(min(ys) - m, max(ys) + m)
        ax.set_aspect("equal")
        for gx in np.arange(math.floor(min(xs) - m), max(xs) + m + 1, 1.0):
            ax.axvline(gx, color=GRID_C, lw=0.6, zorder=0)
        for gy in np.arange(math.floor(min(ys) - m), max(ys) + m + 1, 1.0):
            ax.axhline(gy, color=GRID_C, lw=0.6, zorder=0)
        ax.add_patch(patches.Rectangle((0, 0), W, H, fill=False, ls="--", color="#8b949e", lw=1.2))
        ax.axhspan(H, H + R, color="orange", alpha=0.10, zorder=0)
        ax.axhspan(-R, 0, color="orange", alpha=0.10, zorder=0)
        for col, pp in self.info["polys"]:
            ax.plot(pp[:, 0], pp[:, 1], "-", color=col, lw=6, alpha=0.22, solid_capstyle="round")
        ax.plot([], [], "-", color=COL_CUT, lw=4, label="Lurus - cutter ON (dipotong)")
        ax.plot([], [], "-", color=COL_TURN, lw=4, label="Belok / U-turn (tidak dipotong)")
        ex, ey, _ = self.info["end"]
        ax.plot([0], [0], "^", color="white", ms=10, label="Start")
        ax.plot([ex], [ey], "r*", ms=15, label="Target akhir")
        self.traj_cut, = ax.plot([], [], "-", color=COL_CUT, lw=2.2, alpha=0.95)
        self.traj_turn, = ax.plot([], [], "-", color=COL_TURN, lw=2.2, alpha=0.95)
        ax.plot([], [], "-", color="white", lw=2, label="Garis tipis = jejak robot (odometri)")
        hw = BODY_W / 2
        body = np.array([[-hw, -REAR_OVERHANG], [hw, -REAR_OVERHANG],
                         [hw, BODY_LEN - REAR_OVERHANG], [-hw, BODY_LEN - REAR_OVERHANG]])
        self.body_local = body
        self.body = patches.Polygon(body, closed=True, facecolor="#9aa4b2", alpha=0.85, zorder=10)
        ax.add_patch(self.body)
        wl = np.array([[-WHEEL_W / 2, -WHEEL_D / 2], [WHEEL_W / 2, -WHEEL_D / 2],
                       [WHEEL_W / 2, WHEEL_D / 2], [-WHEEL_W / 2, WHEEL_D / 2]])
        self.wheel_local = wl
        ht = TRACK / 2
        self.wheel_centers = [(-ht, 0.0), (ht, 0.0), (-ht, WHEELBASE), (ht, WHEELBASE)]
        self.wheels = [patches.Polygon(wl.copy(), closed=True, facecolor="#e6edf3", zorder=11)
                       for _ in self.wheel_centers]
        for p in self.wheels:
            ax.add_patch(p)
        self.cut_dot = patches.Circle((0, 0), 0.06, color="red", alpha=0.0, zorder=12)
        ax.add_patch(self.cut_dot)
        ax.set_title(f"Coverage Path - {self.info['n_lanes']} lajur, RS {self.row_sp:.2f} m, "
                     f"area {W:.1f} x {H:.1f} m  (zona oranye = ruang belok di luar area)",
                     fontsize=9)
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.legend(loc="upper right", fontsize=7, facecolor="#161b22", edgecolor="#30363d")

    def _update(self, frame):
        x, y, yaw, steer, cut, traj = self.state.snapshot()
        tel = self.link.get_tel()
        hw_mode = self.mode == "HW" or (self.mission and self.mission.hw and self._busy())

        # robot (Rotasi: yaw 0 = +Y, positif searah jarum jam)
        rot = -yaw
        t = transforms.Affine2D().rotate(rot).translate(x, y) + self.ax.transData
        self.body.set_transform(t)
        c, s = math.cos(rot), math.sin(rot)
        steer_rad = -math.radians(STEER_DRAW_DEG) * steer
        for i, (cx, cy) in enumerate(self.wheel_centers):
            wx = x + c * cx - s * cy
            wy = y + s * cx + c * cy
            r = rot + (steer_rad if i >= 2 else 0.0)
            M = np.array([[math.cos(r), -math.sin(r)], [math.sin(r), math.cos(r)]])
            self.wheels[i].set_xy(self.wheel_local @ M.T + np.array([wx, wy]))
        self.cut_dot.center = (x + math.sin(yaw) * 0.12, y + math.cos(yaw) * 0.12)
        self.cut_dot.set_alpha(0.8 if cut else 0.0)
        if traj:
            cx, cy, tx2, ty2 = [], [], [], []
            for k in range(1, len(traj)):
                x0, y0, _ = traj[k - 1]
                x1, y1, c1 = traj[k]
                if c1:
                    cx += [x0, x1, np.nan]; cy += [y0, y1, np.nan]
                else:
                    tx2 += [x0, x1, np.nan]; ty2 += [y0, y1, np.nan]
            self.traj_cut.set_data(cx, cy)
            self.traj_turn.set_data(tx2, ty2)

        # grafik
        if hw_mode:
            sv_v, mot_v, ct_v = tel.get("sv", SERVO_CENTER), tel.get("mk", 0), tel.get("ct", 0)
        else:
            m = self.mission
            moving = bool(m and m.running() and m.phase not in ("PAUSE", "IDLE"))
            sv_v = SERVO_CENTER + (SERVO_RIGHT - SERVO_CENTER if steer > 0 else
                                   (SERVO_LEFT - SERVO_CENTER if steer < 0 else 0))
            mot_v, ct_v = (1 if moving else 0), cut
        self.h_sv.append(sv_v)
        self.h_mot.append(mot_v)
        self.h_ct.append(ct_v)
        self.ln_sv.set_ydata(list(self.h_sv))
        self.ln_mot.set_ydata(list(self.h_mot))
        self.ln_ct.set_ydata(list(self.h_ct))

        # status
        online = self.link.robot_online
        m = self.mission
        phase = m.phase if m else "IDLE"
        if self.mode == "HW":
            txt, col = f"ROBOT STATUS: ROBOT {'ONLINE' if online else 'OFFLINE'} | {phase}", \
                       ("#3fb950" if online else "#ff7b72")
        elif self.mode == "SIM":
            txt, col = f"ROBOT STATUS: SIMULASI ({SIM_SPEEDUP:.0f}x lebih cepat) | {phase}", "#58a6ff"
        elif self.mode == "SELESAI":
            txt, col = "ROBOT STATUS: SELESAI - tandai posisi akhir robot di lantai", "#3fb950"
        elif self.mode == "STOP":
            txt, col = "ROBOT STATUS: STOP", "#ff7b72"
        else:
            txt, col = f"ROBOT STATUS: IDLE | WS {'OK' if self.link.connected else '--'} | " \
                       f"UNO {'OK' if online else 'OFF'}  (START = {'ROBOT' if online else 'SIMULASI'})", "white"
        self.status_txt.set_text(txt)
        self.status_txt.set_color(col)

        ex, ey, eyaw = self.info.get("end", (0, 0, 0))
        seg_i = m.seg_idx if m else 0
        lines = [
            f"MODE: {self.mode}   FASE: {phase}",
            f"Segmen: {min(seg_i + 1, len(self.segs))}/{len(self.segs)}   "
            f"Durasi rencana: {self.info.get('t_total', 0):.1f} s",
            "-" * 44,
            f"[POSE ODOMETRI] x={x:.2f} y={y:.2f} m  yaw={math.degrees(yaw) % 360:.0f} deg",
            f"[TARGET AKHIR]  x={ex:.2f} y={ey:.2f} m  yaw={math.degrees(eyaw) % 360:.0f} deg",
            f"[UNO] servo={tel.get('sv')} mk={tel.get('mk')} mr={tel.get('mr')} "
            f"ct={tel.get('ct')} wd={tel.get('wd')} bf={tel.get('bf')}",
            "-" * 44,
        ] + status_snapshot(10)
        self.term.set_text("\n".join(lines))
        return ()

    def show(self):
        plt.show()


# ──────────────────────────────────────────────────────────────
# MAIN
# ──────────────────────────────────────────────────────────────
def main():
    push_status("ELITA Coverage Path Test v5")
    link = RobotLink(ws_url(DEFAULT_IP))
    driver = Driver(link)
    state = RobotState()
    gui = ElitaGUI(link, driver, state)
    gui.show()


if __name__ == "__main__":
    main()