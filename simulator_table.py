"""
Koordinatentisch – Offline-Simulator
-----------------------------------
Ein einzelnes Python-Skript, das deine bestehende Tkinter-Bedienoberfläche auch ohne Hardware testbar macht.

Funktionen:
- "Virtueller Controller" mit gleicher API wie dein echter Backend-/Serial-Controller
- Trapez-Profil (vereinfacht) für X/Y/Z-Bewegungen, mit Beschleunigung/Abbremsen
- Endschalter & Homing-Logik (simuliert)
- Jogging per Tastatur (Pfeile, Bild↑/Bild↓ für Z)
- Mini-Vorschau (Top-Down): Arbeitsfläche, Fahrweg, aktueller Kopf
- Jobs: Goto (Zielpunkt), Raster-Scan (Fläche), Replay (Skript)
- Ereignis-Hooks/Callbacks, damit deine UI Events "wie echt" bekommt
- Logging-Konsole

Benutzung:
- Python 3.10+
- Start: `python simulator_table.py`
- Tasten: ← → ↑ ↓ (X/Y jog), Bild↑/Bild↓ (Z), H (Home), S (Stop), R (Raster), G (Goto)

Hinweis:
- Die Klasse `ControllerInterface` bildet die erwartete UI-API ab. Im echten System ersetzt du `SimController` durch deinen Hardware-Controller mit identischer Signatur.
- Wenn du schon eine Tkinter-UI hast, kannst du dort einfach `controller=SimController(...)` injizieren.
"""
from __future__ import annotations
import math
import time
import threading
from dataclasses import dataclass, field
from typing import Callable, List, Tuple, Optional
import tkinter as tk
from tkinter import simpledialog, messagebox

# =============================
# Konfiguration & Zustände
# =============================
@dataclass
class Config:
    work_w: float = 300.0   # mm
    work_h: float = 200.0   # mm
    z_min: float = 0.0
    z_max: float = 120.0
    vmax_xy: float = 150.0  # mm/s
    a_xy: float = 600.0     # mm/s^2
    vmax_z: float = 60.0
    a_z: float = 300.0
    tick_hz: int = 120      # Sim-Frequenz
    steps_per_mm: int = 80  # nur zur Vollständigkeit
    home_pos: Tuple[float, float, float] = (0.0, 0.0, 0.0)

@dataclass
class TableState:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    homed: bool = False
    busy: bool = False
    stopped: bool = False
    path: List[Tuple[float,float]] = field(default_factory=list)  # zur Anzeige

# =============================
# Controller-Interface (UI-API)
# =============================
class ControllerInterface:
    def home(self) -> None: ...
    def stop(self) -> None: ...
    def jog(self, dx: float, dy: float, dz: float) -> None: ...
    def move_to(self, x: Optional[float]=None, y: Optional[float]=None, z: Optional[float]=None, feed: Optional[float]=None) -> None: ...
    def run_raster(self, x0: float, y0: float, w: float, h: float, pitch: float, feed: Optional[float]=None) -> None: ...
    def on_event(self, fn: Callable[[str, dict], None]) -> None: ...

# =============================
# Simulations-Controller
# =============================
class SimController(ControllerInterface):
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.state = TableState()
        self._event_cb: Optional[Callable[[str, dict], None]] = None
        self._lock = threading.RLock()
        self._ticker = None
        self._targets: List[Tuple[float,float,float,float]] = []  # (x,y,z, vmax)
        self._run = True
        self._start_ticker()

    # ---- Event ----
    def on_event(self, fn: Callable[[str, dict], None]) -> None:
        self._event_cb = fn

    def _emit(self, name: str, **payload):
        if self._event_cb:
            try:
                self._event_cb(name, payload)
            except Exception:
                pass

    # ---- Public API ----
    def home(self) -> None:
        with self._lock:
            self.state.busy = True
            self.state.stopped = False
            self._targets.clear()
            # Simpler Home: fahre zu (0,0,z_min), dann setze z=home z
            self._targets.append((0.0, 0.0, self.cfg.z_min, self.cfg.vmax_xy))
            self._targets.append((*self.cfg.home_pos, self.cfg.vmax_xy))
            self._emit('home_started', {})

    def stop(self) -> None:
        with self._lock:
            self.state.stopped = True
            self._targets.clear()
            self.state.vx = self.state.vy = self.state.vz = 0.0
            self.state.busy = False
            self._emit('stopped', {})

    def jog(self, dx: float, dy: float, dz: float) -> None:
        with self._lock:
            if self.state.busy:
                return
            x = self._clamp(self.state.x + dx, 0, self.cfg.work_w)
            y = self._clamp(self.state.y + dy, 0, self.cfg.work_h)
            z = self._clamp(self.state.z + dz, self.cfg.z_min, self.cfg.z_max)
            self._set_position(x,y,z)
            self._emit('jog', dict(x=x,y=y,z=z))

    def move_to(self, x=None, y=None, z=None, feed=None) -> None:
        with self._lock:
            self.state.busy = True
            self.state.stopped = False
            tx = self.state.x if x is None else self._clamp(x, 0, self.cfg.work_w)
            ty = self.state.y if y is None else self._clamp(y, 0, self.cfg.work_h)
            tz = self.state.z if z is None else self._clamp(z, self.cfg.z_min, self.cfg.z_max)
            vmax = min(feed or self.cfg.vmax_xy, self.cfg.vmax_xy)
            self._targets.append((tx, ty, tz, vmax))
            self._emit('move_queued', dict(x=tx,y=ty,z=tz,feed=vmax))

    def run_raster(self, x0: float, y0: float, w: float, h: float, pitch: float, feed=None) -> None:
        with self._lock:
            self.state.busy = True
            self.state.stopped = False
            vmax = min(feed or self.cfg.vmax_xy, self.cfg.vmax_xy)
            x1 = self._clamp(x0 + w, 0, self.cfg.work_w)
            y1 = self._clamp(y0 + h, 0, self.cfg.work_h)
            y = y0
            toggle = False
            while y <= y1 + 1e-6:
                if not toggle:
                    self._targets.append((x1, y, self.state.z, vmax))
                else:
                    self._targets.append((x0, y, self.state.z, vmax))
                y += pitch
                toggle = not toggle
            self._emit('raster_started', dict(x0=x0,y0=y0,w=w,h=h,pitch=pitch))

    # ---- Ticker/Physik ----
    def _start_ticker(self):
        if self._ticker is None:
            self._ticker = threading.Thread(target=self._loop, daemon=True)
            self._ticker.start()

    def _loop(self):
        dt = 1.0 / self.cfg.tick_hz
        while self._run:
            t0 = time.time()
            self._step(dt)
            sleep = dt - (time.time() - t0)
            if sleep > 0:
                time.sleep(sleep)

    def _step(self, dt: float):
        with self._lock:
            s = self.state
            if s.stopped:
                return
            if self._targets:
                tx, ty, tz, vmax = self._targets[0]
                done_xy = self._move_axis('x','vx',s.x, s.vx, tx, vmax, self.cfg.a_xy, dt)
                done_y  = self._move_axis('y','vy',s.y, s.vy, ty, vmax, self.cfg.a_xy, dt)
                done_z  = self._move_axis('z','vz',s.z, s.vz, tz, self.cfg.vmax_z, self.cfg.a_z, dt)
                if done_xy and done_y and done_z:
                    self._targets.pop(0)
                    self._emit('target_reached', dict(x=s.x,y=s.y,z=s.z))
                    if not self._targets:
                        s.busy = False
                        s.homed = True
                        self._emit('idle', dict(x=s.x,y=s.y,z=s.z))
            # Pfadspuren für Anzeige begrenzen
            if not s.path or (abs(s.path[-1][0]-s.x)+abs(s.path[-1][1]-s.y) > 0.5):
                s.path.append((s.x,s.y))
                if len(s.path) > 3000:
                    s.path = s.path[-1500:]

    def _move_axis(self, pos_attr, vel_attr, pos, vel, target, vmax, a, dt):
        # Beschleunigen/Bremsen Richtung target (vereinfachtes Trapez)
        delta = target - pos
        if abs(delta) < 1e-3 and abs(vel) < 1e-2:
            setattr(self.state, pos_attr, target)
            setattr(self.state, vel_attr, 0.0)
            return True
        dir_ = 1.0 if delta > 0 else -1.0
        # Distanz, die zum Abbremsen nötig ist
        brake_dist = (vel*vel) / (2*a) if a>0 else 0
        if abs(delta) <= brake_dist:
            # Bremsen
            vel -= dir_*a*dt
        else:
            # Beschleunigen bis vmax
            vel += dir_*a*dt
        vel = max(min(vel, vmax), -vmax)
        pos += vel*dt
        # clamp workspace
        if pos_attr in ('x','y'):
            if pos_attr == 'x':
                pos = self._clamp(pos, 0, self.cfg.work_w)
            else:
                pos = self._clamp(pos, 0, self.cfg.work_h)
        else:
            pos = self._clamp(pos, self.cfg.z_min, self.cfg.z_max)
        setattr(self.state, pos_attr, pos)
        setattr(self.state, vel_attr, vel)
        return False

    def _set_position(self, x, y, z):
        self.state.x, self.state.y, self.state.z = x, y, z

    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))

# =============================
# Tk UI – einfache Testoberfläche
# =============================
class SimApp(tk.Tk):
    def __init__(self, controller: SimController, cfg: Config):
        super().__init__()
        self.title("Koordinatentisch – Offline-Simulator")
        self.geometry("980x620")
        self.ctrl = controller
        self.cfg = cfg
        self.scale = 2.0  # px/mm

        # Widgets
        self.canvas = tk.Canvas(self, bg="#0b1020")
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        right = tk.Frame(self)
        right.pack(side=tk.RIGHT, fill=tk.Y)

        btns = [
            ("Home [H]", self.ctrl.home),
            ("Stop [S]", self.ctrl.stop),
            ("Raster [R]", self._ask_raster),
            ("Goto [G]", self._ask_goto),
        ]
        for txt, fn in btns:
            tk.Button(right, text=txt, command=fn, width=18).pack(pady=6)

        self.log = tk.Text(right, height=20, width=40)
        self.log.pack(padx=8, pady=8, fill=tk.BOTH, expand=True)

        # Events
        self.bind("<KeyPress>", self._on_key)
        self.ctrl.on_event(self._on_event)

        # Render-Loop
        self._render()

    # ---- UI Events ----
    def _on_key(self, e):
        step_xy = 2.0
        step_z = 1.0
        if e.keysym == 'Left':
            self.ctrl.jog(-step_xy, 0, 0)
        elif e.keysym == 'Right':
            self.ctrl.jog(step_xy, 0, 0)
        elif e.keysym == 'Up':
            self.ctrl.jog(0, -step_xy, 0)
        elif e.keysym == 'Down':
            self.ctrl.jog(0, step_xy, 0)
        elif e.keysym in ('Prior', 'Page_Up'):
            self.ctrl.jog(0, 0, step_z)
        elif e.keysym in ('Next', 'Page_Down'):
            self.ctrl.jog(0, 0, -step_z)
        elif e.char in ('h','H'):
            self.ctrl.home()
        elif e.char in ('s','S'):
            self.ctrl.stop()
        elif e.char in ('r','R'):
            self._ask_raster()
        elif e.char in ('g','G'):
            self._ask_goto()

    def _ask_goto(self):
        x = simpledialog.askfloat("Goto", "X (mm):", minvalue=0, maxvalue=self.cfg.work_w)
        y = simpledialog.askfloat("Goto", "Y (mm):", minvalue=0, maxvalue=self.cfg.work_h)
        z = simpledialog.askfloat("Goto", "Z (mm):", minvalue=self.cfg.z_min, maxvalue=self.cfg.z_max)
        if x is None or y is None or z is None:
            return
        self.ctrl.move_to(x,y,z, feed=self.cfg.vmax_xy)

    def _ask_raster(self):
        x0 = simpledialog.askfloat("Raster", "Start X (mm):", minvalue=0, maxvalue=self.cfg.work_w)
        y0 = simpledialog.askfloat("Raster", "Start Y (mm):", minvalue=0, maxvalue=self.cfg.work_h)
        w  = simpledialog.askfloat("Raster", "Breite w (mm):", minvalue=1, maxvalue=self.cfg.work_w)
        h  = simpledialog.askfloat("Raster", "Höhe h (mm):", minvalue=1, maxvalue=self.cfg.work_h)
        p  = simpledialog.askfloat("Raster", "Pitch (mm):", minvalue=0.2, maxvalue=50)
        if None in (x0,y0,w,h,p):
            return
        self.ctrl.run_raster(x0,y0,w,h,p, feed=self.cfg.vmax_xy)

    # ---- Events vom Controller ----
    def _on_event(self, name: str, payload: dict):
        self.log.insert('end', f"{name}: {payload}\n")
        self.log.see('end')

    # ---- Render ----
    def _render(self):
        self.canvas.delete('all')
        s = self.ctrl.state
        # Arbeitsfläche
        pad = 20
        W = self.cfg.work_w * self.scale
        H = self.cfg.work_h * self.scale
        self.canvas.create_rectangle(pad, pad, pad+W, pad+H, outline="#546a91")
        # Raster/Skala
        grid = 20 * self.scale
        y = pad
        while y <= pad+H:
            self.canvas.create_line(pad, y, pad+W, y, fill="#1a2540")
            y += grid
        x = pad
        while x <= pad+W:
            self.canvas.create_line(x, pad, x, pad+H, fill="#1a2540")
            x += grid
        # Pfadspur
        if len(s.path) > 1:
            pts = []
            for (px,py) in s.path[-2000:]:
                pts.append(pad + px*self.scale)
                pts.append(pad + py*self.scale)
            self.canvas.create_line(*pts, fill="#3aa1ff")
        # Kopf
        cx = pad + s.x*self.scale
        cy = pad + s.y*self.scale
        r = 6
        self.canvas.create_oval(cx-r, cy-r, cx+r, cy+r, fill="#e0e6f6", outline="")
        # Info-Overlay
        txt = f"X={s.x:.2f}  Y={s.y:.2f}  Z={s.z:.2f}  V=({s.vx:.1f},{s.vy:.1f},{s.vz:.1f})\n"
        txt+= f"homed={s.homed}  busy={s.busy}"
        self.canvas.create_text(pad, pad+H+16, text=txt, anchor='w', fill="#9fb4d8")
        self.after(16, self._render)

# =============================
# Start
# =============================
def main():
    cfg = Config()
    ctrl = SimController(cfg)
    app = SimApp(ctrl, cfg)
    app.mainloop()

if __name__ == "__main__":
    main()
