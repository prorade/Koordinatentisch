import tkinter as tk
from tkinter import ttk, messagebox
import threading, queue, time
from math import sqrt

# ==================================================
# Simulation mit grafischer Vorschau (Canvas)
# ==================================================
class SimulatedSerialClient:
    """
    Drop-in-Ersatz für eine serielle Verbindung.
    - Versteht die von deiner GUI verwendeten Befehle
    - Führt Bewegungen in einem Hintergrund-Thread aus (50 Hz)
    - Sendet zyklisch Telemetrie "POS x y z" (10 Hz)
    """
    def __init__(self):
        self.rxq = queue.Queue()
        self._stop = threading.Event()
        self.rx_thread = None
        self.is_open = False

        # Arbeitsraum und Parameter
        self.work_w = 300.0
        self.work_h = 200.0
        self.zmin = 0.0
        self.zmax = 60.0
        self.dx = 10.0
        self.dy = 10.0
        self.sx = 50.0
        self.sy = 30.0

        # Zustände
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.ox = 0.0
        self.oy = 0.0
        self.oz = 0.0
        self.pick = (0.0, 0.0, 0.0)

        # Bewegung
        self._targets = []  # Liste (x,y,z)
        self._busy = False
        self._abort = False
        self.speed_xy = 150.0  # mm/s
        self.speed_z = 60.0    # mm/s
        self._motion_lock = threading.Lock()

    # --------------------- Serial-ähnliche API ---------------------
    def ports(self):
        return ["SIMULATOR"]

    def connect(self, port):
        self.close()
        if port != "SIMULATOR":
            raise ValueError("Nur SIMULATOR-Port verfügbar")
        self.is_open = True
        self._stop.clear()
        # Start Threads: Motion, Telemetry, Dummy-RX
        threading.Thread(target=self._motion_loop, daemon=True).start()
        threading.Thread(target=self._telemetry_loop, daemon=True).start()
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()
        self._send("SIM READY | Port=SIMULATOR | tip: 'H' für Hilfe")

    def close(self):
        self._stop.set()
        if self.rx_thread and self.rx_thread.is_alive():
            self.rx_thread.join(timeout=0.3)
        self.rx_thread = None
        self.is_open = False

    def send_line(self, s: str):
        if not self.is_open:
            return
        if not s.endswith("\n"):
            s += "\n"
        self._handle_command(s.strip())

    # --------------------- Hilfsfunktionen ---------------------
    def _send(self, line: str):
        self.rxq.put(line)

    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))

    # --------------------- Command-Parser ---------------------
    def _handle_command(self, s: str):
        self._send(f"< {s}")  # Echo
        parts = s.split()
        if not parts:
            return
        cmd = parts[0].upper()
        arg = parts[1:]

        try:
            if cmd == 'H':
                self._send("H: H,P, RX d, RY d, RZ d, X v, Y v, Z v, O,OZ,O0, R,RZ0,R0, PSET, P x y z, P?, PEXY, PE, dx v, dy v, sx v, sy v, S, !, ZMAX v")
            elif cmd == 'P':
                self._send(f"PARAM dx={self.dx} dy={self.dy} sx={self.sx} sy={self.sy} zmax={self.zmax}")
                self._send(f"ORIGIN {self.ox:.3f} {self.oy:.3f} {self.oz:.3f}")
                self._send(f"PICK {self.pick[0]:.3f} {self.pick[1]:.3f} {self.pick[2]:.3f}")
                self._send(f"POS {self.x:.3f} {self.y:.3f} {self.z:.3f}")
            elif cmd in ('RX','RY','RZ'):
                d = float(arg[0]) if arg else 0.0
                dx = d if cmd=='RX' else 0.0
                dy = d if cmd=='RY' else 0.0
                dz = d if cmd=='RZ' else 0.0
                self._enqueue_rel(dx, dy, dz)
                self._send(f"OK MOVE_REL {dx} {dy} {dz}")
            elif cmd in ('X','Y','Z'):
                v = float(arg[0]) if arg else 0.0
                self._enqueue_abs(x=v if cmd=='X' else None,
                                   y=v if cmd=='Y' else None,
                                   z=v if cmd=='Z' else None)
            elif cmd == 'O':
                self.ox, self.oy = self.x, self.y
                self._send("OK ORIGIN XY gesetzt")
            elif cmd == 'OZ':
                self.oz = self.z
                self._send("OK ORIGIN Z gesetzt")
            elif cmd == 'O0':
                self.ox, self.oy, self.oz = self.x, self.y, self.z
                self._send("OK ORIGIN XYZ gesetzt")
            elif cmd == 'R':
                self._enqueue_abs(x=self.ox, y=self.oy, z=None)
            elif cmd == 'RZ0':
                self._enqueue_abs(x=None, y=None, z=self.oz)
            elif cmd == 'R0':
                self._enqueue_abs(x=self.ox, y=self.oy, z=self.oz)
            elif cmd == 'PSET':
                self.pick = (self.x, self.y, self.z)
                self._send("OK PICK=CURR XYZ")
            elif cmd == 'P?':
                self._send(f"PICK {self.pick[0]:.3f} {self.pick[1]:.3f} {self.pick[2]:.3f}")
            elif cmd == 'P':
                if len(arg) >= 3:
                    px,py,pz = map(float, arg[:3])
                    self.pick = (self._clamp(px,0,self.work_w), self._clamp(py,0,self.work_h), self._clamp(pz,self.zmin,self.zmax))
                    self._send("OK PICK gesetzt")
                else:
                    self._send("ERR Nutzung: P x y z")
            elif cmd == 'PEXY':
                self._enqueue_abs(x=self.pick[0], y=self.pick[1], z=None)
            elif cmd == 'PE':
                self._enqueue_abs(x=self.pick[0], y=self.pick[1], z=self.pick[2])
            elif cmd in ('DX','DY','SX','SY'):
                v = float(arg[0]) if arg else 0.0
                if cmd == 'DX': self.dx = max(0.1, v)
                if cmd == 'DY': self.dy = max(0.1, v)
                if cmd == 'SX': self.sx = max(0.1, v)
                if cmd == 'SY': self.sy = max(0.1, v)
                self._send(f"OK {cmd}={v}")
            elif cmd == 'ZMAX':
                if arg:
                    self.zmax = max(self.zmin, float(arg[0]))
                    self._send(f"OK ZMAX={self.zmax}")
                else:
                    self._send(f"ZMAX {self.zmax}")
            elif cmd == 'S':
                self._start_scan()
            elif cmd == '!':
                with self._motion_lock:
                    self._abort = True
                    self._targets.clear()
                    self._busy = False
                self._send("ABORT ok")
            else:
                self._send(f"ERR unbekannt: {s}")
        except Exception as ex:
            self._send(f"ERR {type(ex).__name__}: {ex}")

    # --------------------- Bewegungsplanung ---------------------
    def _enqueue_abs(self, x=None, y=None, z=None):
        with self._motion_lock:
            tx = self.x if x is None else self._clamp(x, 0, self.work_w)
            ty = self.y if y is None else self._clamp(y, 0, self.work_h)
            tz = self.z if z is None else self._clamp(z, self.zmin, self.zmax)
            self._targets.append((tx, ty, tz))
            self._busy = True
        self._send(f"MOVE {tx:.3f} {ty:.3f} {tz:.3f}")

    def _enqueue_rel(self, dx=0.0, dy=0.0, dz=0.0):
        self._enqueue_abs(self.x + dx, self.y + dy, self.z + dz)

    def _start_scan(self):
        with self._motion_lock:
            self._targets.clear()
            x0, y0 = self.ox, self.oy
            x1 = self._clamp(x0 + self.sx, 0, self.work_w)
            y1 = self._clamp(y0 + self.sy, 0, self.work_h)
            # zum Startpunkt
            self._targets.append((self._clamp(x0,0,self.work_w), self._clamp(y0,0,self.work_h), self.z))
            toggle = False
            y = y0
            step_y = max(0.1, self.dy)
            step_x = max(0.1, self.dx)
            while y <= y1 + 1e-9:
                if not toggle:
                    x = x0
                    while x <= x1 + 1e-9:
                        self._targets.append((self._clamp(x,0,self.work_w), self._clamp(y,0,self.work_h), self.z))
                        x += step_x
                else:
                    x = x1
                    while x >= x0 - 1e-9:
                        self._targets.append((self._clamp(x,0,self.work_w), self._clamp(y,0,self.work_h), self.z))
                        x -= step_x
                toggle = not toggle
                y += step_y
            self._busy = True
        self._send(f"SCAN start dx={self.dx} dy={self.dy} sx={self.sx} sy={self.sy}")

    # --------------------- Threads ---------------------
    def _motion_loop(self):
        dt = 0.02  # 50 Hz
        while not self._stop.is_set():
            time.sleep(dt)
            with self._motion_lock:
                if self._abort:
                    self._abort = False
                    continue
                if not self._busy or not self._targets:
                    self._busy = False
                    continue
                tx, ty, tz = self._targets[0]
                dx = tx - self.x
                dy = ty - self.y
                dz = tz - self.z
                dist = (dx*dx + dy*dy + dz*dz) ** 0.5
                if dist < 1e-3:
                    self.x, self.y, self.z = tx, ty, tz
                    self._targets.pop(0)
                    self._send(f"AT {self.x:.3f} {self.y:.3f} {self.z:.3f}")
                    if not self._targets:
                        self._busy = False
                        self._send("IDLE")
                    continue
                max_xy = self.speed_xy * dt
                max_z  = self.speed_z * dt
                step = min(max_xy, dist)
                if abs(dz) > 1e-9:
                    ratio = abs(dz) / max(dist, 1e-9)
                    if ratio * step / dt > self.speed_z:
                        step = self.speed_z * dt / max(ratio, 1e-9)
                ux, uy, uz = dx/dist, dy/dist, dz/dist
                self.x = self._clamp(self.x + ux*step, 0, self.work_w)
                self.y = self._clamp(self.y + uy*step, 0, self.work_h)
                self.z = self._clamp(self.z + uz*step, self.zmin, self.zmax)

    def _telemetry_loop(self):
        dt = 0.1  # 10 Hz
        while not self._stop.is_set():
            self._send(f"POS {self.x:.3f} {self.y:.3f} {self.z:.3f}")
            time.sleep(dt)

    def _rx_loop(self):
        while not self._stop.is_set():
            time.sleep(0.05)


# ==================================================
# GUI mit Canvas-Vorschau links + Konsole rechts
# ==================================================
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Koordinatentisch Bedienoberfläche (Simulation + Vorschau)")
        self.geometry("1080x680")
        self.client = SimulatedSerialClient()
        # Visual State
        self.pos = {"x":0.0, "y":0.0, "z":0.0}
        self.path = []
        self.scale = 2.0  # px/mm
        self.pad = 20
        self._build_ui()
        self._poll_rx()
        self._render()
        self.bind_all_hotkeys()

    def _build_ui(self):
        # Top bar
        top = ttk.Frame(self); top.pack(fill="x", padx=8, pady=8)
        ttk.Label(top, text="COM-Port:").pack(side="left")
        self.port_combo = ttk.Combobox(top, width=15, values=self.client.ports())
        self.port_combo.current(0)
        self.port_combo.pack(side="left", padx=4)
        self.connect_btn = ttk.Button(top, text="Verbinden", command=self.toggle_connect)
        self.connect_btn.pack(side="left", padx=8)
        ttk.Button(top, text="Hilfe (H)", command=lambda: self.send("H")).pack(side="left")
        ttk.Button(top, text="Parameter (P)", command=lambda: self.send("P")).pack(side="left", padx=4)

        # Main area: left preview, right console
        main = ttk.Frame(self); main.pack(fill="both", expand=True, padx=8, pady=8)

        # Left: Preview
        left = ttk.Labelframe(main, text="Vorschau"); left.pack(side="left", fill="both", expand=True)
        self.canvas = tk.Canvas(left, bg="#0b1020")
        self.canvas.pack(fill="both", expand=True)

        # Right: Console + Commands
        right = ttk.Labelframe(main, text="Konsole"); right.pack(side="right", fill="both", expand=True, padx=8)
        self.txt = tk.Text(right, height=18)
        self.txt.pack(side="left", fill="both", expand=True)
        sb = ttk.Scrollbar(right, command=self.txt.yview); sb.pack(side="right", fill="y")
        self.txt.configure(yscrollcommand=sb.set)
        self.cmd_entry = ttk.Entry(right)
        self.cmd_entry.pack(fill="x")
        self.cmd_entry.bind("<Return>", lambda e: self.send(self.cmd_entry.get(), clear=True))

        # Footer
        foot = ttk.Frame(self); foot.pack(fill="x", padx=8, pady=4)
        ttk.Label(foot, text="Tastatur: Pfeile=XY, Bild↑/Bild↓=Z, S=Scan, !=Abbruch").pack(side="left")

    def bind_all_hotkeys(self):
        self.bind("<Left>",  lambda e: self.send("RX -1"))
        self.bind("<Right>", lambda e: self.send("RX 1"))
        self.bind("<Up>",    lambda e: self.send("RY 1"))
        self.bind("<Down>",  lambda e: self.send("RY -1"))
        self.bind("<Prior>", lambda e: self.send("RZ 1"))   # PageUp
        self.bind("<Next>",  lambda e: self.send("RZ -1"))  # PageDown
        self.bind("s",       lambda e: self.send("S"))
        self.bind("!",       lambda e: self.send("!"))

    def toggle_connect(self):
        try:
            port = self.port_combo.get().strip()
            self.client.connect(port)
            self.connect_btn.config(text="Trennen")
            self.log(f"Verbunden mit {port}.")
            self.send("H"); self.send("P")
        except Exception as ex:
            messagebox.showerror("Fehler", f"Konnte nicht verbinden:\n{ex}")

    def send(self, line, clear=False):
        if not line:
            return
        self.client.send_line(line)
        self.log(f"> {line}")
        if clear:
            self.cmd_entry.delete(0, "end")

    def _poll_rx(self):
        try:
            while True:
                line = self.client.rxq.get_nowait()
                self._handle_rx_line(line)
        except queue.Empty:
            pass
        self.after(50, self._poll_rx)

    def _handle_rx_line(self, line: str):
        self.log(line)
        try:
            if line.startswith("POS ") or line.startswith("AT "):
                _, xs, ys, zs = line.split(maxsplit=3)
                x = float(xs); y = float(ys); z = float(zs)
                self.pos.update({"x":x, "y":y, "z":z})
                if not self.path or abs(self.path[-1][0]-x)+abs(self.path[-1][1]-y) > 0.5:
                    self.path.append((x,y))
                    if len(self.path) > 4000:
                        self.path = self.path[-2000:]
        except Exception:
            pass

    def log(self, msg):
        self.txt.insert("end", msg + "\n")
        self.txt.see("end")

    def _render(self):
        c = self.canvas
        c.delete("all")
        W = self.client.work_w * self.scale
        H = self.client.work_h * self.scale
        pad = self.pad
        # Arbeitsfläche
        c.create_rectangle(pad, pad, pad+W, pad+H, outline="#546a91")
        # Grid
        grid = 20 * self.scale
        yy = pad
        while yy <= pad+H:
            c.create_line(pad, yy, pad+W, yy, fill="#1a2540")
            yy += grid
        xx = pad
        while xx <= pad+W:
            c.create_line(xx, pad, xx, pad+H, fill="#1a2540")
            xx += grid
        # Pfadspur
        if len(self.path) > 1:
            pts = []
            for (px,py) in self.path[-3000:]:
                pts.append(pad + px*self.scale)
                pts.append(pad + py*self.scale)
            c.create_line(*pts, fill="#3aa1ff")
        # Kopf
        cx = pad + self.pos["x"]*self.scale
        cy = pad + self.pos["y"]*self.scale
        r = 6
        c.create_oval(cx-r, cy-r, cx+r, cy+r, fill="#e0e6f6", outline="")
        # Info
        info = f"X={self.pos['x']:.2f}  Y={self.pos['y']:.2f}  Z={self.pos['z']:.2f}"
        c.create_text(pad, pad+H+16, text=info, anchor='w', fill="#9fb4d8")
        self.after(16, self._render)

    def on_closing(self):
        try:
            self.client.close()
        finally:
            self.destroy()

if __name__ == "__main__":
    app = App()
    app.protocol("WM_DELETE_WINDOW", app.on_closing)
    app.mainloop()
