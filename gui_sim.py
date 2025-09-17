import tkinter as tk
from tkinter import ttk, messagebox
import threading, queue, time
from math import sqrt

# --- PySerial optional: echte Ports, aber für die Simulation nicht nötig ---
try:
    import serial, serial.tools.list_ports  # type: ignore
except Exception:  # pyserial nicht vorhanden -> nur SIM-Port anbieten
    serial = None  # type: ignore

BAUD = 115200

# =============================================================================
# SIMULIERTER SERIAL-PORT (drop-in Ersatz für pyserial.Serial)
# =============================================================================
class SimSerial:
    """Minimaler Serial-Ersatz mit .read(), .write(), .close(), .is_open
    Implementiert einen Koordinatentisch mit einfacher Bewegungsphysik.
    Alle Antworten sind zeilenbasiert (\n-terminiert) und landen in einem internen RX-Puffer.
    """
    def __init__(self):
        self.is_open = True
        self._rx = bytearray()
        self._rx_lock = threading.Lock()
        self._cmd_buf = bytearray()
        self._stop = threading.Event()
        # Maschinenzustand
        self.work_w = 300.0
        self.work_h = 200.0
        self.zmin = 0.0
        self.zmax = 60.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.ox = 0.0
        self.oy = 0.0
        self.oz = 0.0
        self.pick = (0.0, 0.0, 0.0)
        self.dx = 10.0
        self.dy = 10.0
        self.sx = 50.0
        self.sy = 30.0
        # Bewegung
        self._targets = []  # Liste [(x,y,z), ...]
        self._busy = False
        self._abort = False
        self._motion_lock = threading.Lock()
        self.speed_xy = 150.0  # mm/s
        self.speed_z = 60.0    # mm/s
        # Threads
        self._motion_th = threading.Thread(target=self._motion_loop, daemon=True)
        self._motion_th.start()
        self._telemetry_th = threading.Thread(target=self._telemetry_loop, daemon=True)
        self._telemetry_th.start()
        # Begrüßung
        self._send("SIM READY | Port=SIM | tip: 'H' für Hilfe")

    # ------------- low-level Serial API -------------
    def read(self, n: int) -> bytes:
        # non-blocking Read (SerialClient pollt mit short timeout)
        with self._rx_lock:
            if not self._rx:
                return b""
            out = self._rx[:n]
            del self._rx[:n]
            return bytes(out)

    def write(self, data: bytes):
        # Kommandos sind textuell, \n-terminiert
        self._cmd_buf.extend(data)
        while b"\n" in self._cmd_buf:
            line, self._cmd_buf = self._cmd_buf.split(b"\n", 1)
            try:
                txt = line.decode("utf-8", errors="ignore").strip()
            except Exception:
                txt = ""
            if txt:
                self._handle_command(txt)

    def close(self):
        self.is_open = False
        self._stop.set()

    # ------------- helpers -------------
    def _send(self, line: str):
        # Eine komplette Zeile in den RX-Puffer legen
        with self._rx_lock:
            self._rx.extend((line + "\n").encode("utf-8"))

    def _clamp(self, v, lo, hi):
        return max(lo, min(hi, v))

    # ------------- Command Parser -------------
    def _handle_command(self, s: str):
        parts = s.split()
        if not parts:
            return
        cmd = parts[0].upper()
        arg = parts[1:]

        try:
            if cmd == 'H':
                self._send("H: Befehle -> H,P, RX d, RY d, RZ d, X v, Y v, Z v, O,OZ,O0, R,RZ0,R0, PSET, P x y z, P?, PEXY, PE, dx v, dy v, sx v, sy v, S, !, ZMAX v")
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
                    self.pick = (px,py,self._clamp(pz, self.zmin, self.zmax))
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

    # ------------- Bewegungen -------------
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
        # Serpentinen-Scan ab ORIGIN (ox,oy), Bereich sx x sy, Schritt dx/dy
        with self._motion_lock:
            self._targets.clear()
            x0, y0 = self.ox, self.oy
            x1 = self._clamp(x0 + self.sx, 0, self.work_w)
            y1 = self._clamp(y0 + self.sy, 0, self.work_h)
            # zum Startpunkt fahren
            self._targets.append((self._clamp(x0,0,self.work_w), self._clamp(y0,0,self.work_h), self.z))
            toggle = False
            y = y0
            # robust gegen falsche Parameter
            step_y = max(0.1, self.dy)
            step_x = max(0.1, self.dx)
            while y <= y1 + 1e-9:
                if not toggle:
                    # links -> rechts
                    x = x0
                    while x <= x1 + 1e-9:
                        self._targets.append((self._clamp(x,0,self.work_w), self._clamp(y,0,self.work_h), self.z))
                        x += step_x
                else:
                    # rechts -> links
                    x = x1
                    while x >= x0 - 1e-9:
                        self._targets.append((self._clamp(x,0,self.work_w), self._clamp(y,0,self.work_h), self.z))
                        x -= step_x
                toggle = not toggle
                y += step_y
            self._busy = True
        self._send(f"SCAN start dx={self.dx} dy={self.dy} sx={self.sx} sy={self.sy}")

    def _motion_loop(self):
        # 50 Hz Physik
        dt = 0.02
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
                dist = sqrt(dx*dx + dy*dy + dz*dz)
                if dist < 1e-3:
                    # Ziel erreicht
                    self.x, self.y, self.z = tx, ty, tz
                    self._targets.pop(0)
                    self._send(f"AT {self.x:.3f} {self.y:.3f} {self.z:.3f}")
                    if not self._targets:
                        self._busy = False
                        self._send("IDLE")
                    continue
                # Schrittweite bestimmen (begrenze per Achs-Speed)
                # effektive max Strecke pro dt durch XY und Z begrenzen
                max_xy = self.speed_xy * dt
                max_z  = self.speed_z * dt
                # gewünschter Schritt entlang 3D-Vektor
                step = min(max_xy, dist)
                # skaliere so, dass Z-Rate nicht überschritten wird
                if abs(dz) > 1e-9:
                    # Anteil in Z für den Schritt
                    ratio = abs(dz) / dist
                    if ratio * step / dt > self.speed_z:
                        step = self.speed_z * dt / max(ratio, 1e-9)
                ux = dx / dist
                uy = dy / dist
                uz = dz / dist
                self.x += ux * step
                self.y += uy * step
                self.z += uz * step
                # Arbeitsraum einhalten
                self.x = self._clamp(self.x, 0, self.work_w)
                self.y = self._clamp(self.y, 0, self.work_h)
                self.z = self._clamp(self.z, self.zmin, self.zmax)

    def _telemetry_loop(self):
        # 10 Hz Positionsmeldung
        dt = 0.1
        while not self._stop.is_set():
            self._send(f"POS {self.x:.3f} {self.y:.3f} {self.z:.3f}")
            time.sleep(dt)

# =============================================================================
# SerialClient: nutzt echten Port ODER den Simulator (Port-Name "SIM")
# =============================================================================
class SerialClient:
    def __init__(self):
        self.ser = None
        self.rxq = queue.Queue()
        self._stop = threading.Event()
        self.rx_thread = None

    def ports(self):
        # SIM immer anbieten
        real = []
        try:
            if serial is not None:
                real = [p.device for p in serial.tools.list_ports.comports()]
        except Exception:
            real = []
        return ["SIM"] + real

    def connect(self, port):
        self.close()
        if str(port).upper().startswith("SIM"):
            self.ser = SimSerial()
        else:
            if serial is None:
                raise RuntimeError("pyserial nicht installiert – nur SIM verfügbar")
            self.ser = serial.Serial(port, BAUD, timeout=0.05)
        self._stop.clear()
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()

    def close(self):
        self._stop.set()
        if self.rx_thread and self.rx_thread.is_alive():
            self.rx_thread.join(timeout=0.3)
        if self.ser and getattr(self.ser, 'is_open', False):
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def _rx_loop(self):
        buf = b""
        while not self._stop.is_set():
            try:
                data = self.ser.read(1024) if self.ser else b""
                if data:
                    buf += data
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        self.rxq.put(line.decode(errors="ignore").strip())
                else:
                    time.sleep(0.01)
            except Exception:
                time.sleep(0.05)

    def send_line(self, s: str):
        if self.ser and getattr(self.ser, 'is_open', False):
            if not s.endswith("\n"):
                s += "\n"
            self.ser.write(s.encode("utf-8"))

# =============================================================================
# Deine bestehende GUI (leicht erweitert: SIM-Port vorbefüllt)
# =============================================================================
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Koordinatentisch Bedienoberfläche – mit Simulator")
        self.geometry("980x620")
        self.client = SerialClient()
        self._build_ui()
        self._poll_rx()
        self.bind_all_hotkeys()

    def _build_ui(self):
        # Top: connection
        top = ttk.Frame(self); top.pack(fill="x", padx=8, pady=8)
        ttk.Label(top, text="COM-Port:").pack(side="left")
        self.port_combo = ttk.Combobox(top, width=15, values=self.client.ports())
        self.port_combo.pack(side="left", padx=4)
        if "SIM" in self.port_combo["values"]:
            self.port_combo.set("SIM")
        ttk.Button(top, text="Aktualisieren", command=self.refresh_ports).pack(side="left", padx=4)
        self.connect_btn = ttk.Button(top, text="Verbinden", command=self.toggle_connect)
        self.connect_btn.pack(side="left", padx=8)
        ttk.Button(top, text="Hilfe (H)", command=lambda: self.send("H")).pack(side="left")
        ttk.Button(top, text="Parameter (P)", command=lambda: self.send("P")).pack(side="left", padx=4)

        # Left column: jog & positions
        left = ttk.Labelframe(self, text="Jog & Positionen"); left.pack(side="left", fill="y", padx=8, pady=8)
        stepf = ttk.Frame(left); stepf.pack(pady=6)
        ttk.Label(stepf, text="Jog-Schritt (mm):").pack(side="left")
        self.jog_step = tk.DoubleVar(value=1.0)
        ttk.Entry(stepf, textvariable=self.jog_step, width=7).pack(side="left", padx=4)

        # jog grid
        grid = ttk.Frame(left); grid.pack(pady=6)
        ttk.Button(grid, text="Y +", width=8, command=lambda: self.send(f"RY {self.jog_step.get()}"))\
            .grid(row=0, column=1, pady=2)
        ttk.Button(grid, text="X -", width=8, command=lambda: self.send(f"RX {-self.jog_step.get()}"))\
            .grid(row=1, column=0, padx=2)
        ttk.Button(grid, text="X +", width=8, command=lambda: self.send(f"RX {self.jog_step.get()}"))\
            .grid(row=1, column=2, padx=2)
        ttk.Button(grid, text="Y -", width=8, command=lambda: self.send(f"RY {-self.jog_step.get()}"))\
            .grid(row=2, column=1, pady=2)

        # Z jog
        zf = ttk.Frame(left); zf.pack(pady=6)
        ttk.Label(zf, text="Z-Jog (PgUp / PgDn):").pack()
        ttk.Button(zf, text="Z +", width=8, command=lambda: self.send(f"RZ {self.jog_step.get()}"))\
            .pack(side="left", padx=4)
        ttk.Button(zf, text="Z -", width=8, command=lambda: self.send(f"RZ {-self.jog_step.get()}"))\
            .pack(side="left", padx=4)

        # absolute moves
        absf = ttk.Labelframe(left, text="Absolut anfahren (mm)")
        absf.pack(pady=8, fill="x")
        self.abs_x = tk.DoubleVar(value=0.0)
        self.abs_y = tk.DoubleVar(value=0.0)
        self.abs_z = tk.DoubleVar(value=0.0)
        row = ttk.Frame(absf); row.pack(fill="x", pady=2)
        ttk.Label(row, text="X:").pack(side="left"); ttk.Entry(row, textvariable=self.abs_x, width=8).pack(side="left", padx=4)
        ttk.Button(row, text="X fahren", command=lambda: self.send(f"X {self.abs_x.get()}"))\
            .pack(side="left", padx=4)
        row = ttk.Frame(absf); row.pack(fill="x", pady=2)
        ttk.Label(row, text="Y:").pack(side="left"); ttk.Entry(row, textvariable=self.abs_y, width=8).pack(side="left", padx=4)
        ttk.Button(row, text="Y fahren", command=lambda: self.send(f"Y {self.abs_y.get()}"))\
            .pack(side="left", padx=4)
        row = ttk.Frame(absf); row.pack(fill="x", pady=2)
        ttk.Label(row, text="Z:").pack(side="left"); ttk.Entry(row, textvariable=self.abs_z, width=8).pack(side="left", padx=4)
        ttk.Button(row, text="Z fahren", command=lambda: self.send(f"Z {self.abs_z.get()}"))\
            .pack(side="left", padx=4)

        # origin & pickup
        op = ttk.Labelframe(left, text="Nullpunkt & Entnahme")
        op.pack(fill="x", pady=8)
        ttk.Button(op, text="Origin XY setzen (O)", command=lambda: self.send("O")).pack(fill="x", pady=2)
        ttk.Button(op, text="Origin Z setzen (OZ)", command=lambda: self.send("OZ")).pack(fill="x", pady=2)
        ttk.Button(op, text="Origin XYZ setzen (O0)", command=lambda: self.send("O0")).pack(fill="x", pady=2)
        ttk.Separator(op, orient="horizontal").pack(fill="x", pady=6)
        ttk.Button(op, text="zu Origin XY (R)", command=lambda: self.send("R")).pack(fill="x", pady=2)
        ttk.Button(op, text="zu Origin Z (RZ0)", command=lambda: self.send("RZ0")).pack(fill="x", pady=2)
        ttk.Button(op, text="zu Origin XYZ (R0)", command=lambda: self.send("R0")).pack(fill="x", pady=2)
        ttk.Separator(op, orient="horizontal").pack(fill="x", pady=6)
        ttk.Button(op, text="Entnahme = aktuelle XYZ (PSET)", command=lambda: self.send("PSET")).pack(fill="x", pady=2)
        pickf = ttk.Frame(op); pickf.pack(fill="x", pady=2)
        self.pick_x = tk.DoubleVar(value=0.0)
        self.pick_y = tk.DoubleVar(value=0.0)
        self.pick_z = tk.DoubleVar(value=0.0)
        ttk.Entry(pickf, textvariable=self.pick_x, width=8).pack(side="left", padx=2)
        ttk.Entry(pickf, textvariable=self.pick_y, width=8).pack(side="left", padx=2)
        ttk.Entry(pickf, textvariable=self.pick_z, width=8).pack(side="left", padx=2)
        ttk.Button(op, text="Entnahme setzen (P x y z)", command=self.set_pick_xyz).pack(fill="x", pady=2)
        ttk.Button(op, text="Entnahme XY (PEXY)", command=lambda: self.send("PEXY")).pack(fill="x", pady=2)
        ttk.Button(op, text="Entnahme XYZ (PE)", command=lambda: self.send("PE")).pack(fill="x", pady=2)
        ttk.Button(op, text="Entnahme anzeigen (P?)", command=lambda: self.send("P?")).pack(fill="x", pady=2)

        # Right column: scan & ZMAX & console
        right = ttk.Labelframe(self, text="Scan & System"); right.pack(side="left", fill="both", expand=True, padx=8, pady=8)

        sc = ttk.Frame(right); sc.pack(fill="x", pady=4)
        self.dx = tk.DoubleVar(value=10.0)
        self.dy = tk.DoubleVar(value=10.0)
        self.spanx = tk.DoubleVar(value=50.0)
        self.spany = tk.DoubleVar(value=30.0)
        ttk.Label(sc, text="dx:").grid(row=0, column=0); ttk.Entry(sc, textvariable=self.dx, width=7).grid(row=0, column=1)
        ttk.Label(sc, text="dy:").grid(row=0, column=2); ttk.Entry(sc, textvariable=self.dy, width=7).grid(row=0, column=3)
        ttk.Label(sc, text="spanX:").grid(row=1, column=0); ttk.Entry(sc, textvariable=self.spanx, width=7).grid(row=1, column=1)
        ttk.Label(sc, text="spanY:").grid(row=1, column=2); ttk.Entry(sc, textvariable=self.spany, width=7).grid(row=1, column=3)
        ttk.Button(sc, text="Setzen", command=self.apply_scan_params).grid(row=0, column=4, padx=6)
        ttk.Button(sc, text="Scan starten (S)", command=lambda: self.send("S")).grid(row=1, column=4, padx=6)
        ttk.Button(sc, text="Abbrechen (!)", command=lambda: self.send("!")).grid(row=2, column=4, padx=6, pady=2)

        zmaxf = ttk.Frame(right); zmaxf.pack(fill="x", pady=4)
        self.zmax = tk.DoubleVar(value=60.0)
        ttk.Label(zmaxf, text="ZMAX:").pack(side="left")
        ttk.Entry(zmaxf, textvariable=self.zmax, width=7).pack(side="left", padx=4)
        ttk.Button(zmaxf, text="Setzen", command=lambda: self.send(f"ZMAX {self.zmax.get()}"))\
            .pack(side="left", padx=4)

        # Console
        cons = ttk.Frame(right); cons.pack(fill="both", expand=True, pady=4)
        self.txt = tk.Text(cons, height=18)
        self.txt.pack(side="left", fill="both", expand=True)
        sb = ttk.Scrollbar(cons, command=self.txt.yview); sb.pack(side="right", fill="y")
        self.txt.configure(yscrollcommand=sb.set)
        self.cmd_entry = ttk.Entry(right)
        self.cmd_entry.pack(fill="x")
        self.cmd_entry.bind("<Return>", lambda e: self.send(self.cmd_entry.get(), clear=True))

        # footer
        foot = ttk.Frame(self); foot.pack(fill="x", padx=8, pady=4)
        ttk.Label(foot, text="Tastatur: Pfeile=XY, Bild↑/Bild↓=Z, S=Scan, !=Abbruch").pack(side="left")

    def bind_all_hotkeys(self):
        self.bind("<Left>",  lambda e: self.send(f"RX {-self.jog_step.get()}"))
        self.bind("<Right>", lambda e: self.send(f"RX {self.jog_step.get()}"))
        self.bind("<Up>",    lambda e: self.send(f"RY {self.jog_step.get()}"))
        self.bind("<Down>",  lambda e: self.send(f"RY {-self.jog_step.get()}"))
        self.bind("<Prior>", lambda e: self.send(f"RZ {self.jog_step.get()}"))   # PageUp
        self.bind("<Next>",  lambda e: self.send(f"RZ {-self.jog_step.get()}"))  # PageDown
        self.bind("s",       lambda e: self.send("S"))
        self.bind("!",       lambda e: self.send("!"))

    def refresh_ports(self):
        self.port_combo["values"] = self.client.ports()
        if "SIM" in self.port_combo["values"]:
            self.port_combo.set("SIM")

    def toggle_connect(self):
        if self.client.ser and getattr(self.client.ser, 'is_open', False):
            self.client.close()
            self.connect_btn.config(text="Verbinden")
            self.log("Getrennt.")
        else:
            port = self.port_combo.get().strip()
            if not port:
                messagebox.showwarning("Hinweis", "Bitte COM-Port wählen.")
                return
            try:
                self.client.connect(port)
                self.connect_btn.config(text="Trennen")
                self.log(f"Verbunden mit {port}.")
                # Initial: Hilfe & Parameter abfragen
                self.send("H"); self.send("P")
            except Exception as ex:
                messagebox.showerror("Fehler", f"Konnte nicht verbinden:\n{ex}")

    def send(self, line, clear=False):
        if not line: return
        self.client.send_line(line)
        self.log(f"> {line}")
        if clear:
            self.cmd_entry.delete(0, "end")

    def set_pick_xyz(self):
        self.send(f"P {self.pick_x.get()} {self.pick_y.get()} {self.pick_z.get()}")

    def apply_scan_params(self):
        self.send(f"dx {self.dx.get()}")
        self.send(f"dy {self.dy.get()}")
        self.send(f"sx {self.spanx.get()}")
        self.send(f"sy {self.spany.get()}")

    def _poll_rx(self):
        try:
            while True:
                line = self.client.rxq.get_nowait()
                self.log(line)
        except queue.Empty:
            pass
        self.after(50, self._poll_rx)

    def log(self, msg):
        self.txt.insert("end", msg + "\n")
        self.txt.see("end")

    def on_closing(self):
        self.client.close()
        self.destroy()

if __name__ == "__main__":
    app = App()
    app.protocol("WM_DELETE_WINDOW", app.on_closing)
    app.mainloop()
