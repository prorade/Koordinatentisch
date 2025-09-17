import tkinter as tk
from tkinter import ttk, messagebox
import threading, queue, time
import serial, serial.tools.list_ports

BAUD = 115200

# ----------------------------- Simulator -----------------------------

class FakeSerial:
    """Minimaler 'serieller' Simulator mit Arduino-ähnlichen Antworten."""
    def __init__(self):
        self.is_open = True
        self._rx = queue.Queue()      # Bytes, die "gelesen" werden
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._scan_thread = None

        # --- Simulierter Maschinenzustand (mm) ---
        self.X = 0.0; self.Y = 0.0; self.Z = 0.0
        self.X_MIN, self.X_MAX = 0.0, 140.0
        self.Y_MIN, self.Y_MAX = 0.0, 70.0
        self.Z_MIN, self.Z_MAX = 0.0, 60.0

        self.X_STEPS_PER_MM = 410.3
        self.Y_STEPS_PER_MM = 390.0
        self.Z_STEPS_PER_MM = 2222.2

        self.originX = 0.0; self.originY = 0.0; self.originZ = 0.0
        self.pickX = 0.0; self.pickY = 0.0; self.pickZ = 0.0

        # Scanparameter
        self.dx = 10.0; self.dy = 10.0; self.spanX = 50.0; self.spanY = 30.0
        self.backAfterScan = True
        self.abortScan = False

        # Begrüßung wie im Sketch
        self._println("System bereit. 'H' fuer Hilfe.")
        self._print_params()

    # --- Utility ---
    def _clamp(self, v, lo, hi): return max(lo, min(hi, v))
    def _println(self, s): self._rx.put((s + "\n").encode("utf-8"))

    def close(self):
        self._stop.set()
        self.is_open = False

    def write(self, b: bytes):
        """Interpretier eingehende Kommandos synchron & stoße ggf. asynchrone Scans an."""
        try:
            line = b.decode("utf-8", errors="ignore").strip()
            if not line: return
            self._handle_cmd(line)
        except Exception as ex:
            self._println(f"[SIM] Fehler: {ex}")

    def read(self, n: int) -> bytes:
        """Nicht-blockierendes 'read': gibt bis zu n Bytes vom internen RX-Buffer zurück."""
        try:
            data = b""
            while not self._rx.empty() and len(data) < n:
                data += self._rx.get_nowait()
            return data
        except queue.Empty:
            return b""

    # --- Kommandoparser (spiegelt deinen Arduino-Sketch) ---
    def _handle_cmd(self, cmd: str):
        c = cmd.strip()
        if c.upper() == "H":
            self._print_help()
        elif c.upper() == "P":
            self._print_params()

        elif c.upper() == "O":
            self.originX, self.originY = self.X, self.Y
            self._println("Origin XY gesetzt.")
        elif c.upper() == "OZ":
            self.originZ = self.Z
            self._println("Origin Z gesetzt.")
        elif c.upper() == "O0":
            self.originX, self.originY, self.originZ = self.X, self.Y, self.Z
            self._println("Origin XYZ gesetzt.")

        elif c.upper() == "R":
            self._move_xy(self.originX, self.originY); self._println("Am Origin XY.")
        elif c.upper() == "RZ0":
            self._move_z(self.originZ); self._println("Am Origin Z.")
        elif c.upper() == "R0":
            self._move_xyz(self.originX, self.originY, self.originZ); self._println("Am Origin XYZ.")

        elif c.upper() == "S":
            self.abortScan = False
            self._start_scan()
        elif c == "!":
            self.abortScan = True
            self._println("Scan abgebrochen (Befehl '!').")

        elif c.upper().startswith("BA "):
            try:
                v = int(c.split()[1]); self.backAfterScan = (v != 0)
                self._println(f"backAfterScan = {'AN' if self.backAfterScan else 'AUS'}")
            except:
                self._println("Format: BA 0/1")

        elif c.upper().startswith("DX "):
            try: self.dx = float(c.split()[1]); self._println(f"dx={self.dx:.3f}")
            except: self._println("Format: dx <mm>")
        elif c.upper().startswith("DY "):
            try: self.dy = float(c.split()[1]); self._println(f"dy={self.dy:.3f}")
            except: self._println("Format: dy <mm>")
        elif c.upper().startswith("SX "):
            try: self.spanX = float(c.split()[1]); self._println(f"spanX={self.spanX:.3f}")
            except: self._println("Format: sx <mm>")
        elif c.upper().startswith("SY "):
            try: self.spanY = float(c.split()[1]); self._println(f"spanY={self.spanY:.3f}")
            except: self._println("Format: sy <mm>")

        elif c.upper().startswith("X "):
            try: self._move_xy(self._clamp(float(c.split()[1]), self.X_MIN, self.X_MAX), self.Y)
            except: self._println("Format: X <mm>")
        elif c.upper().startswith("Y "):
            try: self._move_xy(self.X, self._clamp(float(c.split()[1]), self.Y_MIN, self.Y_MAX))
            except: self._println("Format: Y <mm>")
        elif c.upper().startswith("Z "):
            try: self._move_z(self._clamp(float(c.split()[1]), self.Z_MIN, self.Z_MAX))
            except: self._println("Format: Z <mm>")

        elif c.upper().startswith("RX "):
            try: self._move_xy(self._clamp(self.X + float(c.split()[1]), self.X_MIN, self.X_MAX), self.Y)
            except: self._println("Format: RX <mm>")
        elif c.upper().startswith("RY "):
            try: self._move_xy(self.X, self._clamp(self.Y + float(c.split()[1]), self.Y_MIN, self.Y_MAX))
            except: self._println("Format: RY <mm>")
        elif c.upper().startswith("RZ "):
            try: self._move_z(self._clamp(self.Z + float(c.split()[1]), self.Z_MIN, self.Z_MAX))
            except: self._println("Format: RZ <mm>")

        elif c.upper().startswith("G "):
            parts = c.split()
            try:
                if len(parts)==3:
                    tx = float(parts[1]); ty = float(parts[2])
                    self._move_xy(self._clamp(tx, self.X_MIN, self.X_MAX), self._clamp(ty, self.Y_MIN, self.Y_MAX))
                elif len(parts)==4:
                    tx = float(parts[1]); ty = float(parts[2]); tz=float(parts[3])
                    self._move_xyz(self._clamp(tx, self.X_MIN, self.X_MAX),
                                   self._clamp(ty, self.Y_MIN, self.Y_MAX),
                                   self._clamp(tz, self.Z_MIN, self.Z_MAX))
                else:
                    self._println("Format: g <x> <y>  oder  g <x> <y> <z>")
            except:
                self._println("Format: g <x> <y> [z]")

        elif c.upper().startswith("ZMAX "):
            try: self.Z_MAX = float(c.split()[1]); self._println(f"Z_MAX_MM = {self.Z_MAX:.3f}")
            except: self._println("Format: ZMAX <mm>")

        elif c.upper() == "PSET":
            self.pickX, self.pickY, self.pickZ = self.X, self.Y, self.Z
            self._println("Entnahme XYZ gelernt.")
        elif c.upper().startswith("P?"):
            self._println(f"Entnahme XYZ: ({self.pickX:.3f}, {self.pickY:.3f}, {self.pickZ:.3f}) mm")
        elif c.upper().startswith("P "):
            try:
                _, sx, sy, sz = c.split()
                self.pickX = self._clamp(float(sx), self.X_MIN, self.X_MAX)
                self.pickY = self._clamp(float(sy), self.Y_MIN, self.Y_MAX)
                self.pickZ = self._clamp(float(sz), self.Z_MIN, self.Z_MAX)
                self._println("Entnahmeposition gesetzt.")
            except:
                self._println("Format: P <x> <y> <z>")
        elif c.upper() == "PE":
            self._move_xyz(self.pickX, self.pickY, self.pickZ); self._println("Entnahme XYZ angefahren.")
        elif c.upper() == "PEXY":
            self._move_xy(self.pickX, self.pickY); self._println("Entnahme XY angefahren.")

        else:
            self._println("Unbekannter Befehl. 'H' fuer Hilfe.")

    # --- "Bewegungen": wir simulieren Zeit & Statusausgaben ---
    def _move_xy(self, x, y):
        dx = abs(x - self.X) + abs(y - self.Y)
        self._println(f"Fahre XY -> ({x:.3f}, {y:.3f})")
        time.sleep(min(0.8, 0.01 * dx))  # kleine Verzögerung
        self.X, self.Y = x, y
        self._println(f"XY erreicht: ({self.X:.3f}, {self.Y:.3f})")

    def _move_z(self, z):
        dz = abs(z - self.Z)
        self._println(f"Fahre Z -> {z:.3f}")
        time.sleep(min(0.6, 0.015 * dz))
        self.Z = z
        self._println(f"Z erreicht: {self.Z:.3f}")

    def _move_xyz(self, x, y, z):
        self._println(f"Fahre XYZ -> ({x:.3f}, {y:.3f}, {z:.3f})")
        # simuliert paralleles Fahren
        t = min(1.0, 0.008*(abs(x-self.X)+abs(y-self.Y)) + 0.012*abs(z-self.Z))
        time.sleep(t)
        self.X, self.Y, self.Z = x, y, z
        self._println(f"XYZ erreicht: ({self.X:.3f}, {self.Y:.3f}, {self.Z:.3f})")

    def _start_scan(self):
        if self._scan_thread and self._scan_thread.is_alive():
            self._println("Scan läuft bereits.")
            return
        self._scan_thread = threading.Thread(target=self._run_scan, daemon=True)
        self._scan_thread.start()

    def _run_scan(self):
        x0, y0 = self.originX, self.originY
        self._println("Starte Scan...")
        self._move_xy(x0, y0)
        nx = int(self.spanX/self.dx)+1 if self.dx>0 else 1
        ny = int(self.spanY/self.dy)+1 if self.dy>0 else 1
        nx = max(1, nx); ny = max(1, ny)
        self._println(f"Rasterpunkte: nx={nx} ny={ny}")
        for j in range(ny):
            if self.abortScan: break
            y = self._clamp(y0 + j*self.dy, self.Y_MIN, self.Y_MAX)
            rng = range(nx) if (j%2==0) else range(nx-1, -1, -1)
            for i in rng:
                if self.abortScan: break
                x = self._clamp(x0 + i*self.dx, self.X_MIN, self.X_MAX)
                self._move_xy(x, y)
                time.sleep(0.02)  # Verweilzeit je Rasterpunkt (kurz)
            if j < ny-1 and not self.abortScan:
                y_next = self._clamp(y0 + (j+1)*self.dy, self.Y_MIN, self.Y_MAX)
                self._move_xy(self.X, y_next)
        if self.abortScan:
            self._println("Scan abgebrochen.")
        else:
            self._println("Scan fertig.")
            if self.backAfterScan:
                self._println("Zurueck zu Origin XY...")
                self._move_xy(self.originX, self.originY)
                self._println("Am Origin XY.")

    def _print_help(self):
        self._println("Befehle (115200 Baud): H,P,O,OZ,O0,S,!,R,RZ0,R0, BA 0/1,")
        self._println("X/Y/Z <mm>, RX/RY/RZ <dmm>, g x y [z], ZMAX <mm>,")
        self._println("PSET, P x y z, P?, PEXY, PE")

    def _print_params(self):
        self._println("=== Parameter ===")
        self._println(f"Steps/mm: X={self.X_STEPS_PER_MM:.3f}  Y={self.Y_STEPS_PER_MM:.3f}  Z={self.Z_STEPS_PER_MM:.3f}")
        self._println(f"Soft-Limits: X[0..{self.X_MAX:.1f}]  Y[0..{self.Y_MAX:.1f}]  Z[{self.Z_MIN:.1f}..{self.Z_MAX:.1f}] mm")
        self._println(f"Origin XYZ: ({self.originX:.3f}, {self.originY:.3f}, {self.originZ:.3f}) mm")
        self._println(f"Raster: dx={self.dx:.3f}  dy={self.dy:.3f}  spanX={self.spanX:.3f}  spanY={self.spanY:.3f}")
        self._println(f"Auto-Rueckfahrt nach Scan: {'AN' if self.backAfterScan else 'AUS'}")
        self._println(f"Entnahme XYZ: ({self.pickX:.3f}, {self.pickY:.3f}, {self.pickZ:.3f}) mm")
        self._println("=================")

# ----------------------------- Original GUI (leicht erweitert) -----------------------------

class SerialClient:
    def __init__(self):
        self.ser = None
        self.rxq = queue.Queue()
        self._stop = threading.Event()
        self.rx_thread = None
        self.use_sim = False

    def ports(self):
        return [p.device for p in serial.tools.list_ports.comports()]

    def connect(self, port, use_sim=False):
        self.close()
        self.use_sim = use_sim
        if use_sim:
            self.ser = FakeSerial()
        else:
            self.ser = serial.Serial(port, BAUD, timeout=0.05)
        self._stop.clear()
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()

    def close(self):
        self._stop.set()
        if self.rx_thread and self.rx_thread.is_alive():
            self.rx_thread.join(timeout=0.3)
        if self.ser:
            try:
                self.ser.close()
            except: pass
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
                        try:
                            self.rxq.put(line.decode(errors="ignore").strip())
                        except: pass
                else:
                    time.sleep(0.01)
            except Exception:
                time.sleep(0.05)

    def send_line(self, s: str):
        if self.ser:
            if not s.endswith("\n"): s += "\n"
            self.ser.write(s.encode("utf-8"))

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Koordinatentisch Bedienoberfläche (mit Simulator)")
        self.geometry("1000x660")
        self.client = SerialClient()
        self._build_ui()
        self._poll_rx()
        self.bind_all_hotkeys()

    def _build_ui(self):
        # Top: connection + simulator
        top = ttk.Frame(self); top.pack(fill="x", padx=8, pady=8)
        ttk.Label(top, text="COM-Port:").pack(side="left")
        self.port_combo = ttk.Combobox(top, width=15, values=self.client.ports())
        self.port_combo.pack(side="left", padx=4)
        ttk.Button(top, text="Aktualisieren", command=self.refresh_ports).pack(side="left", padx=4)
        self.sim_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(top, text="Simulator", variable=self.sim_var).pack(side="left", padx=8)
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

        grid = ttk.Frame(left); grid.pack(pady=6)
        ttk.Button(grid, text="Y +", width=8, command=lambda: self.send(f"RY {self.jog_step.get()}")).grid(row=0, column=1, pady=2)
        ttk.Button(grid, text="X -", width=8, command=lambda: self.send(f"RX {-self.jog_step.get()}")).grid(row=1, column=0, padx=2)
        ttk.Button(grid, text="X +", width=8, command=lambda: self.send(f"RX {self.jog_step.get()}")).grid(row=1, column=2, padx=2)
        ttk.Button(grid, text="Y -", width=8, command=lambda: self.send(f"RY {-self.jog_step.get()}")).grid(row=2, column=1, pady=2)

        zf = ttk.Frame(left); zf.pack(pady=6)
        ttk.Label(zf, text="Z-Jog (PgUp / PgDn):").pack()
        ttk.Button(zf, text="Z +", width=8, command=lambda: self.send(f"RZ {self.jog_step.get()}")).pack(side="left", padx=4)
        ttk.Button(zf, text="Z -", width=8, command=lambda: self.send(f"RZ {-self.jog_step.get()}")).pack(side="left", padx=4)

        absf = ttk.Labelframe(left, text="Absolut anfahren (mm)")
        absf.pack(pady=8, fill="x")
        self.abs_x = tk.DoubleVar(value=0.0)
        self.abs_y = tk.DoubleVar(value=0.0)
        self.abs_z = tk.DoubleVar(value=0.0)
        row = ttk.Frame(absf); row.pack(fill="x", pady=2)
        ttk.Label(row, text="X:").pack(side="left"); ttk.Entry(row, textvariable=self.abs_x, width=8).pack(side="left", padx=4)
        ttk.Button(row, text="X fahren", command=lambda: self.send(f"X {self.abs_x.get()}")).pack(side="left", padx=4)
        row = ttk.Frame(absf); row.pack(fill="x", pady=2)
        ttk.Label(row, text="Y:").pack(side="left"); ttk.Entry(row, textvariable=self.abs_y, width=8).pack(side="left", padx=4)
        ttk.Button(row, text="Y fahren", command=lambda: self.send(f"Y {self.abs_y.get()}")).pack(side="left", padx=4)
        row = ttk.Frame(absf); row.pack(fill="x", pady=2)
        ttk.Label(row, text="Z:").pack(side="left"); ttk.Entry(row, textvariable=self.abs_z, width=8).pack(side="left", padx=4)
        ttk.Button(row, text="Z fahren", command=lambda: self.send(f"Z {self.abs_z.get()}")).pack(side="left", padx=4)

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
        ttk.Button(zmaxf, text="Setzen", command=lambda: self.send(f"ZMAX {self.zmax.get()}")).pack(side="left", padx=4)

        cons = ttk.Frame(right); cons.pack(fill="both", expand=True, pady=4)
        self.txt = tk.Text(cons, height=18)
        self.txt.pack(side="left", fill="both", expand=True)
        sb = ttk.Scrollbar(cons, command=self.txt.yview); sb.pack(side="right", fill="y")
        self.txt.configure(yscrollcommand=sb.set)
        self.cmd_entry = ttk.Entry(right); self.cmd_entry.pack(fill="x")
        self.cmd_entry.bind("<Return>", lambda e: self.send(self.cmd_entry.get(), clear=True))

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

    def toggle_connect(self):
        if self.client.ser and (getattr(self.client.ser, "is_open", False) or True):
            self.client.close()
            self.connect_btn.config(text="Verbinden")
            self.log("Getrennt.")
        else:
            use_sim = self.sim_var.get()
            if not use_sim:
                port = self.port_combo.get().strip()
                if not port:
                    messagebox.showwarning("Hinweis", "Bitte COM-Port wählen oder Simulator aktivieren.")
                    return
            try:
                self.client.connect(self.port_combo.get().strip(), use_sim=use_sim)
                self.connect_btn.config(text="Trennen")
                self.log("Simulator verbunden." if use_sim else f"Verbunden mit {self.port_combo.get().strip()}.")
                self.send("H"); self.send("P")
            except Exception as ex:
                messagebox.showerror("Fehler", f"Konnte nicht verbinden:\n{ex}")

    def send(self, line, clear=False):
        if not line: return
        self.client.send_line(line)
        self.log(f"> {line}")
        if clear: self.cmd_entry.delete(0, "end")

    def set_pick_xyz(self):
        self.send(f"P {self.pick_x.get()} {self.pick_y.get()} {self.pick_z.get()}")

    def apply_scan_params(self):
        self.send(f"dx {self.dx.get()}"); self.send(f"dy {self.dy.get()}")
        self.send(f"sx {self.spanx.get()}"); self.send(f"sy {self.spany.get()}")

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
