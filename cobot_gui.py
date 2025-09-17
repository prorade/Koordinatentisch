import tkinter as tk
from tkinter import ttk, messagebox
import threading, queue, time
import serial, serial.tools.list_ports

BAUD = 115200

class SerialClient:
    def __init__(self):
        self.ser = None
        self.rxq = queue.Queue()
        self._stop = threading.Event()
        self.rx_thread = None

    def ports(self):
        return [p.device for p in serial.tools.list_ports.comports()]

    def connect(self, port):
        self.close()
        self.ser = serial.Serial(port, BAUD, timeout=0.05)
        self._stop.clear()
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()

    def close(self):
        self._stop.set()
        if self.rx_thread and self.rx_thread.is_alive():
            self.rx_thread.join(timeout=0.3)
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.ser = None

    def _rx_loop(self):
        buf = b""
        while not self._stop.is_set():
            try:
                data = self.ser.read(1024)
                if data:
                    buf += data
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        self.rxq.put(line.decode(errors="ignore").strip())
                else:
                    time.sleep(0.01)
            except Exception:
                time.sleep(0.1)

    def send_line(self, s: str):
        if self.ser and self.ser.is_open:
            if not s.endswith("\n"):
                s += "\n"
            self.ser.write(s.encode("utf-8"))

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Koordinatentisch Bedienoberfläche")
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
        ttk.Button(grid, text="Y +", width=8, command=lambda: self.send(f"RY {self.jog_step.get()}")).grid(row=0, column=1, pady=2)
        ttk.Button(grid, text="X -", width=8, command=lambda: self.send(f"RX {-self.jog_step.get()}")).grid(row=1, column=0, padx=2)
        ttk.Button(grid, text="X +", width=8, command=lambda: self.send(f"RX {self.jog_step.get()}")).grid(row=1, column=2, padx=2)
        ttk.Button(grid, text="Y -", width=8, command=lambda: self.send(f"RY {-self.jog_step.get()}")).grid(row=2, column=1, pady=2)

        # Z jog
        zf = ttk.Frame(left); zf.pack(pady=6)
        ttk.Label(zf, text="Z-Jog (PgUp / PgDn):").pack()
        ttk.Button(zf, text="Z +", width=8, command=lambda: self.send(f"RZ {self.jog_step.get()}")).pack(side="left", padx=4)
        ttk.Button(zf, text="Z -", width=8, command=lambda: self.send(f"RZ {-self.jog_step.get()}")).pack(side="left", padx=4)

        # absolute moves
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
        ttk.Button(zmaxf, text="Setzen", command=lambda: self.send(f"ZMAX {self.zmax.get()}")).pack(side="left", padx=4)

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

    def toggle_connect(self):
        if self.client.ser and self.client.ser.is_open:
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
