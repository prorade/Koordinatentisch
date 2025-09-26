# baumer_live_focus_full.py
# Baumer VCX Live-Viewer (GenTL/Harvester) mit:
# - Stabiler Anzeige (eigene Frame-Kopie → kein Tearing)
# - Robuster Pixel-Decode (Mono8 / Mono12Packed / Mono12 / Mono16 / Mono10)
# - Mess-ROI im Bild + Schärfemetriken (Laplacian / Tenengrad / Brenner)
# - Live-Overlay (Werte, FPS, Bildgröße, PF, Binning/Decimation)
# - Kamera-AOI (1/2/3) mit sauberem Neustart
# - Hardware-Binning/Decimation (gleiches FOV, weniger Pixel) mit Neustart
# - Autofokus-Stub (Hill-Climb) mit Stepper-Controller (Platzhalter)
#
# Tasten:
#   ← ↑ → ↓     Mess-ROI bewegen
#   [ / ]       Mess-ROI kleiner / größer
#   R           Mess-ROI zentrieren
#   M           Schärfemetrik wechseln (Laplacian / Tenengrad / Brenner)
#   H           Display-Stretch an/aus (nur Anzeige)
#   B / N       Binning-Faktor +1 / −1
#   X / Z       Decimation-Faktor +1 / −1
#   0           Binning & Decimation auf 1× zurücksetzen
#   1 / 2 / 3   Kamera-AOI Full / Half / Quarter (mit Neustart)
#   + / -       Exposure ×1.5 / ÷1.5
#   G / g       Gain +1 / −1 dB
#   Leertaste / P   Snapshot
#   F           Autofokus starten (Stub, siehe StepperController)
#   ESC         Ende

import os, sys, time, cv2
import numpy as np
from harvesters.core import Harvester

# ==== Pfade/Kamera ===========================================================
BIN_DIR  = r"H:\Baumer_neoAPI_1.5.0_win_x86_64_cpp\bin"   # <— ANPASSEN
CTI_PATH = rf"{BIN_DIR}\bgapi2_gige.cti"                  # <— ANPASSEN
TARGET_IP = "192.168.178.2"                               # optional: Kamera-IP für Auswahl

# ==== Start-Defaults =========================================================
START_EXPOSURE_US = 5000.0
START_GAIN_DB     = 6.0
START_FPS         = 60.0
WANT_PACKET       = 9000       # Jumbo; fällt zurück falls nicht möglich
WANT_THROUGHPUT   = 0          # 0=aus; sonst Bit/s (z. B. 800_000_000)
DISPLAY_STRETCH   = False      # stabiler Default (mit 'H' toggeln)

# ==== Utils & GenICam Helper ================================================

def ensure_paths():
    if hasattr(os, "add_dll_directory"):
        os.add_dll_directory(BIN_DIR)
    os.environ["PATH"] = BIN_DIR + os.pathsep + os.environ.get("PATH","")
    if not os.path.exists(CTI_PATH):
        print("CTI nicht gefunden:", CTI_PATH); sys.exit(1)

def set_node(nm, name, value):
    if not name: return False
    try: nm.get_node(name).value = value; return True
    except Exception: return False

def get_val(nm, name):
    try: return nm.get_node(name).value
    except Exception: return None

def get_float(nm, name):
    try: return float(nm.get_node(name).value)
    except Exception: return None

def get_int(nm, name):
    try: return int(nm.get_node(name).value)
    except Exception: return None

def get_limits(nm, name):
    try:
        n = nm.get_node(name)
        return float(getattr(n, "min")), float(getattr(n, "max"))
    except Exception:
        return None, None

def get_inc(nm, name, fallback=1):
    try: return int(getattr(nm.get_node(name), "inc"))
    except Exception: return fallback

def enum_symbolics(nm, enum_name):
    try: return list(nm.get_node(enum_name).symbolics)
    except Exception: return []

def clamp(v, mn, mx):
    if mn is not None: v = max(v, mn)
    if mx is not None: v = min(v, mx)
    return v

def supports(nm, node_name):
    try: nm.get_node(node_name); return True
    except Exception: return False

# ==== Stabile Pixel-Decode (immer kopiert) ==================================

def decode_mono12packed(u8: np.ndarray, w: int, h: int) -> np.ndarray:
    expected = (w*h*3)//2
    triples = u8[:expected].reshape(h, w//2, 3)
    b0 = triples[:,:,0].astype(np.uint16)
    b1 = triples[:,:,1].astype(np.uint16)
    b2 = triples[:,:,2].astype(np.uint16)
    p0 = (b0 | ((b1 & 0x0F)<<8)) & 0x0FFF
    p1 = ((b1>>4) | (b2<<4)) & 0x0FFF
    out = np.empty((h, w), dtype=np.uint16)
    out[:,0::2] = p0; out[:,1::2] = p1
    return out

def to_display_u8_copy(comp, pf_str: str):
    """
    Gibt immer eine EIGENE Kopie (np.uint8, C-contiguous) zurück → kein Tearing.
    """
    h, w = comp.height, comp.width
    u8 = comp.data
    total = u8.size
    if w==0 or h==0 or total==0:
        return np.zeros((10,10), np.uint8), "Empty"
    pf = (pf_str or "").lower()

    if "mono8" in pf:
        return u8.reshape(h, w).copy(), "Mono8 (8b)"

    if "mono12packed" in pf:
        img12 = decode_mono12packed(u8, w, h)
        return (img12>>4).astype(np.uint8, copy=True), "Mono12Packed (>>4)"

    if "mono12" in pf and "packed" not in pf:
        img16 = np.frombuffer(u8.tobytes(), dtype=np.uint16, count=w*h).reshape(h, w)
        return (img16>>4).astype(np.uint8, copy=True), "Mono12 (>>4)"

    if "mono16" in pf:
        img16 = np.frombuffer(u8.tobytes(), dtype=np.uint16, count=w*h).reshape(h, w)
        return (img16>>8).astype(np.uint8, copy=True), "Mono16 (>>8)"

    if "mono10" in pf and "packed" not in pf:
        img16 = np.frombuffer(u8.tobytes(), dtype=np.uint16, count=w*h).reshape(h, w)
        return (img16>>2).astype(np.uint8, copy=True), "Mono10 (>>2)"

    # Fallback anhand Bytes/Pixel
    bpp = total / (w*h)
    if abs(bpp-1.0)<1e-3:
        return u8.reshape(h, w).copy(), "8b(?)"
    if abs(bpp-2.0)<1e-3:
        img16 = np.frombuffer(u8.tobytes(), dtype=np.uint16, count=w*h).reshape(h, w)
        return (img16>>8).astype(np.uint8, copy=True), "16b(>>8?)"
    if abs(bpp-1.5)<1e-2 and (w%2==0):
        img12 = decode_mono12packed(u8, w, h)
        return (img12>>4).astype(np.uint8, copy=True), "12p(>>4?)"

    return u8.reshape(h, -1)[:, :w].copy(), f"Unknown bpp={bpp:.2f}"

# ==== Kamera-Basisconfig =====================================================

def configure_camera_basics(nm):
    for k, v in [("TriggerMode","Off"), ("ExposureMode","Timed"),
                 ("AcquisitionMode","Continuous")]:
        set_node(nm, k, v)

    # PixelFormat bevorzugt Mono8 (einfach/schnell)
    pf_list = enum_symbolics(nm, "PixelFormat")
    if "Mono8" in pf_list:
        set_node(nm, "PixelFormat", "Mono8")

    # Auto/Look-Up-Tables/Gamma aus
    set_node(nm, "ExposureAuto", "Off")
    set_node(nm, "GainAuto", "Off")
    set_node(nm, "GammaEnable", False)
    set_node(nm, "LUTEnable", False)

    # BlackLevel auf 0 (falls vorhanden)
    bl_min, bl_max = get_limits(nm, "BlackLevel")
    if bl_min is not None and bl_max is not None:
        set_node(nm, "BlackLevel", 0.0)

    # Exposure (µs)
    exp_name = "ExposureTime" if get_limits(nm, "ExposureTime") != (None, None) else "ExposureTimeAbs"
    emin, emax = get_limits(nm, exp_name)
    set_node(nm, exp_name, float(clamp(START_EXPOSURE_US, emin, emax)))

    # Gain (dB oder Raw)
    gain_name = "Gain" if get_limits(nm, "Gain") != (None, None) else "GainRaw"
    gmin, gmax = get_limits(nm, gain_name)
    set_node(nm, gain_name, float(clamp(START_GAIN_DB, gmin, gmax)))

    # Framerate
    set_node(nm, "AcquisitionFrameRateEnable", True)
    fmin, fmax = get_limits(nm, "AcquisitionFrameRate")
    if fmin is not None or fmax is not None:
        set_node(nm, "AcquisitionFrameRate", float(clamp(START_FPS, fmin, fmax)))

    # Throughput/Packets
    set_node(nm, "DeviceLinkThroughputLimitMode", "Off")
    if WANT_THROUGHPUT and WANT_THROUGHPUT > 0:
        set_node(nm, "DeviceLinkThroughputLimitMode", "On")
        set_node(nm, "DeviceLinkThroughputLimit", int(WANT_THROUGHPUT))

    for p in [WANT_PACKET, 9000, 8192, 3000, 1500]:
        if p and set_node(nm, "GevSCPSPacketSize", int(p)):
            break

    return exp_name, gain_name

# ==== AOI/ROI (Kamera) ======================================================

def set_roi_full(nm):
    try:
        Wmax = int(get_val(nm, "WidthMax")); Hmax = int(get_val(nm, "HeightMax"))
    except Exception: return
    w_inc = get_inc(nm, "Width", 2); h_inc = get_inc(nm, "Height", 2)
    set_node(nm, "OffsetX", 0); set_node(nm, "OffsetY", 0)
    set_node(nm, "Width",  (Wmax // w_inc) * w_inc)
    set_node(nm, "Height", (Hmax // h_inc) * h_inc)

# ==== Binning / Decimation ===================================================

def apply_binning(nm, factor, mode="Average"):
    # Decimation zurücksetzen
    for n in ("DecimationHorizontal", "DecimationVertical"):
        if supports(nm, n): set_node(nm, n, 1)
    ok = False
    if supports(nm,"BinningHorizontal") and supports(nm,"BinningVertical"):
        if supports(nm, "BinningMode"):
            syms = enum_symbolics(nm, "BinningMode")
            if mode in syms: set_node(nm, "BinningMode", mode)
            elif "Average" in syms: set_node(nm, "BinningMode", "Average")
            elif "Sum" in syms: set_node(nm, "BinningMode", "Sum")
        set_node(nm, "BinningHorizontal", int(max(1, factor)))
        set_node(nm, "BinningVertical",   int(max(1, factor)))
        ok = True
    return ok

def apply_decimation(nm, factor):
    # Binning zurücksetzen
    for n in ("BinningHorizontal","BinningVertical"):
        if supports(nm, n): set_node(nm, n, 1)
    ok = False
    if supports(nm,"DecimationHorizontal") and supports(nm,"DecimationVertical"):
        set_node(nm, "DecimationHorizontal", int(max(1, factor)))
        set_node(nm, "DecimationVertical",   int(max(1, factor)))
        ok = True
    return ok

def get_scaling_state(nm):
    b_h = get_int(nm, "BinningHorizontal") if supports(nm,"BinningHorizontal") else 1
    b_v = get_int(nm, "BinningVertical")   if supports(nm,"BinningVertical")   else 1
    d_h = get_int(nm, "DecimationHorizontal") if supports(nm,"DecimationHorizontal") else 1
    d_v = get_int(nm, "DecimationVertical")   if supports(nm,"DecimationVertical")   else 1
    return (b_h or 1), (b_v or 1), (d_h or 1), (d_v or 1)

# ==== Harvester Lifecycle ====================================================

def create_ia(h, device_index):
    ia = h.create(device_index)
    try: ia.num_buffers = 4     # geringe Latenz
    except Exception: pass
    return ia

def stop_destroy(ia):
    try: ia.stop()
    except Exception: pass
    try: ia.destroy()
    except Exception: pass

def restart_stream(h, device_index, pf_hint=None):
    ia = create_ia(h, device_index)
    nm = ia.remote_device.node_map
    exp_name, gain_name = configure_camera_basics(nm)
    pf_str = str(get_val(nm, "PixelFormat")) or (pf_hint or "Mono8")
    ia.start()
    return ia, nm, exp_name, gain_name, pf_str

def restart_with_roi(h, device_index, pf_hint, scale):
    # zentrierte ROI via temporärem IA setzen, dann neu starten
    ia_tmp = create_ia(h, device_index)
    nm = ia_tmp.remote_device.node_map
    set_node(nm, "TLParamsLocked", 0)
    try: nm.get_node("AcquisitionStop").execute()
    except Exception: pass

    # zentrierte ROI
    try:
        Wmax = int(get_val(nm, "WidthMax")); Hmax = int(get_val(nm, "HeightMax"))
        w_inc = get_inc(nm, "Width", 2); h_inc = get_inc(nm, "Height", 2)
        ox_inc = get_inc(nm, "OffsetX", 1); oy_inc = get_inc(nm, "OffsetY", 1)
        w = (max(16, int(Wmax*scale)) // w_inc) * w_inc
        h = (max(16, int(Hmax*scale)) // h_inc) * h_inc
        ox = ((Wmax - w)//2 // ox_inc) * ox_inc
        oy = ((Hmax - h)//2 // oy_inc) * oy_inc
        set_node(nm, "OffsetX", int(ox)); set_node(nm, "OffsetY", int(oy))
        set_node(nm, "Width",  int(w));   set_node(nm, "Height", int(h))
    except Exception:
        pass

    set_node(nm, "TLParamsLocked", 1)
    stop_destroy(ia_tmp)
    return restart_stream(h, device_index, pf_hint)

def restart_with_scaling(h, device_index, pf_hint, use_binning, factor):
    ia_tmp = create_ia(h, device_index)
    nm = ia_tmp.remote_device.node_map
    set_node(nm, "TLParamsLocked", 0)
    try: nm.get_node("AcquisitionStop").execute()
    except Exception: pass

    # gleiches FOV → volle ROI
    set_roi_full(nm)

    ok = False
    if use_binning:
        ok = apply_binning(nm, factor, mode="Average")
        if not ok:
            ok = apply_decimation(nm, factor)
    else:
        ok = apply_decimation(nm, factor)
        if not ok:
            ok = apply_binning(nm, factor, mode="Average")

    set_node(nm, "TLParamsLocked", 1)
    stop_destroy(ia_tmp)
    return restart_stream(h, device_index, pf_hint)

# ==== Schärfe-Metriken ======================================================

def focus_metric_laplacian(u8_roi: np.ndarray) -> float:
    lap = cv2.Laplacian(u8_roi, cv2.CV_64F, ksize=3)
    return float(lap.var())

def focus_metric_tenengrad(u8_roi: np.ndarray) -> float:
    gx = cv2.Sobel(u8_roi, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(u8_roi, cv2.CV_32F, 0, 1, ksize=3)
    mag2 = gx*gx + gy*gy
    return float(mag2.mean())

def focus_metric_brenner(u8_roi: np.ndarray) -> float:
    diff = u8_roi[:, 2:].astype(np.float32) - u8_roi[:, :-2].astype(np.float32)
    return float((diff*diff).mean())

FOCUS_METHODS = [("Laplacian", focus_metric_laplacian),
                 ("Tenengrad", focus_metric_tenengrad),
                 ("Brenner",   focus_metric_brenner)]

# ==== Autofokus (Stub) ======================================================

class StepperController:
    """Platzhalter für Z-Antrieb. Ersetze move_relative()/wait_settle() durch echte Ansteuerung."""
    def move_relative(self, steps: int):
        # TODO: Implementieren (z. B. serielle Kommandos, GPIO, etc.)
        print(f"[Stepper] move_relative({steps}) (Stub)")

    def wait_settle(self, seconds: float = 0.12):
        time.sleep(seconds)

def autofocus_hill_climb(get_focus_value, stepper: StepperController,
                         start_step=200, min_step=5, settle=0.12):
    """
    Einfache coarse→fine Hill-Climb-Suche auf Maximum der Schärfe.
    get_focus_value(): callable → float (neuen RAW-Wert messen)
    """
    cur = get_focus_value()
    step = start_step
    direction = +1
    improved = True

    while step >= min_step:
        stepper.move_relative(direction*step)
        stepper.wait_settle(settle)
        val = get_focus_value()

        if val > cur:
            cur = val
            improved = True
        else:
            # zurück, Richtung wechseln, Schritt halbieren
            stepper.move_relative(-direction*step)
            stepper.wait_settle(settle)
            direction *= -1
            step = max(min_step, step // 2)
            improved = False

        if not improved and step > min_step:
            step = max(min_step, step // 2)

    return cur

# ==== Main ==================================================================

def main():
    ensure_paths()
    h = Harvester(); h.add_file(CTI_PATH); h.update()
    if not h.device_info_list:
        print("Keine GenTL-Geräte gefunden."); h.reset(); sys.exit(1)

    # Zielgerät auswählen (per IP, sonst erstes)
    device_index = next((i for i,d in enumerate(h.device_info_list) if TARGET_IP in str(d)), 0)

    ia = create_ia(h, device_index)
    nm = ia.remote_device.node_map
    exp_name, gain_name = configure_camera_basics(nm)
    pf_str = str(get_val(nm, "PixelFormat")) or "Mono8"
    model  = get_val(nm, "DeviceModelName")
    serial = get_val(nm, "DeviceSerialNumber")

    # Start: Full-FOV
    set_roi_full(nm)
    ia.start()

    cv2.namedWindow("Live", cv2.WINDOW_AUTOSIZE)
    print("Tasten: ←↑→↓ ROI  [ ] Größe  R Center  M Metrik  H Stretch  B/N Bin ±  X/Z Dec ±  0 Reset  1/2/3 AOI  +/- Exp  G/g Gain  SPACE/P Snapshot  F AF  ESC")

    # Mess-ROI
    auto_stretch = DISPLAY_STRETCH
    method_idx = 0
    ema = None
    ema_alpha = 0.2
    roi_w, roi_h = 256, 256
    roi_step = 16
    roi_x, roi_y = 100, 100

    stepper = StepperController()

    last = time.time(); frames=0; disp_fps=0.0

    try:
        while True:
            # Frame holen (mit Kopie)
            with ia.fetch(timeout=1500) as buf:
                comp = buf.payload.components[0]
                disp_u8, decode_label = to_display_u8_copy(comp, pf_str)

            hgt, wid = disp_u8.shape[:2]

            # ROI clampen
            roi_w = max(32, min(roi_w, wid))
            roi_h = max(32, min(roi_h, hgt))
            roi_x = min(max(0, roi_x), wid - roi_w)
            roi_y = min(max(0, roi_y), hgt - roi_h)

            # Schärfe (RAW + EMA)
            name, func = FOCUS_METHODS[method_idx]
            roi = disp_u8[roi_y:roi_y+roi_h, roi_x:roi_x+roi_w]
            raw = func(roi)
            if ema is None: ema = raw
            else: ema = ema_alpha*raw + (1.0-ema_alpha)*ema

            # FPS
            frames += 1
            now = time.time()
            if now - last >= 1.0:
                disp_fps = frames/(now-last); frames=0; last=now

            # Anzeige
            show = disp_u8
            vmin, vmax = int(show.min()), int(show.max())
            if auto_stretch and vmax > vmin:
                show = cv2.normalize(disp_u8, None, 0, 255, cv2.NORM_MINMAX)

            out = show.copy()
            # ROI-Kasten
            cv2.rectangle(out, (roi_x, roi_y), (roi_x+roi_w, roi_y+roi_h), 255, 2)

            # Status-/Info-Text
            b_h, b_v, d_h, d_v = get_scaling_state(nm)
            w = int(get_val(nm, "Width") or wid)
            h = int(get_val(nm, "Height") or hgt)
            exp = get_float(nm, exp_name)
            # Gain-Knotenname erkennen
            if supports(nm, "Gain"):
                gname = "Gain"
            else:
                gname = "GainRaw"
            gain = get_float(nm, gname)

            lines = [
                f"{model} SN:{serial}  {w}x{h}  PF={pf_str} / {decode_label}",
                f"Focus({name})  raw={raw:,.1f}   ema={ema:,.1f}   ROI={roi_w}x{roi_h}@({roi_x},{roi_y})",
                f"FPS={disp_fps:.1f}  Exp={int(exp) if exp else '—'}us  Gain={gain:.2f}dB  Bin={b_h}x{b_v}  Dec={d_h}x{d_v}  Stretch={'ON' if auto_stretch else 'OFF'}"
            ]
            y=20
            for t in lines:
                cv2.putText(out, t, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, 255, 2, cv2.LINE_AA); y+=22

            cv2.imshow("Live", out)

            # -------- Tastatur ----------
            k = cv2.waitKey(1) & 0xFF
            if k == 27:  # ESC
                break


            # Snapshot: SPACE (32) / P
            elif k == 32 or k in (ord('p'), ord('P')):
                fn = time.strftime("snapshot_%Y%m%d_%H%M%S.png")
                cv2.imwrite(fn, disp_u8)
                print("Snapshot:", fn)

            # Stretch: H
            elif k in (ord('h'), ord('H')):
                auto_stretch = not auto_stretch

            # Pfeile: 81=←, 82=↑, 83=→, 84=↓ (OpenCV-Qt Codes)
            elif k == 106:  # Left
                roi_x -= roi_step
                print("left")
            elif k == 105:  # Up
                roi_y -= roi_step
                print("top")
            elif k == 108:  # Right
                roi_x += roi_step
                print("right")
            elif k == 107:  # Down
                roi_y += roi_step
                print("down")

            # ROI Größe / Zentrum
            elif k == ord('['):
                roi_w = max(32, roi_w - 16); roi_h = max(32, roi_h - 16)
            elif k == ord(']'):
                roi_w = min(wid, roi_w + 16); roi_h = min(hgt, roi_h + 16)
            elif k in (ord('r'), ord('R')):
                roi_w, roi_h = min(256, wid), min(256, hgt)
                roi_x, roi_y = (wid - roi_w)//2, (hgt - roi_h)//2

            # Schärfemetrik: M
            elif k in (ord('m'), ord('M')):
                method_idx = (method_idx + 1) % len(FOCUS_METHODS)
                ema = None

            # Kamera-AOI (mit Neustart)
            elif k in (ord('1'), ord('2'), ord('3')):
                scale = 1.0 if k==ord('1') else 0.5 if k==ord('2') else 0.25
                stop_destroy(ia)
                ia, nm, exp_name, gain_name, pf_str = restart_with_roi(h, device_index, pf_str, scale)
                # Mess-ROI an neue Größe anpassen (Zentrieren)
                hgt, wid = int(get_val(nm, "Height")), int(get_val(nm, "Width"))
                roi_w, roi_h = min(roi_w, wid), min(roi_h, hgt)
                roi_x, roi_y = (wid - roi_w)//2, (hgt - roi_h)//2
                print(f"AOI neu → {wid}x{hgt}")

            # Autofokus (Hill-Climb; misst RAW-Werte auf frischen Frames)
            elif k in (ord('f'), ord('F')):
                print("Starte Autofokus …")
                def get_focus_now():
                    # neues Bild synchron ziehen & RAW-Schärfe auf aktueller ROI und Metrik messen
                    with ia.fetch(timeout=1500) as bf:
                        cp = bf.payload.components[0]
                        im_u8, _ = to_display_u8_copy(cp, pf_str)
                    sub = im_u8[roi_y:roi_y+roi_h, roi_x:roi_x+roi_w]
                    _, fnc = FOCUS_METHODS[method_idx]
                    return float(fnc(sub))
                autofocus_hill_climb(get_focus_now, stepper, start_step=200, min_step=5, settle=0.12)
                print("Autofokus fertig.")

            # Binning / Decimation / Reset
            elif k in (ord('b'), ord('B'), ord('n'), ord('N'), ord('x'), ord('X'), ord('z'), ord('Z'), ord('0')):
                b_h, b_v, d_h, d_v = get_scaling_state(nm)
                if k in (ord('b'), ord('B')):        # Binning +
                    new_factor = max(b_h, b_v) + 1; use_binning = True
                elif k in (ord('n'), ord('N')):      # Binning -
                    new_factor = max(1, max(b_h, b_v) - 1); use_binning = True
                elif k in (ord('x'), ord('X')):      # Decimation +
                    new_factor = max(d_h, d_v) + 1; use_binning = False
                elif k in (ord('z'), ord('Z')):      # Decimation -
                    new_factor = max(1, max(d_h, d_v) - 1); use_binning = False
                else:                                 # '0' = Reset
                    new_factor = 1; use_binning = True

                stop_destroy(ia)
                ia, nm, exp_name, gain_name, pf_str = restart_with_scaling(h, device_index, pf_str, use_binning, new_factor)
                # ROI wieder ins Zentrum
                hgt, wid = int(get_val(nm, "Height")), int(get_val(nm, "Width"))
                roi_w, roi_h = min(roi_w, wid), min(roi_h, hgt)
                roi_x, roi_y = (wid - roi_w)//2, (hgt - roi_h)//2
                b_h, b_v, d_h, d_v = get_scaling_state(nm)
                print(f"Scaling neu → Bin={b_h}x{b_v}  Dec={d_h}x{d_v}  Auflösung={wid}x{hgt}")

            # Exposure / Gain
            elif k in (ord('+'), ord('-'), ord('g'), ord('G')):
                if k in (ord('+'), ord('-')):
                    emin, emax = get_limits(nm, exp_name)
                    cur = get_float(nm, exp_name) or START_EXPOSURE_US
                    factor = 1.5 if k==ord('+') else (1/1.5)
                    new = clamp(cur*factor, emin, emax)
                    set_node(nm, "ExposureAuto", "Off")
                    set_node(nm, exp_name, float(new))
                    print(f"Exposure -> {new:.0f} us")
                if k in (ord('g'), ord('G')):
                    if supports(nm, "Gain"):
                        gname = "Gain"
                    else:
                        gname = "GainRaw"
                    gmin, gmax = get_limits(nm, gname)
                    curg = get_float(nm, gname) or START_GAIN_DB
                    delta = 1.0 if k==ord('G') else -1.0
                    newg = clamp(curg + delta, gmin, gmax)
                    set_node(nm, "GainAuto", "Off")
                    set_node(nm, gname, float(newg))
                    print(f"Gain -> {newg:.2f} dB")

    finally:
        try: stop_destroy(ia)
        except Exception: pass
        h.reset()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    ensure_paths()
    main()
