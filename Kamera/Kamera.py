# baumer_live_focus_full_fixed2.py
# Stabiler Baumer-Viewer mit Fokus-Messfenster, Binning/Decimation (robust), AOI-Neustart, Autofokus-Stub
# WICHTIG: Keine Überschneidung mehr mit Harvester-Variable 'h' (cam_w / cam_h statt w/h)

import os, sys, time, cv2
import numpy as np
from harvesters.core import Harvester

# ==== Pfade/Kamera ===========================================================
BIN_DIR  = r"H:\Baumer_neoAPI_1.5.0_win_x86_64_cpp\bin"   # <— ANPASSEN
CTI_PATH = rf"{BIN_DIR}\bgapi2_gige.cti"                  # <— ANPASSEN
TARGET_IP = "192.168.178.2"                               # optional: Kamera-IP

# ==== Start-Defaults =========================================================
START_EXPOSURE_US = 5000.0
START_GAIN_DB     = 6.0
START_FPS         = 60.0
WANT_PACKET       = 9000
WANT_THROUGHPUT   = 0
DISPLAY_STRETCH   = False

# ==== Arrow-Key Codes (robust) ==============================================
ARROW_LEFT_CODES  = {81, 2424832, 0x250000}
ARROW_UP_CODES    = {82, 2490368, 0x260000}
ARROW_RIGHT_CODES = {83, 2555904, 0x270000}
ARROW_DOWN_CODES  = {84, 2621440, 0x280000}

def ensure_paths():
    if hasattr(os, "add_dll_directory"):
        os.add_dll_directory(BIN_DIR)
    os.environ["PATH"] = BIN_DIR + os.pathsep + os.environ.get("PATH","")
    if not os.path.exists(CTI_PATH):
        print("CTI nicht gefunden:", CTI_PATH); sys.exit(1)

# ==== GenICam Utils ==========================================================
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
    h, w = comp.height, comp.width
    u8 = comp.data
    total = u8.size
    if w==0 or h==0 or total==0:
        return np.zeros((10,10), np.uint8), "Empty"
    pf = (pf_str or "").lower()

    if "mono8" in pf:
        return u8.reshape(h, w).copy(), "Mono8 (8b)"
    if "mono12packed" in pf:
        img12 = decode_mono12packed(u8, w, h); return (img12>>4).astype(np.uint8, copy=True), "Mono12Packed (>>4)"
    if "mono12" in pf and "packed" not in pf:
        img16 = np.frombuffer(u8.tobytes(), dtype=np.uint16, count=w*h).reshape(h, w); return (img16>>4).astype(np.uint8, copy=True), "Mono12 (>>4)"
    if "mono16" in pf:
        img16 = np.frombuffer(u8.tobytes(), dtype=np.uint16, count=w*h).reshape(h, w); return (img16>>8).astype(np.uint8, copy=True), "Mono16 (>>8)"
    if "mono10" in pf and "packed" not in pf:
        img16 = np.frombuffer(u8.tobytes(), dtype=np.uint16, count=w*h).reshape(h, w); return (img16>>2).astype(np.uint8, copy=True), "Mono10 (>>2)"

    bpp = total / (w*h)
    if abs(bpp-1.0) < 1e-3:
        return u8.reshape(h, w).copy(), "8b(?)"
    if abs(bpp-2.0) < 1e-3:
        img16 = np.frombuffer(u8.tobytes(), dtype=np.uint16, count=w*h).reshape(h, w); return (img16>>8).astype(np.uint8, copy=True), "16b(>>8?)"
    if abs(bpp-1.5) < 1e-2 and (w % 2 == 0):
        img12 = decode_mono12packed(u8, w, h); return (img12>>4).astype(np.uint8, copy=True), "12p(>>4?)"

    return u8.reshape(h, -1)[:, :w].copy(), f"Unknown bpp={bpp:.2f}"

# ==== Kamera-Basisconfig =====================================================
def configure_camera_basics(nm):
    for k, v in [("TriggerMode","Off"), ("ExposureMode","Timed"),
                 ("AcquisitionMode","Continuous")]:
        set_node(nm, k, v)

    pf_list = enum_symbolics(nm, "PixelFormat")
    if "Mono8" in pf_list: set_node(nm, "PixelFormat", "Mono8")

    set_node(nm, "ExposureAuto", "Off")
    set_node(nm, "GainAuto", "Off")
    set_node(nm, "GammaEnable", False)
    set_node(nm, "LUTEnable", False)

    bl_min, bl_max = get_limits(nm, "BlackLevel")
    if bl_min is not None and bl_max is not None:
        set_node(nm, "BlackLevel", 0.0)

    exp_name = "ExposureTime" if get_limits(nm, "ExposureTime") != (None, None) else "ExposureTimeAbs"
    emin, emax = get_limits(nm, exp_name)
    set_node(nm, exp_name, float(clamp(START_EXPOSURE_US, emin, emax)))

    gain_name = "Gain" if get_limits(nm, "Gain") != (None, None) else "GainRaw"
    gmin, gmax = get_limits(nm, gain_name)
    set_node(nm, gain_name, float(clamp(START_GAIN_DB, gmin, gmax)))

    set_node(nm, "AcquisitionFrameRateEnable", True)
    fmin, fmax = get_limits(nm, "AcquisitionFrameRate")
    if fmin is not None or fmax is not None:
        set_node(nm, "AcquisitionFrameRate", float(clamp(START_FPS, fmin, fmax)))

    set_node(nm, "DeviceLinkThroughputLimitMode", "Off")
    if WANT_THROUGHPUT and WANT_THROUGHPUT > 0:
        set_node(nm, "DeviceLinkThroughputLimitMode", "On")
        set_node(nm, "DeviceLinkThroughputLimit", int(WANT_THROUGHPUT))

    for p in [WANT_PACKET, 9000, 8192, 3000, 1500]:
        if p and set_node(nm, "GevSCPSPacketSize", int(p)): break

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

# ==== Binning / Decimation (robust) =========================================
def apply_binning(nm, factor, mode="Average"):
    for n in ("DecimationHorizontal", "DecimationVertical"):
        if supports(nm, n): set_node(nm, n, 1)
    if not (supports(nm,"BinningHorizontal") and supports(nm,"BinningVertical")):
        return False, (1, 1)

    bminH, bmaxH = get_limits(nm, "BinningHorizontal")
    bminV, bmaxV = get_limits(nm, "BinningVertical")
    incH = get_inc(nm, "BinningHorizontal", 1)
    incV = get_inc(nm, "BinningVertical", 1)

    def clamp_factor(val, mn, mx, inc):
        if mn is None: mn = 1
        if mx is None: mx = max(1, int(val))
        val = max(mn, min(mx, int(val)))
        if inc > 1: val = mn + ((val - mn) // inc) * inc
        return max(1, int(val))

    fH = clamp_factor(factor, bminH, bmaxH, incH)
    fV = clamp_factor(factor, bminV, bmaxV, incV)

    if supports(nm, "BinningMode"):
        syms = enum_symbolics(nm, "BinningMode")
        if mode in syms: set_node(nm, "BinningMode", mode)
        elif "Average" in syms: set_node(nm, "BinningMode", "Average")
        elif "Sum" in syms: set_node(nm, "BinningMode", "Sum")

    okH = set_node(nm, "BinningHorizontal", int(fH))
    okV = set_node(nm, "BinningVertical",   int(fV))
    return (okH and okV), (int(fH), int(fV))

def apply_decimation(nm, factor):
    for n in ("BinningHorizontal","BinningVertical"):
        if supports(nm, n): set_node(nm, n, 1)
    if not (supports(nm,"DecimationHorizontal") and supports(nm,"DecimationVertical")):
        return False, (1, 1)

    dminH, dmaxH = get_limits(nm, "DecimationHorizontal")
    dminV, dmaxV = get_limits(nm, "DecimationVertical")
    incH = get_inc(nm, "DecimationHorizontal", 1)
    incV = get_inc(nm, "DecimationVertical", 1)

    def clamp_factor(val, mn, mx, inc):
        if mn is None: mn = 1
        if mx is None: mx = max(1, int(val))
        val = max(mn, min(mx, int(val)))
        if inc > 1: val = mn + ((val - mn) // inc) * inc
        return max(1, int(val))

    fH = clamp_factor(factor, dminH, dmaxH, incH)
    fV = clamp_factor(factor, dminV, dmaxV, incV)

    okH = set_node(nm, "DecimationHorizontal", int(fH))
    okV = set_node(nm, "DecimationVertical",   int(fV))
    return (okH and okV), (int(fH), int(fV))

def get_scaling_state(nm):
    b_h = get_int(nm, "BinningHorizontal") if supports(nm,"BinningHorizontal") else 1
    b_v = get_int(nm, "BinningVertical")   if supports(nm,"BinningVertical")   else 1
    d_h = get_int(nm, "DecimationHorizontal") if supports(nm,"DecimationHorizontal") else 1
    d_v = get_int(nm, "DecimationVertical")   if supports(nm,"DecimationVertical")   else 1
    return (b_h or 1), (b_v or 1), (d_h or 1), (d_v or 1)

# ==== Harvester Lifecycle ====================================================
def create_ia(h, device_index):
    ia = h.create(device_index)
    try: ia.num_buffers = 4
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
    ia_tmp = create_ia(h, device_index)
    nm = ia_tmp.remote_device.node_map
    set_node(nm, "TLParamsLocked", 0)
    try: nm.get_node("AcquisitionStop").execute()
    except Exception: pass

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

    ok = False
    info = ""
    try:
        if use_binning:
            ok, bf = apply_binning(nm, factor, mode="Average"); info = f"Binning={bf[0]}x{bf[1]}"
            if not ok:
                ok, df = apply_decimation(nm, factor); info = f"Decimation={df[0]}x{df[1]}"
        else:
            ok, df = apply_decimation(nm, factor); info = f"Decimation={df[0]}x{df[1]}"
            if not ok:
                ok, bf = apply_binning(nm, factor, mode="Average"); info = f"Binning={bf[0]}x{bf[1]}"
        set_roi_full(nm)  # nach Scaling
    except Exception as e:
        print("Scaling-Fehler:", e); ok = False

    set_node(nm, "TLParamsLocked", 1)
    stop_destroy(ia_tmp)

    ia = create_ia(h, device_index)
    nm = ia.remote_device.node_map
    exp_name, gain_name = configure_camera_basics(nm)
    pf_str = str(get_val(nm, "PixelFormat")) or (pf_hint or "Mono8")
    ia.start()

    if ok:
        print("Scaling OK →", info, f"  Auflösung={int(get_val(nm,'Width'))}x{int(get_val(nm,'Height'))}")
    else:
        print("Scaling NICHT unterstützt oder fehlgeschlagen – Stream ohne Änderung neu gestartet.")
    return ia, nm, exp_name, gain_name, pf_str

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
    def move_relative(self, steps: int):
        print(f"[Stepper] move_relative({steps}) (Stub)")
    def wait_settle(self, seconds: float = 0.12):
        time.sleep(seconds)

def autofocus_hill_climb(get_focus_value, stepper: StepperController,
                         start_step=200, min_step=5, settle=0.12):
    cur = get_focus_value()
    step = start_step
    direction = +1
    improved = True
    while step >= min_step:
        stepper.move_relative(direction*step)
        stepper.wait_settle(settle)
        val = get_focus_value()
        if val > cur:
            cur = val; improved = True
        else:
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

    device_index = next((i for i,d in enumerate(h.device_info_list) if TARGET_IP in str(d)), 0)

    ia = create_ia(h, device_index)
    nm = ia.remote_device.node_map
    exp_name, gain_name = configure_camera_basics(nm)
    pf_str = str(get_val(nm, "PixelFormat")) or "Mono8"
    model  = get_val(nm, "DeviceModelName")
    serial = get_val(nm, "DeviceSerialNumber")

    set_roi_full(nm)
    ia.start()

    cv2.namedWindow("Live", cv2.WINDOW_AUTOSIZE)
    print("Tasten: ←↑→↓ ROI (J/I/L/K Fallback)  [ ] Größe  R Center  M Metrik  H Stretch  "
          "B/N Bin ±  X/Z Dec ±  0 Reset  1/2/3 AOI  +/- Exp  G/g Gain  SPACE/P Snapshot  F AF  ESC")

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
            with ia.fetch(timeout=1500) as buf:
                comp = buf.payload.components[0]
                disp_u8, decode_label = to_display_u8_copy(comp, pf_str)

            cam_h, cam_w = disp_u8.shape[:2]   # <-- KEIN 'h' überschreiben!
            roi_w = max(32, min(roi_w, cam_w))
            roi_h = max(32, min(roi_h, cam_h))
            roi_x = min(max(0, roi_x), cam_w - roi_w)
            roi_y = min(max(0, roi_y), cam_h - roi_h)

            name, func = FOCUS_METHODS[method_idx]
            roi = disp_u8[roi_y:roi_y+roi_h, roi_x:roi_x+roi_w]
            raw = func(roi)
            if ema is None: ema = raw
            else: ema = ema_alpha*raw + (1.0-ema_alpha)*ema

            frames += 1
            now = time.time()
            if now - last >= 1.0:
                disp_fps = frames/(now-last); frames=0; last=now

            show = disp_u8
            vmin, vmax = int(show.min()), int(show.max())
            if auto_stretch and vmax > vmin:
                show = cv2.normalize(disp_u8, None, 0, 255, cv2.NORM_MINMAX)

            out = show.copy()
            cv2.rectangle(out, (roi_x, roi_y), (roi_x+roi_w, roi_y+roi_h), 255, 2)

            b_h, b_v, d_h, d_v = get_scaling_state(nm)
            cam_w_node = int(get_val(nm, "Width") or cam_w)
            cam_h_node = int(get_val(nm, "Height") or cam_h)
            exp = get_float(nm, exp_name)
            gname = "Gain" if supports(nm, "Gain") else "GainRaw"
            gain = get_float(nm, gname)

            lines = [
                f"{model} SN:{serial}  {cam_w_node}x{cam_h_node}  PF={pf_str} / {decode_label}",
                f"Focus({name})  raw={raw:,.1f}   ema={ema:,.1f}   ROI={roi_w}x{roi_h}@({roi_x},{roi_y})",
                f"FPS={disp_fps:.1f}  Exp={int(exp) if exp else '—'}us  Gain={gain:.2f}dB  Bin={b_h}x{b_v}  Dec={d_h}x{d_v}  Stretch={'ON' if auto_stretch else 'OFF'}"
            ]
            y=20
            for t in lines:
                cv2.putText(out, t, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, 255, 2, cv2.LINE_AA); y+=22

            cv2.imshow("Live", out)

            # --- Tastatur (waitKeyEx für Pfeile) ---
            k = cv2.waitKeyEx(1)
            if k == 27:  # ESC
                break

            elif k == 32 or k in (ord('p'), ord('P')):  # SPACE / P
                fn = time.strftime("snapshot_%Y%m%d_%H%M%S.png")
                cv2.imwrite(fn, disp_u8); print("Snapshot:", fn)

            elif k in (ord('h'), ord('H')):
                auto_stretch = not auto_stretch

            # Pfeile (robust) + Fallback I/J/K/L
            elif (k in ARROW_LEFT_CODES) or (k in (ord('j'), ord('J'))):
                roi_x -= roi_step
            elif (k in ARROW_UP_CODES) or (k in (ord('i'), ord('I'))):
                roi_y -= roi_step
            elif (k in ARROW_RIGHT_CODES) or (k in (ord('l'), ord('L'))):
                roi_x += roi_step
            elif (k in ARROW_DOWN_CODES) or (k in (ord('k'), ord('K'))):
                roi_y += roi_step

            # ROI Größe / Zentrieren
            elif k == ord('['):
                roi_w = max(32, roi_w - 16); roi_h = max(32, roi_h - 16)
            elif k == ord(']'):
                roi_w = min(cam_w, roi_w + 16); roi_h = min(cam_h, roi_h + 16)
            elif k in (ord('r'), ord('R')):
                roi_w, roi_h = min(256, cam_w), min(256, cam_h)
                roi_x, roi_y = (cam_w - roi_w)//2, (cam_h - roi_h)//2

            # Schärfemetrik
            elif k in (ord('m'), ord('M')):
                method_idx = (method_idx + 1) % len(FOCUS_METHODS)
                ema = None

            # Kamera-AOI (mit Neustart)
            elif k in (ord('1'), ord('2'), ord('3')):
                scale = 1.0 if k==ord('1') else 0.5 if k==ord('2') else 0.25
                stop_destroy(ia)
                ia, nm, exp_name, gain_name, pf_str = restart_with_roi(h, device_index, pf_str, scale)
                cam_h, cam_w = int(get_val(nm, "Height")), int(get_val(nm, "Width"))
                roi_w, roi_h = min(roi_w, cam_w), min(roi_h, cam_h)
                roi_x, roi_y = (cam_w - roi_w)//2, (cam_h - roi_h)//2
                print(f"AOI neu → {cam_w}x{cam_h}")

            # Autofokus (RAW auf frischen Frames)
            elif k in (ord('f'), ord('F')):
                print("Starte Autofokus …")
                def get_focus_now():
                    with ia.fetch(timeout=1500) as bf:
                        cp = bf.payload.components[0]
                        im_u8, _ = to_display_u8_copy(cp, pf_str)
                    sub = im_u8[roi_y:roi_y+roi_h, roi_x:roi_x+roi_w]
                    _, fnc = FOCUS_METHODS[method_idx]
                    return float(fnc(sub))
                autofocus_hill_climb(get_focus_now, stepper, start_step=200, min_step=5, settle=0.12)
                print("Autofokus fertig.")

            # Binning / Decimation / Reset (robust)
            elif k in (ord('b'), ord('B'), ord('n'), ord('N'), ord('x'), ord('X'), ord('z'), ord('Z'), ord('0')):
                try:
                    b_h, b_v, d_h, d_v = get_scaling_state(nm)
                    if k in (ord('b'), ord('B')):        # Binning +
                        desired = max(b_h, b_v) + 1; use_binning = True
                    elif k in (ord('n'), ord('N')):      # Binning -
                        desired = max(1, max(b_h, b_v) - 1); use_binning = True
                    elif k in (ord('x'), ord('X')):      # Decimation +
                        desired = max(d_h, d_v) + 1; use_binning = False
                    elif k in (ord('z'), ord('Z')):      # Decimation -
                        desired = max(1, max(d_h, d_v) - 1); use_binning = False
                    else:                                 # '0' Reset
                        desired = 1; use_binning = True

                    if (use_binning and desired == max(b_h, b_v)) or ((not use_binning) and desired == max(d_h, d_v)):
                        print("Scaling unverändert – keine Aktion.")
                    else:
                        stop_destroy(ia)
                        ia, nm, exp_name, gain_name, pf_str = restart_with_scaling(
                            h, device_index, pf_str, use_binning, desired
                        )
                        cam_h, cam_w = int(get_val(nm, "Height")), int(get_val(nm, "Width"))
                        roi_w, roi_h = min(roi_w, cam_w), min(roi_h, cam_h)
                        roi_x, roi_y = (cam_w - roi_w)//2, (cam_h - roi_h)//2

                except Exception as e:
                    print("Fehler beim Scaling:", e)
                    try:
                        ia, nm, exp_name, gain_name, pf_str = restart_stream(h, device_index, pf_str)
                    except Exception as ee:
                        print("Restart-Fehler:", ee)

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
                    gname = "Gain" if supports(nm, "Gain") else "GainRaw"
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
        # ACHTUNG: Hier ist 'h' wieder der Harvester!
        try: h.reset()
        except Exception: pass
        cv2.destroyAllWindows()

if __name__ == "__main__":
    ensure_paths()
    main()
