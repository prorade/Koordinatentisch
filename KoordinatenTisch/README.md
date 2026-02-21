# Basic Flask Anwendung

Kurze Anleitung zum Starten der minimalen Flask-App.

1. Virtuelle Umgebung erstellen (optional):

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
```

2. Abhängigkeiten installieren:

```powershell
pip install -r requirements.txt
```

Baumer neoAPI (lokales Wheel aus dem Repo) installieren (Python-Version muss passen, z.B. cp310/cp311/cp312):

```powershell
pip install .\baumerLib\Baumer_neoAPI_1.5.0_win_x86_64_python\wheel\baumer_neoapi-1.5.0-cp310.cp311.cp312-none-win_amd64.whl
```

3. (Optional) GRBL-Port/FPS/Exposure konfigurieren (per Environment Variablen):

```powershell
$env:GRBL_PORT = "COM3"
$env:GRBL_BAUD = "115200"
$env:CAMERA_EXPOSURE_US = "10000"
$env:CAMERA_FPS = "10"
```

4. App starten:

```powershell
python app.py
```

## Demo Modus (ohne Geräte)

Startet eine simulierte GRBL-Maschine und eine synthetische Kamera mit QR-Codes.

```powershell
$env:DEMO_MODE = "1"
$env:DEMO_POINTS = "12"
$env:DEMO_SPAN_X_MM = "60"
$env:DEMO_SPAN_Y_MM = "40"
# optional reproduzierbar:
# $env:DEMO_SEED = "123"
pip install -r requirements.txt
python app.py
```

Im Browser:
- zuerst „Startposition setzen“ (Demo speichert die aktuelle `MPos`)
- dann „Start“ (Workflow liest eine deterministische QR-Kette bis `END`)

Die App läuft dann unter `http://127.0.0.1:5000`.

Hinweis zu Koordinaten:
- Diese Minimal-App verwendet bewusst Maschinenkoordinaten (GRBL `MPos`) und sendet absolute Fahrten als `G53`.
- Das passt zu Setups, in denen `$H` (Homing) die Maschinen-Null auf `0,0,0` setzt und alle Zielpositionen absolut von dort angegeben werden.
