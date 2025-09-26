#include <AccelStepper.h>

/***  CNC-Shield V3 Pinning  ***/
const int X_STEP = 2;  const int X_DIR = 5;
const int Y_STEP = 3;  const int Y_DIR = 6;
const int Z_STEP = 4;  const int Z_DIR = 7;
const int EN_PIN = 8;

/***  Steps/mm (DEINE kalibrierten Werte einsetzen)  ***/
float X_STEPS_PER_MM = 410.3f;   // X kalibriert
float Y_STEPS_PER_MM = 390.0f;   // Y kalibriert
float Z_STEPS_PER_MM = 2222.2f;  // Z kalibriert (M5x0.8 @ 1/8 µStep ~2000..2222)

/***  Soft-Limits (mm)  ***/
const float X_MIN_MM = 0.0f, X_MAX_MM = 140.0f;
const float Y_MIN_MM = 0.0f, Y_MAX_MM = 70.0f;
float       Z_MIN_MM = 0.0f, Z_MAX_MM = 60.0f;   // Z-Max bei Bedarf per Befehl änderbar

/***  Bewegungsprofil (Steps/s, Steps/s^2)  ***/
const float X_MAXSPEED = 5000;   // ~ 12 mm/s bei deinen Steps/mm
const float Y_MAXSPEED = 5000;
const float Z_MAXSPEED = 3000;   // ~ 1.3 mm/s bei Z-Steps/mm
const float X_ACCEL    = 2000;
const float Y_ACCEL    = 2000;
const float Z_ACCEL    = 800;

/***  Stepper-Objekte  ***/
AccelStepper X(AccelStepper::DRIVER, X_STEP, X_DIR);
AccelStepper Y(AccelStepper::DRIVER, Y_STEP, Y_DIR);
AccelStepper Z(AccelStepper::DRIVER, Z_STEP, Z_DIR);

/***  Raster-Scan-Parameter (XY)  ***/
float dx = 10.0f, dy = 10.0f;     // Rasterabstände
float spanX = 50.0f, spanY = 30.0f;
int   nx = 0, ny = 0;

/***  Nullpunkt (Origin)  ***/
float originX = 0.0f, originY = 0.0f, originZ = 0.0f;
bool  backAfterScan = true;       // nach Scan automatisch zu (originX,originY)

/***  Entnahmeposition (Pickup)  ***/
float pickX = 0.0f, pickY = 0.0f, pickZ = 0.0f;

/***  Steuerflags  ***/
bool abortScan = false;

/***  Helpers: Umrechnung & Limits  ***/
long mmToStepsX(float mm){ return (long)(mm * X_STEPS_PER_MM); }
long mmToStepsY(float mm){ return (long)(mm * Y_STEPS_PER_MM); }
long mmToStepsZ(float mm){ return (long)(mm * Z_STEPS_PER_MM); }

float clampX(float x){ if (x < X_MIN_MM) return X_MIN_MM; if (x > X_MAX_MM) return X_MAX_MM; return x; }
float clampY(float y){ if (y < Y_MIN_MM) return Y_MIN_MM; if (y > Y_MAX_MM) return Y_MAX_MM; return y; }
float clampZ(float z){ if (z < Z_MIN_MM) return Z_MIN_MM; if (z > Z_MAX_MM) return Z_MAX_MM; return z; }

/***  Bewegungen  ***/
void goToXY_mm(float x_mm, float y_mm) {
  x_mm = clampX(x_mm);  y_mm = clampY(y_mm);
  X.moveTo(mmToStepsX(x_mm));
  Y.moveTo(mmToStepsY(y_mm));
  while (X.distanceToGo()!=0 || Y.distanceToGo()!=0) {
    if (abortScan) return;
    X.run(); Y.run();
  }
}

void goToZ_mm(float z_mm) {
  z_mm = clampZ(z_mm);
  Z.moveTo(mmToStepsZ(z_mm));
  while (Z.distanceToGo()!=0) {
    if (abortScan) return;
    Z.run();
  }
}

void goToXYZ_mm(float x_mm, float y_mm, float z_mm) {
  x_mm = clampX(x_mm);  y_mm = clampY(y_mm);  z_mm = clampZ(z_mm);
  X.moveTo(mmToStepsX(x_mm));
  Y.moveTo(mmToStepsY(y_mm));
  Z.moveTo(mmToStepsZ(z_mm));
  while (X.distanceToGo()!=0 || Y.distanceToGo()!=0 || Z.distanceToGo()!=0) {
    if (abortScan) return;
    X.run(); Y.run(); Z.run();
  }
}

/***  Infoausgaben  ***/
void printHelp(){
  Serial.println(F("Befehle (115200 Baud):"));
  Serial.println(F("  H            : Hilfe"));
  Serial.println(F("  P            : Parameter anzeigen"));
  Serial.println(F("  O            : aktuellen Punkt als Origin XY setzen"));
  Serial.println(F("  OZ           : aktuellen Z als Origin Z setzen"));
  Serial.println(F("  O0           : aktuellen Punkt als Origin XYZ setzen"));
  Serial.println(F("  S            : Raster-Scan starten (XY, Serpentine)"));
  Serial.println(F("  R            : XY zum Nullpunkt (originX,originY)"));
  Serial.println(F("  RZ0          : Z zum Nullpunkt (originZ)"));
  Serial.println(F("  R0           : XYZ zum Nullpunkt (originX,Y,Z)"));
  Serial.println(F("  BA 0/1       : Auto-Rueckfahrt nach Scan AUS/AN"));
  Serial.println(F("  X <mm>       : X ABSOLUT anfahren"));
  Serial.println(F("  Y <mm>       : Y ABSOLUT anfahren"));
  Serial.println(F("  Z <mm>       : Z ABSOLUT anfahren"));
  Serial.println(F("  RX <mm>      : X RELATIV verfahren"));
  Serial.println(F("  RY <mm>      : Y RELATIV verfahren"));
  Serial.println(F("  RZ <mm>      : Z RELATIV verfahren"));
  Serial.println(F("  g <x> <y>    : ABSOLUT zu (x,y)"));
  Serial.println(F("  g <x> <y> <z>: ABSOLUT zu (x,y,z)"));
  Serial.println(F("  ZMAX <mm>    : Z-Max-Softlimit setzen"));
  Serial.println(F("  PSET         : aktuelle XYZ als Entnahme speichern"));
  Serial.println(F("  P <x> <y> <z>: Entnahmeposition setzen"));
  Serial.println(F("  PE           : Entnahmeposition XYZ anfahren"));
  Serial.println(F("  PEXY         : Entnahmeposition nur XY anfahren"));
  Serial.println(F("  P?           : Entnahmeposition anzeigen"));
  Serial.println(F("  !            : Abbrechen/Stop"));
  Serial.println();
}

void printParams(){
  Serial.println(F("=== Parameter ==="));
  Serial.print(F("Steps/mm: X=")); Serial.print(X_STEPS_PER_MM,3);
  Serial.print(F("  Y=")); Serial.print(Y_STEPS_PER_MM,3);
  Serial.print(F("  Z=")); Serial.println(Z_STEPS_PER_MM,3);
  Serial.print(F("Soft-Limits: X[0..")); Serial.print(X_MAX_MM,1);
  Serial.print(F("]  Y[0..")); Serial.print(Y_MAX_MM,1);
  Serial.print(F("]  Z["));
  Serial.print(Z_MIN_MM,1); Serial.print(F("..")); Serial.print(Z_MAX_MM,1); Serial.println(F("] mm"));
  Serial.print(F("Origin XYZ: ("));
  Serial.print(originX,3); Serial.print(F(", "));
  Serial.print(originY,3); Serial.print(F(", "));
  Serial.print(originZ,3); Serial.println(F(") mm"));
  Serial.print(F("Raster: dx=")); Serial.print(dx,3);
  Serial.print(F("  dy=")); Serial.print(dy,3);
  Serial.print(F("  spanX=")); Serial.print(spanX,3);
  Serial.print(F("  spanY=")); Serial.println(spanY,3);
  Serial.print(F("Auto-Rueckfahrt nach Scan: "));
  Serial.println(backAfterScan ? F("AN") : F("AUS"));
  Serial.print(F("Entnahme XYZ: ("));
  Serial.print(pickX,3); Serial.print(F(", "));
  Serial.print(pickY,3); Serial.print(F(", "));
  Serial.print(pickZ,3); Serial.println(F(") mm"));
  Serial.println(F("================="));
}

/***  Raster-Scan (XY)  ***/
void doScanSerpentine(){
  float x0 = originX, y0 = originY;

  // Start zum Nullpunkt XY
  abortScan = false;
  goToXY_mm(x0, y0); if (abortScan) return;

  // Anzahl Punkte
  nx = (dx>0) ? (int)(spanX/dx)+1 : 1;
  ny = (dy>0) ? (int)(spanY/dy)+1 : 1;
  if (nx<1) nx=1; if (ny<1) ny=1;

  Serial.print(F("Starte Scan: nx=")); Serial.print(nx);
  Serial.print(F(" ny=")); Serial.println(ny);

  for (int j=0; j<ny && !abortScan; ++j) {
    float y = clampY(y0 + j*dy);
    if ((j % 2) == 0) {
      for (int i=0; i<nx && !abortScan; ++i) {
        float x = clampX(x0 + i*dx);
        goToXY_mm(x, y);
      }
    } else {
      for (int i=nx-1; i>=0 && !abortScan; --i) {
        float x = clampX(x0 + i*dx);
        goToXY_mm(x, y);
      }
    }
    if (j < ny-1 && !abortScan) {
      float yNext = clampY(y0 + (j+1)*dy);
      goToXY_mm(X.currentPosition()/X_STEPS_PER_MM, yNext);
    }
  }

  if (abortScan) {
    Serial.println(F("Scan abgebrochen."));
  } else {
    Serial.println(F("Scan fertig."));
    if (backAfterScan) {
      Serial.println(F("Zurueck zu Origin XY..."));
      goToXY_mm(originX, originY);
      Serial.println(F("Am Origin XY."));
    }
  }
}

/***  Setup / Loop  ***/
void setup(){
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);  // Treiber aktiv

  X.setMaxSpeed(X_MAXSPEED);  X.setAcceleration(X_ACCEL);
  Y.setMaxSpeed(Y_MAXSPEED);  Y.setAcceleration(Y_ACCEL);
  Z.setMaxSpeed(Z_MAXSPEED);  Z.setAcceleration(Z_ACCEL);

  Serial.begin(115200);
  delay(300);
  Serial.println(F("\nSystem bereit. 'H' fuer Hilfe."));
  printParams();
}

void loop(){
  if (Serial.available()){
    String cmd = Serial.readStringUntil('\n'); cmd.trim();

    if (cmd.equalsIgnoreCase("H")) { printHelp(); }
    else if (cmd.equalsIgnoreCase("P")) { printParams(); }

    else if (cmd.equalsIgnoreCase("O"))  { originX = X.currentPosition()/X_STEPS_PER_MM; originY = Y.currentPosition()/Y_STEPS_PER_MM; Serial.println(F("Origin XY gesetzt.")); }
    else if (cmd.equalsIgnoreCase("OZ")) { originZ = Z.currentPosition()/Z_STEPS_PER_MM; Serial.println(F("Origin Z gesetzt.")); }
    else if (cmd.equalsIgnoreCase("O0")) { originX = X.currentPosition()/X_STEPS_PER_MM; originY = Y.currentPosition()/Y_STEPS_PER_MM; originZ = Z.currentPosition()/Z_STEPS_PER_MM; Serial.println(F("Origin XYZ gesetzt.")); }

    else if (cmd.equalsIgnoreCase("S"))  { abortScan = false; doScanSerpentine(); }
    else if (cmd.equals("!"))            { abortScan = true; }

    else if (cmd.equalsIgnoreCase("R"))  { abortScan = true; goToXY_mm(originX, originY); Serial.println(F("Am Origin XY.")); }
    else if (cmd.equalsIgnoreCase("RZ0")){ abortScan = true; goToZ_mm(originZ);          Serial.println(F("Am Origin Z.")); }
    else if (cmd.equalsIgnoreCase("R0")) { abortScan = true; goToXYZ_mm(originX, originY, originZ); Serial.println(F("Am Origin XYZ.")); }

    else if (cmd.startsWith("BA ")) { backAfterScan = (cmd.substring(3).toInt()!=0); Serial.print(F("backAfterScan = ")); Serial.println(backAfterScan?F("AN"):F("AUS")); }

    else if (cmd.startsWith("X "))   { float v = cmd.substring(2).toFloat(); goToXY_mm(clampX(v), Y.currentPosition()/Y_STEPS_PER_MM); }
    else if (cmd.startsWith("Y "))   { float v = cmd.substring(2).toFloat(); goToXY_mm(X.currentPosition()/X_STEPS_PER_MM, clampY(v)); }
    else if (cmd.startsWith("Z "))   { float v = cmd.substring(2).toFloat(); goToZ_mm(clampZ(v)); }
    else if (cmd.startsWith("RX "))  { float v = cmd.substring(3).toFloat(); float cx = X.currentPosition()/X_STEPS_PER_MM; goToXY_mm(clampX(cx+v), Y.currentPosition()/Y_STEPS_PER_MM); }
    else if (cmd.startsWith("RY "))  { float v = cmd.substring(3).toFloat(); float cy = Y.currentPosition()/Y_STEPS_PER_MM; goToXY_mm(X.currentPosition()/X_STEPS_PER_MM, clampY(cy+v)); }
    else if (cmd.startsWith("RZ "))  { float v = cmd.substring(3).toFloat(); float cz = Z.currentPosition()/Z_STEPS_PER_MM; goToZ_mm(clampZ(cz+v)); }

    else if (cmd.startsWith("g ")) {
      // g x y    oder   g x y z
      float tx=0, ty=0, tz=Z.currentPosition()/Z_STEPS_PER_MM;
      int sp1 = cmd.indexOf(' ');
      int sp2 = cmd.indexOf(' ', sp1+1);
      if (sp2>sp1){
        int sp3 = cmd.indexOf(' ', sp2+1);
        tx = cmd.substring(sp1+1, sp2).toFloat();
        if (sp3==-1) {
          ty = cmd.substring(sp2+1).toFloat();
          goToXY_mm(tx, ty);
        } else {
          ty = cmd.substring(sp2+1, sp3).toFloat();
          tz = cmd.substring(sp3+1).toFloat();
          goToXYZ_mm(tx, ty, tz);
        }
      }
    }

    else if (cmd.startsWith("ZMAX ")) { Z_MAX_MM = cmd.substring(5).toFloat(); Serial.print(F("Z_MAX_MM = ")); Serial.println(Z_MAX_MM,3); }

    else if (cmd.equalsIgnoreCase("PSET")) { pickX = X.currentPosition()/X_STEPS_PER_MM; pickY = Y.currentPosition()/Y_STEPS_PER_MM; pickZ = Z.currentPosition()/Z_STEPS_PER_MM; Serial.println(F("Entnahme XYZ gelernt.")); }
    else if (cmd.startsWith("P?")) { Serial.print(F("Entnahme XYZ: (")); Serial.print(pickX,3); Serial.print(F(", ")); Serial.print(pickY,3); Serial.print(F(", ")); Serial.print(pickZ,3); Serial.println(F(") mm")); }
    else if (cmd.startsWith("P ")) {
      // P x y z
      float px=pickX, py=pickY, pz=pickZ;
      int sp1 = cmd.indexOf(' ');
      int sp2 = cmd.indexOf(' ', sp1+1);
      int sp3 = cmd.indexOf(' ', sp2+1);
      if (sp1>0 && sp2>sp1 && sp3>sp2) {
        px = cmd.substring(sp1+1, sp2).toFloat();
        py = cmd.substring(sp2+1, sp3).toFloat();
        pz = cmd.substring(sp3+1).toFloat();
        pickX=clampX(px); pickY=clampY(py); pickZ=clampZ(pz);
        Serial.println(F("Entnahmeposition gesetzt."));
      } else {
        Serial.println(F("Format: P <x> <y> <z>"));
      }
    }
    else if (cmd.equalsIgnoreCase("PE"))   { goToXYZ_mm(pickX, pickY, pickZ); Serial.println(F("Entnahme XYZ angefahren.")); }
    else if (cmd.equalsIgnoreCase("PEXY")) { goToXY_mm(pickX, pickY);         Serial.println(F("Entnahme XY angefahren.")); }

    else { Serial.println(F("Unbekannter Befehl. 'H' fuer Hilfe.")); }
  }

  X.run(); Y.run(); Z.run();
}
