// https://github.com/adafruit/Sous_Viduino/blob/master/Sous_Viduino.ino
// https://playground.arduino.cc/Code/PIDAutotuneLibrary/
// http://brettbeauregard.com/blog/2012/01/arduino-pid-autotune-library/#more-1893
#include <Wire.h>
#include <LiquidCrystal_PCF8574.h>
#include <CapacitiveSensor.h>
#include <OneWire.h>
#include <PID_v1.h>
#include <EEPROM.h>

/* ======================== PIN SET UP ========================= */

/* PIN      Usage
    0       -
    1       -
    2       Scroll wheel sensor A
    3 ~     Scroll wheel sensor B
    4       Common button PIN
    5 ~     OK Button sensor
    6 ~     Cancel Button sensor
    7
    8
    9 ~     Piezo buzzer
   10 ~
   11 ~
   12       Transistor (for 12V relay)
   13       Temperature sensors
   A0
   A1
   A2
   A3
   A4       LCD display (SDA)
   A5       LCD display (SCL)
*/

/* ======================== DEFINITIONS ========================= */
#define DEBUG

#define SW_Version              "v 0.6" // Max 5 chars

#define PIN_SWSensorA           2
#define PIN_SWSensorB           3
#define PIN_BtnCommon           4
#define PIN_BtnOK               5
#define PIN_BtnCancel           6
#define PIN_Buzzer              9
#define PIN_RelayTransistor     12
#define PIN_TempSensors         13

#define PID_Kp              100   // Find this by auto tune!
#define PID_Ki              0.1
#define PID_Kd              5.0

#define PID_Kp_Agg          150
#define PID_Ki_Agg          0.0
#define PID_Kd_Agg          0.0
#define PID_Agg_Threshold   15.0 // Degrees below setpoint where we go from aggressive to normal PID values
#define PID_SampleTime      10000 // The frequency with witch the PID calculation is performed
#define PID_WindowSize      10000

#define LCD_Width         16
#define LCD_Height        2
#define LCD_Address       0x27
#define LCD_Brightnes     255

#define BUZZER_Frequency  2100
#define BUZZER_Duration   100
#define BUZZER_Delay      100

#define BUTTON_DebounceTime 500
#define BUTTON_ReadTime     30
#define BUTTON_CapThreshold 1700
#define BUTTON_OK           true
#define BUTTON_Cancel       false

#define TEMP_TempReadingFrequency 1000 // Should not be below 1000
#define TEMP_MaxHeaterTemp 70.0
#define TEMP_MaxWaterTemp  90.0
#define TEMP_MinWaterTemp  30.0
#define TEMP_Increment      0.1
#define TEMP_MaxWaterTempDiff 0.05

#define TIME_MaxTime    599 // in minutes
#define TIME_MinTime    10   // in minutes
#define TIME_Increment  1.0 // in minutes
#define TIME_CheckWaterTempTimer 300000 // Time in millisec after which the water temp must have risen

/* ======================== GLOBAL VARIABLES ========================= */


// =============== Mode handling ===============

enum            Mode {MODE_check, MODE_start, MODE_setTemp, MODE_setTime, MODE_warmUp, MODE_ready, MODE_cook, MODE_editParams, MODE_manual, MODE_error, MODE_stop};
String          operatingModeName[3] = {"Automatic       ", "Manual          ", "Edit parameters "};
Mode            mode = MODE_check;
byte            lastMode = MODE_check;
byte            lastError = 0;
unsigned long   modeTimer = 0; // Generic, mode common timer

// =============== Scroll Wheel sensor ===============
volatile unsigned int tmpA = 0;
volatile unsigned int tmpB = 0;
volatile unsigned int oldA = 0;
volatile unsigned int oldB = 0;
volatile float        val = 0.0;
volatile float        oldVal = 0.0;
volatile float        swIncrement = 0.1;
bool                  scrollWheelEnabled = false;

// =============== Buttons ===============
unsigned long OKClickTime = 0;
unsigned long CancelClickTime = 0;
unsigned long OKForcedDown = 0;
unsigned long CancelForcedDown = 0;
bool          currentOKState = false;
bool          currentCancelState = false;

// =============== Temp sensors ===============
byte            tempSensorAddress[2][8] = {{0x28, 0xD5, 0x77, 0xCA, 0x06, 0x00, 0x00, 0xFE}, {0x28, 0xFF, 0xCA, 0x7A, 0x86, 0x16, 0x05, 0xAE}};
float           tempHeaterCurrent = 0.00;
float           tempWaterCurrent =  0.00;
float           tempWaterAim =      57.00;
float           tempWaterStart = 0.00;
bool            readTemps = false;
unsigned long   tempReadingTime = 0;

// =============== PID ===============
int           PIDWindowSize = PID_WindowSize;
unsigned long PIDWindowStartTime = 0;
float         PIDRelayOnTime = 0.0;
float         PIDParams[9] = {PID_Kp, PID_Ki, PID_Kd, PID_Kp_Agg, PID_Ki_Agg, PID_Kd_Agg, PID_Agg_Threshold, PID_SampleTime, PID_WindowSize};
String        PIDParamsName[9] = {"kP", "kI", "kD:", "kP (aggressive):", "kI (aggressive):", "kD (aggressive):", "Aggr. threshold:", "Sample time:", "Window size:"};
float         PIDParamsMin[9] = {1, 0, 0, 1, 0, 0, 5, 500, 1000}; // Min PID parameter values
float         PIDParamsMax[9] = {180, 10, 10, 200, 20, 20, 40, 30000, 60000}; // Max PID parameter values
byte          PIDParamsDecimals[9] = {0, 1, 1, 0, 1, 1, 0, 0, 0}; // Nr of decimals for each parameter
bool          PIDEnabled = false;

// =============== Misc. ===============
long            cookingTimeMinutes = 90;  // in minutes
long            timeRemaining = 0;     // In seconds
unsigned long   cookingEndTime = 0;  // In milliseconds
byte            buzzerTimer = 0;
long            nowRemainingSeconds = 0; // In seconds
bool            heaterOn = false;
byte            parameterNr = 0; // The parameter that is edited just now
bool            paramFirstEdit = true;

LiquidCrystal_PCF8574   lcd(LCD_Address);
OneWire                 tempSensors(PIN_TempSensors);
PID                     thePID((double *)&tempWaterCurrent, (double *)&PIDRelayOnTime, (double *)&tempWaterAim, (double)PID_Kp, (double)PID_Ki, (double)PID_Kd, DIRECT);
CapacitiveSensor        btnOK     = CapacitiveSensor(PIN_BtnCommon, PIN_BtnOK);
CapacitiveSensor        btnCancel = CapacitiveSensor(PIN_BtnCommon, PIN_BtnCancel);

/* ======================== SETUP ========================= */

void setup() {
  // OUTPUTS
  // Relay
  pinMode(PIN_RelayTransistor, OUTPUT);
  // Buzzer
  pinMode(PIN_Buzzer, OUTPUT);
  // LCD display
  lcd.begin(LCD_Width, LCD_Height);
  lcd.setBacklight(LCD_Brightnes);

  // INPUTS
  // Touch buttons
  // ...
  // Scroll wheel
  // ...
  // Temp sensors
  Wire.begin();
  Wire.beginTransmission(LCD_Address);
  if (Wire.endTransmission() != 0) {
    // LCD-error, beep sound?
  }

} 


/* ======================== LOOP ========================= */

void loop() {
  if (readTemps) updateTempReadings();
  updatePID(PIDEnabled);
  getButtonStates();
  switch (mode) {   // Mode {MODE_check, MODE_start, MODE_setTemp, MODE_setTime, MODE_warmUp, MODE_ready, MODE_cook, MODE_editParams, MODE_manual, MODE_error, MODE_stop};
    case MODE_check:         // System check
      systemCheck();
      break;
    case MODE_start:         // Start menu ("Automatic", "Manual" or "Edit parameters")
      startupMenu();
      break;
    case MODE_setTemp:         // Set water temperature aim
      setTempMenu();
      break;
    case MODE_setTime:         // Set time
      setTimeMenu();
      break;
    case MODE_warmUp:         // Heat to water temperature aim
      warmUp();
      break;
    case MODE_ready:         // Buzzes and waits to start timer
      readyToCook();
      break;
    case MODE_cook:         // Keeps temp and counts down
      cook();
      break;
    case MODE_editParams:        // Edit PID and other parameters
      editParams();
      break;
    case MODE_manual:       // Error (check lastError)
      manualMode();
      break;
    case MODE_error:       // Error (check lastError)
      errorMsg();
      break;
    default:
      stopAll();
      break;
  }
}



/* ======================== MODES ========================= */
// ********** SYSTEM CHECK - Mode 0 **************
//   +----------------+
//   |Starting (v 0.1)|
//   | Please wait... |
//   +----------------+
void systemCheck() {
  if (lastMode != MODE_check) {
    lastError = 3;
    mode = MODE_error;
    return;
  }
  lcd.setCursor(0, 0);
  lcd.print("Starting (");
  lcd.print(SW_Version);
  lcd.print(")");
  lcd.setCursor(0, 1);
  lcd.print(" Please wait...");

  GetParams();
  thePID.SetOutputLimits(0, PIDParams[7]);
  thePID.SetSampleTime(PIDParams[8]);

  // Check temperature sensors
  for (byte a = 0; a < 2; a++) {
    tempSensors.reset();
    tempSensors.select(tempSensorAddress[a]);
    tempSensors.write(0x44);
  }
  delay(1000);
  int16_t raw;
  byte datas[2][12];
  for (byte a = 0; a < 2; a++) {
    tempSensors.reset();
    tempSensors.select(tempSensorAddress[a]);
    tempSensors.write(0xBE);
    for (byte i = 0; i < 9; i++) {
      datas[a][i] = tempSensors.read();
    }
    raw = (datas[a][1] << 8) | datas[a][0];
    if (raw < 80) { // ((raw < 80) || (raw > 1600)) {
      lastError = 1;
      break;
    }
  }
  if (lastError != 0) {
    mode = MODE_error;
    return;
  }
  // Check buttons
  getButtonStates();
  if (currentOKState || currentCancelState) {
    // If any of the buttons are pressed at start up
    lastError = 2;
    mode = MODE_error;
    return;
  }
  lastMode = MODE_check;
  mode = MODE_start;
}

// ********** START MENU - Mode 5 **************
//   +----------------+
//   |Operating mode: |
//   |Edit parameters |
//   +----------------+
void startupMenu() {
  if (lastMode != mode) {
    newMode(false, false, true, 1);
    lcd.print("Operating mode:");
    lcd.setCursor(0, 1);
    lcd.print("Automatic");
  }

  if (val != oldVal) {
    lcd.setCursor(0, 1);
    if (val < 0) val = 2;
    if (val > 2) val = 0;
    oldVal = val;
    lcd.print(operatingModeName[byte(oldVal)]);
  }

  if (currentOKState) {
    handleButton(BUTTON_OK);
    if (oldVal == 0) {        // Automatic mode, set temp
      mode = MODE_setTemp;
    } else if (oldVal == 1) { // Manual mode
      mode = MODE_manual;
    } else if (oldVal == 2) { // Edit parameters
      mode = MODE_editParams;
    }
  }
}


// ********** SET TEMP MENU - Mode 10 **************
//   +----------------+
//   |Set temperature:|
//   |     60.5°C     |
//   +----------------+
void setTempMenu() {
  if (lastMode != mode) {
    newMode(false, false, true, TEMP_Increment);
    val = tempWaterAim;
    oldVal = val;
    lcd.print("Set temperature:");
    lcd.setCursor(5, 1);
    printTemp(val);
  }

  if (val != oldVal) {
    lcd.setCursor(5, 1);
    if (val < TEMP_MinWaterTemp) val = TEMP_MinWaterTemp;
    if (val > TEMP_MaxWaterTemp) val = TEMP_MaxWaterTemp;
    oldVal = val;
    printTemp(oldVal);
  }

  if (currentOKState) {
    handleButton(BUTTON_OK);
    tempWaterAim = oldVal;
    mode = MODE_setTime;
    return;
  }
  if (currentCancelState) {
    handleButton(BUTTON_Cancel);
    mode = MODE_start;
    return;
  }

}

// ********** SET TIME MENU - Mode 20 **************
//   +----------------+
//   |   Set time:    |
//   |     1h 30m     |
//   +----------------+
void setTimeMenu() {
  if (lastMode != mode) {
    newMode(false, false, true, TIME_Increment);
    val = cookingTimeMinutes; // val is in minutes
    oldVal = val;
    lcd.setCursor(3, 0);
    lcd.print("Set time:");
    lcd.setCursor(5, 1);
    printTime(val, false);
  }

  if (val != oldVal) {
    lcd.setCursor(5, 1);
    if (val < (TIME_MinTime)) val = TIME_MinTime;
    if (val > (TIME_MaxTime)) val = TIME_MaxTime;
    oldVal = val;
    printTime(oldVal, false);
  }

  if (currentOKState) {
    handleButton(BUTTON_OK);
    cookingTimeMinutes = oldVal; // cookingTimeMinutes is in seconds
    mode = MODE_warmUp;
    return;
  }
  if (currentCancelState) {
    handleButton(BUTTON_Cancel);
    mode = MODE_setTemp;
    return;
  }
}

// ********** WARMING UP - Mode 30 **************
//   +----------------+
//   | Warming up...  |
//   |47.3° -> (60.5°)|
//   +----------------+
void warmUp() {
  if (lastMode != mode) {
    newMode(true, true, false, 0);
    tempWaterStart = tempWaterCurrent;
    lcd.print(" Warming up...");
  }
#ifdef DEBUG
  lcd.setCursor(0, 0);
  lcd.print(PIDRelayOnTime, 2);
#endif

  lcd.setCursor(0, 1);
  lcd.print(tempWaterCurrent, 1);
  lcd.print((char)223);
  lcd.print(" -> (");
  lcd.print(tempWaterAim, 1);
  lcd.print((char)223);
  lcd.print(")");

  // Check to time out if water temp not increases...
  /*
    if ((modeTimer != 0) && (millis() > modeTimer)) {
    if (tempWaterCurrent <= tempWaterStart){
      lastError = 4;
      mode = MODE_error;
      return;
    } else {  // Water temp has risen during the TIME_CheckWaterTempTimer period
      modeTimer = 0;
    }
    }
  */
  if (tempWaterCurrent >= tempWaterAim) {
    mode = MODE_ready;
    return;
  }
  if (currentCancelState) {
    handleButton(BUTTON_Cancel);
    updatePID(false);
    readTemps = false; // Needed...?
    mode = MODE_start;   // Should we have a "really cancel?" mode?
    return;
  }
}

// ********** READY TO COOK - Mode 35 **************
//   +----------------+
//   | Water is warm! |
//   |  OK to start?  |
//   +----------------+
void readyToCook() {
  if (lastMode != mode) {
    newMode(true, true, false, 0);
    lcd.print(" Water is warm!");
    lcd.setCursor(2, 1);
    lcd.print("OK to start?");
    buzzerTimer = 3;
  }
  if ((millis() >= modeTimer + 200) && (buzzerTimer > 0)) {
    doBuzzer();
    buzzerTimer--;
    modeTimer = millis() + 200;
  }

  if (currentOKState) {
    handleButton(BUTTON_OK);
    mode = MODE_cook;
    return;
  }
  if (currentCancelState) {
    handleButton(BUTTON_Cancel);
    updatePID(false);
    readTemps = false;
    mode = MODE_start; // Should we have a "really cancel?" mode?
    return;
  }
}

// ********** MAINTAIN HEAT FOR TIMER - Mode 40 **************
//   +----------------+
//   |1h 40m <- 2h 10m|
//   |58.7°C -> 60.5°C|
//   +----------------+
void cook() {
  if (lastMode != mode) {
    newMode(true, true, false, 0);
    lcd.setCursor(6, 0);
    lcd.print(" <- ");
    printTime(cookingTimeMinutes, false);
    lcd.setCursor(6, 1);
    lcd.print(" -> ");
    printTemp(tempWaterAim);
    //    lcd.print(tempWaterAim, 1);
    //    lcd.print((char)223);
    //    lcd.print("C");
    cookingEndTime = (millis() / 1000 + (cookingTimeMinutes * 60)); // In seconds
    timeRemaining = cookingTimeMinutes / 60; // In seconds
  }

  nowRemainingSeconds = (cookingEndTime - (millis() / 1000)); // nowRemainingSeconds in seconds
  if ((nowRemainingSeconds >= 0) && (nowRemainingSeconds != timeRemaining)) {
    timeRemaining = nowRemainingSeconds;
    lcd.setCursor(0, 0);
    printTime(timeRemaining / 60, false);
  }
# ifdef DEBUG
  lcd.setCursor(0, 0);
  lcd.print(PIDRelayOnTime, 2);
# endif

  lcd.setCursor(0, 1);
  printTemp(tempWaterCurrent);

  if (nowRemainingSeconds <= 0) {
    if (modeTimer == 0) {
      modeTimer = millis() + 200;
    } else if (millis() >= modeTimer) {
      doBuzzer();
      modeTimer = millis() + 200;
    }
  }

  if ((currentCancelState) || (currentOKState)) {
    handleButton(BUTTON_Cancel);
    handleButton(BUTTON_OK);
    updatePID(false);
    readTemps = false;
    mode = MODE_start;
    return;
  }
}

// ********** EDIT PARAMETERS - Mode 100 **************
//   +----------------+
//   |Param name:     |
//   |20.20           |
//   +----------------+
void editParams() {
  /*
     parameterNr can be changed to modeTimer
  */
  if (lastMode != mode) {
    newMode(false, false, false, 0);
    lastMode = mode;
    parameterNr = 0;
    paramFirstEdit = true;
  }
  if (parameterNr >= sizeof(PIDParams)) {
    mode = MODE_start;
    return;
  }
  if (paramFirstEdit) {
    lcd.clear();
    byte swVal;
    if (PIDParamsDecimals[parameterNr] == 0) {
      swVal = 1;
    } else {
      swVal = 0.1;
    }
    enableScrollWheel(true, swVal);
    lcd.print(PIDParamsName[parameterNr]);
    lcd.setCursor(0, 1);
    val = PIDParams[parameterNr];
    oldVal = val;
    lcd.print(val, PIDParamsDecimals[parameterNr]);
    paramFirstEdit = false;
  }
  if (val != oldVal) {
    oldVal = val;
    lcd.setCursor(0, 1);
    if (val < (PIDParamsMin[parameterNr])) val = PIDParamsMin[parameterNr];
    if (val > (PIDParamsMax[parameterNr])) val = PIDParamsMax[parameterNr];
    lcd.print(val, PIDParamsDecimals[parameterNr]);
    lcd.print("     ");
  }

  if (currentOKState) {
    handleButton(BUTTON_OK);
    PutParam(val, parameterNr);
    PIDParams[parameterNr] = val;
    parameterNr++;
    paramFirstEdit = true;
  }
}


// ********** MANUAL MODE - Mode 200 **************
//   +----------------+
//   |Water:    57.0°C|
//   |Power:      22.5|
//   +----------------+
void manualMode() {
  if (lastMode != mode) {
    newMode(true, true, true, 0.1);
    lcd.print("Water:    ");
    printTemp(tempWaterCurrent);
    modeTimer = now + TEMP_TempReadingFrequency;
    lcd.setCursor(0, 1);
    lcd.print("Power:");
    PIDWindowStartTime = 0;
  }

  // TODO: Change this to a recoverable state after the temp has dropped
  if ((tempWaterCurrent >= TEMP_MaxWaterTemp) || (tempHeaterCurrent >= TEMP_MaxHeaterTemp)) {
    digitalWrite(PIN_RelayTransistor, LOW);
    heaterOn = false;
    lastError = (tempWaterCurrent >= TEMP_MaxWaterTemp) ? 5 : 6;
    mode = MODE_error;
    return;
  }

  unsigned long now = millis();
  if (val != oldVal) {
    oldVal = val;
    lcd.setCursor(12, 1);
    if (oldVal < 10) {
      lcd.print(" ");
    }
    lcd.print(oldVal, 1);
    PIDWindowStartTime = now; // Create new time window
  }

  if (now > (PIDWindowStartTime + PID_WindowSize)) { // Are we outside the time window?
    PIDWindowStartTime = now;
  } else {  // We're inside the time window
    if (now > (PIDWindowStartTime + (oldVal * 1000))) { // Are we outisde the "heater on" window?
      digitalWrite(PIN_RelayTransistor, LOW);
      heaterOn = false;
    } else {
      digitalWrite(PIN_RelayTransistor, HIGH);
      heaterOn = true;
    }
  }

  if (now >= modeTimer) { // Time to update the temp reading on the lcd
    lcd.setCursor(10, 0);
    printTemp(tempWaterCurrent);
    modeTimer = now + TEMP_TempReadingFrequency;
  }

  if (currentCancelState) {
    handleButton(BUTTON_Cancel);
    mode = MODE_start;
    return;
  }

  // TODO: OK should go in to "maintain current temp"-mode
  if (currentOKState) {
    handleButton(BUTTON_OK);
    mode = MODE_start;
    return;
  }
}


// ********** ERROR MESSAGE - Mode 255 **************
//   +----------------+
//   |ERROR 1         |
//   |Temp sensors!   |
//   +----------------+
void errorMsg() {
  stopAll();
  if (lastMode == MODE_error) return;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("ERROR ");
  lcd.print(lastError, DEC);
  lcd.setCursor(0, 1);
  switch (lastError) {
    case 1:         // No temp sensor or erronuos values read
      lcd.print("Temp sensors!");
      break;
    case 2:         // Buttons pressed or erronuos values read
      lcd.print("Clean buttons?");
      break;
    case 3:       // Error in state machine
      lcd.print("Internal error");
      break;
    case 4:
      lcd.print("Water temp error");
      break;
    default:
      lcd.print("Unknown error");
      break;
  }
  lastMode = MODE_error;
}

// ********** ERROR MESSAGE - Mode unknown and default **************
void stopAll() {
  updatePID(false);
  digitalWrite(PIN_RelayTransistor, LOW);
  heaterOn = false;
  readTemps = false;
  mode = MODE_error;
  lastError = 255;
}

/* ======================== MISC. ========================= */

void newMode(bool readT, bool updateP, bool enableSW, float swInc) {
  lastMode = mode;
  readTemps = readT;
  modeTimer = 0;
  if (updateP) PIDWindowStartTime = millis();
  updatePID(updateP);
  if (enableSW) {
    val = 0;
    oldVal = val;
  }
  lcd.clear();
  enableScrollWheel(enableSW, swInc);
}

/* ======================== PID ========================= */

void updatePID(bool enablePID) {
  if ((PIDEnabled) && (!enablePID)) {
    thePID.SetMode(MANUAL);
  } else if ((!PIDEnabled) && (enablePID)) {
    thePID.SetMode(AUTOMATIC);
  }
  PIDEnabled = enablePID;
  if (!PIDEnabled) {
    digitalWrite(PIN_RelayTransistor, LOW);
    heaterOn = false;
    return;
  }
  thePID.Compute();
  unsigned long now = millis();
  if (now >= (PIDWindowStartTime + PIDWindowSize)) { // Window's ending, time to create new
    PIDWindowStartTime = now;
    if ((tempWaterAim - tempWaterCurrent) >= PID_Agg_Threshold) {  // We are "far" from the target temp, use aggressive PID-values
      thePID.SetTunings(PIDParams[3], PIDParams[4], PIDParams[5]);
    } else {
      thePID.SetTunings(PIDParams[0], PIDParams[1], PIDParams[2]);
    }
  } else {  // We're inside the PID window. Check if relay should be on or off
    if (now >= (PIDWindowStartTime + PIDRelayOnTime)) { // The on-time window has passed...
      digitalWrite(PIN_RelayTransistor, LOW);
      heaterOn = false;
    } else {  // We're in the on-time window
      digitalWrite(PIN_RelayTransistor, HIGH);
      heaterOn = true;
    }
  }
}

/* ======================== LCD DISPLAY ========================= */

void printTemp(float tmp) {
  lcd.print(tmp, 1);
  lcd.print((char)223);
  lcd.print("C");
}

// printTime takes a number of minutes as argument
//  tm    Nr of minutes that will be displayed as hours and minutes
//  abb   True if the letters 'h' and 'm' are to be dropped from the print out
void printTime(float tm, bool abb) {
  byte hr = (int)tm / 60;
  byte mn = (int)tm % 60;
  char txt[8];
  if (abb) {
    sprintf(txt, "%01d:%02d", hr, mn);
  } else {
    sprintf(txt, "%01dh %02dm", hr, mn);
  }
  lcd.print(txt);
}


void doBuzzer() {
  for (byte i = 0; i < 4; i++) {
    tone(PIN_Buzzer, BUZZER_Frequency, BUZZER_Duration);
    delay(BUZZER_Duration + BUZZER_Delay);
  }
}

/* ======================== EEPROM ========================= */


void GetParams() {
  // float         PIDParams[9] = {PID_Kp, PID_Ki, PID_Kd, PID_Kp_Agg, PID_Ki_Agg, PID_Kd_Agg, PID_Agg_Threshold, PID_SampleTime, PID_WindowSize};
  float tmpVal;
  for (byte nr = 0; nr < sizeof(PIDParams); nr++) {
    val = EEPROM.get(nr * 4, PIDParams[nr]);
  }
}

void PutParam(float val, byte nr) {
  if (val != PIDParams[nr]) {
    EEPROM.put(nr * 4, val);
  }
  /*
    for (int nr = 0; nr < sizeof(PIDParams); nr++) {
    EEPROM.put(nr * 4, PIDParams[nr]);
    }
  */
}

/* ======================== SCROLL WHEEL ========================= */

void enableScrollWheel(bool on, float swi) {
  swIncrement = swi;
  if (on) {
    if (scrollWheelEnabled) return;
    // Set interrupts on Scroll Wheel sensors A & B
    attachInterrupt(digitalPinToInterrupt(2), triggerSWSA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(3), triggerSWSB, CHANGE);
  } else {
    if (!scrollWheelEnabled) return;
    detachInterrupt(digitalPinToInterrupt(2));
    detachInterrupt(digitalPinToInterrupt(3));
  }
  scrollWheelEnabled = on;
}

// Scroll Wheel sensor interrupts
void triggerSWSA() {
  tmpA = digitalRead(PIN_SWSensorA);
  if (tmpA == 1) {
    if (oldB == 0) {
      val += swIncrement;
    } else {
      val -= swIncrement;
    }
  } else {
    if (oldB == 0) {
      val -= swIncrement;
    } else {
      val += swIncrement;
    }
  }
  oldA = tmpA;
}


void triggerSWSB() {
  tmpB = digitalRead(PIN_SWSensorB);
  if (tmpB == 1) {
    if (oldA == 0) {
      val -= swIncrement;
    } else {
      val += swIncrement;
    }
  } else {
    if (oldA == 0) {
      val += swIncrement;
    } else {
      val -= swIncrement;
    }
  }
  oldB = tmpB;
}

/* ======================== BUTTONS ========================= */

void handleButton(bool okBtn) {
  if (okBtn) {
    OKForcedDown = millis() + BUTTON_DebounceTime;
  } else {
    CancelForcedDown = millis() + BUTTON_DebounceTime;
  }
}

// Continuous button state
void getButtonStates() {
  bool tmpState = (btnOK.capacitiveSensor(BUTTON_ReadTime) > BUTTON_CapThreshold);
  if (!currentOKState) {
    if ((tmpState) && (OKForcedDown < millis())) {
      currentOKState = true;
      OKClickTime = millis();
    }
  } else {
    if (OKForcedDown >= millis()) {
      currentOKState = false;
      return;
    }
    if ((millis() - OKClickTime) < BUTTON_DebounceTime) return;
    if (tmpState) {
      OKClickTime = millis();
    } else {
      currentOKState = false;
    }
  }
  tmpState = (btnCancel.capacitiveSensor(BUTTON_ReadTime) > BUTTON_CapThreshold);
  if (!currentCancelState) {
    if ((tmpState) && (CancelForcedDown < millis())) {
      currentCancelState = true;
      CancelClickTime = millis();
    }
  } else {
    if (CancelForcedDown >= millis()) {
      currentCancelState = false;
      return;
    }
    if ((millis() - CancelClickTime) < BUTTON_DebounceTime) return;
    if (tmpState) {
      CancelClickTime = millis();
    } else {
      currentCancelState = false;
    }
  }
}


/* ======================== TEMP SENSORS ========================= */

// Continuous temperature reading
void updateTempReadings() {
  if ((mode == MODE_check) || (mode == MODE_error)) return;
  if (tempReadingTime == 0) {
    for (byte a = 0; a < 2; a++) {
      tempSensors.reset();
      tempSensors.select(tempSensorAddress[a]);
      tempSensors.write(0x44);
    }
    tempReadingTime = millis() + TEMP_TempReadingFrequency;
  } else if (millis() > tempReadingTime) {
    int16_t raw;
    byte datas[2][12];
    float celsius[2];
    for (byte a = 0; a < 2; a++) {
      tempSensors.reset();
      tempSensors.select(tempSensorAddress[a]);
      tempSensors.write(0xBE);
      for (byte i = 0; i < 9; i++) {
        datas[a][i] = tempSensors.read();
      }
      raw = (datas[a][1] << 8) | datas[a][0];
      celsius[a] = (float)raw / 16.0;
    }
    tempHeaterCurrent = celsius[0];
    tempWaterCurrent = celsius[1];
    tempReadingTime = 0;
  }
}
