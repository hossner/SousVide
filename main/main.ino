// https://github.com/adafruit/Sous_Viduino/blob/master/Sous_Viduino.ino
// https://playground.arduino.cc/Code/PIDAutotuneLibrary/
// http://brettbeauregard.com/blog/2012/01/arduino-pid-autotune-library/#more-1893
#include <Wire.h>
#include <LiquidCrystal_PCF8574.h>
#include <CapacitiveSensor.h>
#include <OneWire.h>
#include <PID_v1.h>
//#include <AutoPID.h>

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

#define SW_Version              "v 0.1" // Max 5 chars

#define PIN_SWSensorA           2
#define PIN_SWSensorB           3
#define PIN_BtnCommon           4
#define PIN_BtnOK               5
#define PIN_BtnCancel           6
#define PIN_Buzzer              9
#define PIN_RelayTransistor     12
#define PIN_TempSensors         13

#define PID_Kp              0.5   // Find this by auto tune!
#define PID_Ki              0.1
#define PID_Kd              0.0
#define PID_Kp_Agg          4
#define PID_Ki_Agg          0.0
#define PID_Kd_Agg          0.0
#define PID_Agg_Threshold   10.0 // Degrees below setpoint where we go from aggressive to normal PID values
#define PID_WindowSize      30000

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

#define TIME_MaxTime    599 // 599 min in seconds
#define TIME_MinTime    10   // 10 min in seconds
#define TIME_Increment  1.0 // in minutes
#define TIME_CheckWaterTempTimer 300000 // Time in millisec after which the water temp must have risen

/* ======================== GLOBAL VARIABLES ========================= */


// =============== Scroll Wheel sensor ===============
volatile unsigned int tmpA = 0;
volatile unsigned int tmpB = 0;
volatile unsigned int oldA = 0;
volatile unsigned int oldB = 0;
volatile float        val = 0.0;
volatile float        oldVal = 0.0;
volatile float        swIncrement = 0.1;

// =============== Mode handling ===============
byte            mode = 0;
byte            lastMode = 0;
unsigned long   tempReadingTime = 0;
byte            lastError = 0;

// =============== Buttons ===============
unsigned long OKClickTime = 0;
unsigned long CancelClickTime = 0;
unsigned long OKForcedDown = 0;
unsigned long CancelForcedDown = 0;
bool          currentOKState = false;
bool          currentCancelState = false;

// =============== Temp sensors ===============
byte  tempSensorAddress[2][8] = {{0x28, 0xD5, 0x77, 0xCA, 0x06, 0x00, 0x00, 0xFE}, {0x28, 0xFF, 0xCA, 0x7A, 0x86, 0x16, 0x05, 0xAE}};
float tempHeaterCurrent = 0.00;
float tempWaterCurrent =  0.00;
float tempWaterAim =      60.00;
float tempWaterStart = 0.00;
bool  readTemps = false;

// =============== PID ===============
int           PIDWindowSize = PID_WindowSize;
unsigned long PIDWindowStartTime = 0;
float         PIDRelayOnTime = 0.0;
bool          PIDEnabled = false;

// =============== Misc. ===============
long            cookingTimeMinutes =       90;  // in minutes
long            timeRemaining = 0;     // In seconds
unsigned long   cookingEndTime = 0;  // In milliseconds
byte            buzzerTimer = 0;
unsigned long   modeTimer = 0;
long            nowRemainingSeconds = 0; // In seconds
bool            heaterOn = false;

LiquidCrystal_PCF8574   lcd(LCD_Address);
OneWire                 tempSensors(PIN_TempSensors);
PID                     thePID((double *)&tempWaterCurrent, (double *)&PIDRelayOnTime, (double *)&tempWaterAim, (double)PID_Kp, (double)PID_Ki, (double)PID_Kd, DIRECT);
CapacitiveSensor        btnOK     = CapacitiveSensor(PIN_BtnCommon, PIN_BtnOK);
CapacitiveSensor        btnCancel = CapacitiveSensor(PIN_BtnCommon, PIN_BtnCancel);

/* ======================== SETUP ========================= */

void setup() {
  Wire.begin();
  Wire.beginTransmission(LCD_Address);
  if (Wire.endTransmission() != 0) {
    // LCD-error, beep sound?
  }

  lcd.begin(LCD_Width, LCD_Width);
  lcd.setBacklight(LCD_Brightnes);

  pinMode(PIN_RelayTransistor, OUTPUT);
  pinMode(PIN_Buzzer, OUTPUT);

}


/* ======================== LOOP ========================= */

void loop() {
  if (readTemps) updateTempReadings();
  if (PIDEnabled) updatePID();
  getButtonStates();
  switch (mode) {
    case 0:         // System check
      systemCheck();
      break;
    case 10:         // Set water temperature aim
      setTempMenu();
      break;
    case 20:         // Set time
      setTimeMenu();
      break;
    case 30:         // Heat to water temperature aim
      heatUp();
      break;
    case 35:         // Buzzes and waits to start timer
      readyToCook();
      break;
    case 40:         // Keeps temp and counts down
      cook();
      break;
    case 255:       // Error (check lastError)
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
  if (lastMode != 0) {
    lastError = 3;
    mode = 255;
    return;
  }
  lcd.setCursor(0, 0);
  lcd.print("Starting (");
  lcd.print(SW_Version);
  lcd.print(")");
  lcd.setCursor(0, 1);
  lcd.print(" Please wait...");
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
    //      if (a = 0){ // Set the initial water temp
    //        tempWaterCurrent = (float)raw / 16.0;
    //      }
  }
  if (lastError != 0) {
    mode = 255;
    return;
  }
  // Check buttons
  getButtonStates();
  if (currentOKState || currentCancelState) {
    lastError = 2;
    mode = 255;
    return;
  }
  lastMode = 0;
  mode = 10;
}

// ********** START MENU - Mode 10 **************
//   +----------------+
//   |Set temperature:|
//   |     60.5°C     |
//   +----------------+
void setTempMenu() {
  if (lastMode != 10) {
    enableScrollWheel(true, TEMP_Increment);
    readTemps = false;
    PIDEnabled = false;
    val = tempWaterAim;
    oldVal = val;
    lastMode = 10;
    lcd.clear();
    lcd.print("Set temperature:");
    lcd.setCursor(5, 1);
    printTemp(val);
  }

  if (val != oldVal) {
    oldVal = val;
    if (val < TEMP_MinWaterTemp) val = TEMP_MinWaterTemp;
    if (val > TEMP_MaxWaterTemp) val = TEMP_MaxWaterTemp;
    lcd.setCursor(5, 1);
    printTemp(val);
  }

  if (currentOKState) {
    handleButton(BUTTON_OK);
    tempWaterAim = val;
    enableScrollWheel(false, 0);
    mode = 20;
  }
}

// ********** SET TIME MENU - Mode 20 **************
//   +----------------+
//   |   Set time:    |
//   |     1h 30m     |
//   +----------------+
void setTimeMenu() {
  if (lastMode != 20) {
    enableScrollWheel(true, TIME_Increment);
    val = cookingTimeMinutes; // val is in minutes
    oldVal = val;
    lastMode = 20;
    readTemps = false;
    PIDEnabled = false;
    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print("Set time:");
    lcd.setCursor(5, 1);
    printTime(val, false);
  }

  if (val != oldVal) {
    oldVal = val;
    lcd.setCursor(5, 1);
    if (val < (TIME_MinTime)) val = TIME_MinTime;
    if (val > (TIME_MaxTime)) val = TIME_MaxTime;
    printTime(val, false);
  }

  if (currentCancelState) {
    handleButton(BUTTON_Cancel);
    enableScrollWheel(false, 0);
    mode = 10;
    return;
  }

  if (currentOKState) {
    handleButton(BUTTON_OK);
    cookingTimeMinutes = val; // cookingTimeMinutes is in seconds
    enableScrollWheel(false, 0);
    mode = 30;
    return;
  }

}

// ********** WARMING UP - Mode 30 **************
//   +----------------+
//   | Warming up...  |
//   |47.3° -> (60.5°)|
//   +----------------+
void heatUp() {
  if (lastMode != 30) {
    readTemps = true;
    PIDEnabled = true;
    lastMode = 30;
    tempWaterStart = tempWaterCurrent;
    modeTimer = 0; //millis() + TIME_CheckWaterTempTimer;
    lcd.clear();
    lcd.print(" Warming up...");
  }
  lcd.setCursor(0, 1);
  lcd.print(tempWaterCurrent, 1);
  lcd.print((char)223);
  lcd.print(" -> (");
  lcd.print(tempWaterAim, 1);
  lcd.print((char)223);
  lcd.print(")");

  // Check to time out if water temp not increases...
  if ((modeTimer != 0) && (millis() > modeTimer)) {
    if (tempWaterCurrent <= tempWaterStart){
      lastError = 4;
      mode = 255;
      return;
    } else {  // Water temp has risen during the TIME_CheckWaterTempTimer period
      modeTimer = 0;
    }
  }

  if (currentCancelState) {
    handleButton(BUTTON_Cancel);
    digitalWrite(PIN_RelayTransistor, LOW);
    heaterOn = false;
    readTemps = false; // Needed...?
    PIDEnabled = false;
    mode = 10;
    return;
  }
}

// ********** READY TO COOK - Mode 35 **************
//   +----------------+
//   | Water is warm! |
//   |  OK to start?  |
//   +----------------+
void readyToCook() {
  if (lastMode != 35) {
    readTemps = true;
    PIDEnabled = true;
    lastMode = 35;
    lcd.clear();
    lcd.print(" Water is warm!");
    lcd.setCursor(2, 1);
    lcd.print("OK to start?");
    buzzerTimer = 10;
  }
  if (buzzerTimer > 0) {
    doBuzzer();
    buzzerTimer--;
    delay(200);
  }

  if (currentOKState) {
    handleButton(BUTTON_OK);
    mode = 40;
    return;
  }

  if (currentCancelState) {
    handleButton(BUTTON_Cancel);
    digitalWrite(PIN_RelayTransistor, LOW);
    heaterOn = false;
    readTemps = false; // Needed...?
    mode = 10; // Should we have a "really cancel?" mode?
    return;
  }
}

// ********** MAINTAIN HEAT FOR TIMER - Mode 40 **************
//   +----------------+
//   |1h 40m <- 2h 10m|
//   |58.7°C -> 60.5°C|
//   +----------------+
void cook() {
  if (lastMode != 40) {
    readTemps = true;
    PIDEnabled = true;
    lastMode = 40;
    lcd.clear();
    lcd.setCursor(6, 0);
    lcd.print(" <- ");
    printTime(cookingTimeMinutes, false);
    lcd.setCursor(6, 1);
    lcd.print(" -> ");
    lcd.print(tempWaterAim, 1);
    lcd.print((char)223);
    lcd.print("C");
    cookingEndTime = (millis()/1000 + (cookingTimeMinutes * 60)); // In seconds
    timeRemaining = cookingTimeMinutes / 60; // In seconds
  }

  nowRemainingSeconds = (cookingEndTime - (millis() / 1000)); // nowRemainingSeconds in seconds
  if ((nowRemainingSeconds >= 0) && (nowRemainingSeconds != timeRemaining)) {
    timeRemaining = nowRemainingSeconds;
    lcd.setCursor(0, 0);
    printTime(timeRemaining*60, false);
  }
  lcd.setCursor(0, 1);
  printTemp(tempWaterCurrent);

  if (nowRemainingSeconds <= 0) {
    doBuzzer();
    delay(200);
    return;
  }

  if (currentCancelState) {
    handleButton(BUTTON_Cancel);
    digitalWrite(PIN_RelayTransistor, LOW);
    readTemps = false; // Needed...?
    mode = 10;
    return;
  }

}

// ********** ERROR MESSAGE - Mode 255 **************
//   +----------------+
//   |ERROR 1         |
//   |Temp sensors!   |
//   +----------------+
void errorMsg() {
  digitalWrite(PIN_RelayTransistor, LOW);
  heaterOn = false;
  readTemps = false;
  PIDEnabled = false;
  if (lastMode == 255) return;
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
  lastMode = 255;
}

// ********** ERROR MESSAGE - Mode unknown and default **************
void stopAll() {
  PIDEnabled = false;
  mode = 255;
  lastError = 255;
}


/*
  int PIDWindowSize = PID_WindowSize;
  unsigned long PIDWindowStartTime = 0;
  float PIDRelayOnTime = 0.0;
*/
void updatePID() {
  thePID.Compute();
  unsigned long now = millis();
  if (now >= (PIDWindowStartTime + PIDWindowSize)) { // Window's ending, time to create new
    PIDWindowStartTime = now;
    if (tempWaterCurrent > (tempWaterAim - PID_Agg_Threshold)) { // We are "close" to the target temp, use less aggressive PID-values
      thePID.SetTunings(PID_Kp, PID_Ki, PID_Kd);
    } else { // We're far from the target temp, use aggressive PID-values
      thePID.SetTunings(PID_Kp_Agg, PID_Ki_Agg, PID_Kd_Agg);
    }
  } else {  // We're inside the PID window. Check if relay should be on or off
    if (now > (PIDWindowStartTime + PIDRelayOnTime)) { // The on-time window has passed...
      digitalWrite(PIN_RelayTransistor, LOW);
      heaterOn = false;
    } else {  // We're in the on-time window
      digitalWrite(PIN_RelayTransistor, HIGH);
      heaterOn = true;
    }
  }
}

bool handlePid_old() {
  // Replace this with PID...
  if (tempHeaterCurrent >= TEMP_MaxHeaterTemp) {
    digitalWrite(PIN_RelayTransistor, LOW);
    return false;
  }
  if (tempWaterCurrent >= (tempWaterAim + TEMP_MaxWaterTempDiff)) {
    digitalWrite(PIN_RelayTransistor, LOW);
    return false;
  } else if (tempWaterCurrent <= (tempWaterAim - TEMP_MaxWaterTempDiff)) {
    digitalWrite(PIN_RelayTransistor, HIGH);
    return true;
  }
}

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


/* ======================== SCROLL WHEEL ========================= */

void enableScrollWheel(bool on, float swi) {
  swIncrement = swi;
  if (on) {
    // Set interrupts on Scroll Wheel sensors A & B
    attachInterrupt(digitalPinToInterrupt(2), triggerSWSA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(3), triggerSWSB, CHANGE);
  } else {
    detachInterrupt(digitalPinToInterrupt(2));
    detachInterrupt(digitalPinToInterrupt(3));
  }
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
  if ((mode == 0) || (mode == 255)) return;
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

/* ======================== SCROLL WHEEL INTERRUPTS ========================= */

// Scroll Wheel sensor triggers
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



/*
  void triggerSWSA() {
  tmpA = digitalRead(PIN_SWSensorA);
  if (tmpA == 1){
    if (oldB == 0){
      val++;
    } else {
      val--;
    }
  } else {
    if (oldB == 0) {
      val--;
    } else {
      val++;
    }
  }
  oldA = tmpA;
  }


  void triggerSWSB() {
  tmpB = digitalRead(PIN_SWSensorB);
  if (tmpB == 1){
    if (oldA == 0){
      val--;
    } else {
      val++;
    }
  } else {
    if (oldA == 0) {
      val++;
    } else {
      val--;
    }
  }
  oldB = tmpB;
  }
*/
/*
  // lcd.print((char)223); // Degree symbol
  if (val != oldVal){
    lcd.setCursor(0,0);
    lcd.print(val, DEC);
    //Serial.println(val);
    oldVal = val;
  }
  long total1 =  btnOK.capacitiveSensor(BUTTON_ReadTime);
  long total2 =  btnCancel.capacitiveSensor(BUTTON_ReadTime);
  if (total1 > BUTTON_CapThreshold){
    lcd.setCursor(0,1);
    lcd.print("OK");
    digitalWrite(PIN_RelayTransistor, HIGH);
    lcd.setCursor(5,1);
    lcd.print("      ");
  }
  if (total2 > BUTTON_CapThreshold){
    lcd.setCursor(5,1);
    lcd.print("Cancel");
    tone(buzzer,freq, dur);
    digitalWrite(PIN_RelayTransistor, LOW);
    lcd.setCursor(0,1);
    lcd.print("  ");
  }
  updateTempReadings();
*/



/*
  if (isBtnPressed(true)) {
    if (currentOKState){
      if ((millis() - OKClickTime) > BUTTON_ClickTime) {
        currentOKState = false;
        //tone(PIN_Buzzer, BUZZER_Frequency, BUZZER_Duration);
      }
    } else {
      if (OKClickTime == 0){
        OKClickTime = millis();
        currentOKState = true;
      }
    }
  } else {
    if ((millis() - OKClickTime) > BUTTON_ClickTime){
      currentOKState = false;
      OKClickTime = 0;
    } else {
      currentOKState = true;
    }
  }
*/
/*

  bool tmpState = isBtnPressed(true);
  if (tmpState != oldOKState){
    OKClickTime = millis();
  }
  oldOKState = tmpState;
  if ((millis() - OKClickTime) < BUTTON_DebounceTime){
  //Debug
  tone(PIN_Buzzer, BUZZER_Frequency, BUZZER_Duration);
  // End
    return;
  }
  currentOKState = tmpState;
*/
/*
  tmpState = isBtnPressed(false);
  if (tmpState != oldCancelState){
    CancelClickTime = millis();
  }
  oldCancelState = tmpState;
  if ((millis() - CancelClickTime) < BUTTON_DebounceTime) return;
  currentCancelState = tmpState;
*/
/*
  bool tmpState = (btnOK.capacitiveSensor(BUTTON_ReadTime) > BUTTON_CapThreshold);
  if (tmpState != oldOKState){
  OKClickTime = millis();
  }
  oldOKState = tmpState;

  if ((millis() - OKClickTime) < BUTTON_DebounceTime) return;
  currentOKState = tmpState;
*/

/*

  bool isBtnPressed(bool OKbtn){
  if (OKbtn){
    if (btnOK.capacitiveSensor(BUTTON_ReadTime) > BUTTON_CapThreshold) return true;
  } else {
    if (btnCancel.capacitiveSensor(BUTTON_ReadTime) > BUTTON_CapThreshold) return true;
  }
  tone(PIN_Buzzer, 1500, 10);
  return false;
  }

*/
