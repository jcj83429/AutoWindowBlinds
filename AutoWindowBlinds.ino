#include <Wire.h>
#include "SparkFun_VEML6030_Ambient_Light_Sensor.h"

// using snooze breaks USB serial print
// disable snooze for debugging
#define USE_SNOOZE 1

#define MOTOR_SPEED 240
#define ROTATIONS_TO_OPEN 4
#define ROTATION_SENSOR_TIMEOUT 8000

#define ERR_LOW_BATT 1
#define ERR_ROTATION_FAIL 2
#define ERR_SENSOR_INIT 3

#define MOTOR_FORWARD 1
#define MOTOR_REVERSE -1
#define MOTOR_STOP 0

#define MOTOR_DIR_CURTAIN_OPEN MOTOR_REVERSE
#define MOTOR_DIR_CURTAIN_CLOSE MOTOR_FORWARD

// to save power, only turn on HV circuits (>3.3v) when needed
// HV circuits include motor driver and hall sensor
#define PIN_HV_EN_RELAY 6
#define PIN_MOTOR_FORWARD 10
#define PIN_MOTOR_REVERSE 9
#define PIN_HALL_SENSOR_READ 8

// push button for manually toggling open/close
#define PIN_MANUAL_SW 11

#define VEML6030_ADDR 0x48

enum CurtainState {
  CURTAIN_CLOSED = 0, 
  CURTAIN_OPEN = 1,
};

enum LightState {
  DARK = 0,
  LIGHT = 1,
};

////////////////////////////////////////////////////////////////
// GLOBALS
////////////////////////////////////////////////////////////////

#if USE_SNOOZE
#include <Snooze.h>
// Load drivers
SnoozeTimer snoozeTimer;
SnoozeDigital snoozeDigital;
// install drivers to a SnoozeBlock
SnoozeBlock snoozeConfig(snoozeTimer, snoozeDigital);
#endif

// Assume the system powers on with the curtain closed!!!
CurtainState currentCurtainState = CURTAIN_CLOSED;
int currentCurtainPosition = 0;
int openPosition = ROTATIONS_TO_OPEN;
LightState currentLightState = DARK;

// default light and dark threshold
float dbLuxThreshDark = -5;
float dbLuxThreshLight = 5;

// brightness distribution in the last 24 hours.
// each bucket is 1 dbLux.
// every 24 hours, the resistance thresholds are adjusted based on the brightness stat.
#define DBLUX_STAT_BUCKETS 100
// The sensor can sense 0.0036 to 120000 lux (-24.4 to +50.8 dbLux)
#define DBLUX_STAT_BUCKET_OFFSET 25;
int dbLuxStat[DBLUX_STAT_BUCKETS] = {0};
int dbLuxStatSamples = 0;
unsigned long lastThresholdAdjTime = 0;

// The data is noisy below 0 dbLux, so the threshold must be set higher than 0 dbLux
#define DBLUX_MEDIAN_LIMIT 0

SparkFun_Ambient_Light light(VEML6030_ADDR);

float gainTimeSettings[10][2] = {
  {2, 800},   // 0.0036
  {2, 400},   // 0.0072
  {2, 200},   // 0.0144
  {2, 100},   // 0.0288
  {2, 50},    // 0.0576
  {2, 25},    // 0.1152
  {1, 25},    // 0.2304
  {0.25, 50}, // 0.4608
  {0.25, 25}, // 0.9216
  {0.125, 25},// 1.8432
};

const int numGainTimeSettings = 10;
int curGainTimeSetting = 5;

////////////////////////////////////////////////////////////////
// FUNCTIONS
////////////////////////////////////////////////////////////////

void halt(int error){
  setMotorDirection(MOTOR_STOP, 0);
  digitalWrite(PIN_HV_EN_RELAY, LOW);
  
  digitalWrite(LED_BUILTIN, LOW);
  sleepDispatch(500);
  
  while(true){
    for(int i=0; i<error; i++){
      digitalWrite(LED_BUILTIN, HIGH);
      sleepDispatch(500);
      digitalWrite(LED_BUILTIN, LOW);
      sleepDispatch(500);
    }
    sleepDispatch(1000);
  }
}

void setGainTimeSetting(int index){
  curGainTimeSetting = index;
  light.setGain(gainTimeSettings[index][0]);
  light.setIntegTime(gainTimeSettings[index][1]);
}

float readLuxAutoRange(){
  uint16_t rawValue = light._readRegister(AMBIENT_LIGHT_DATA_REG);
  bool powSaveDisabled = false;
  while((rawValue < 100 && curGainTimeSetting > 0) || (rawValue > 10000 && curGainTimeSetting < numGainTimeSettings - 1)){
    if(!powSaveDisabled){
      light.disablePowSave();
      sleepDispatch(4000);
      powSaveDisabled = true;
    }
    int oldIntegrationTime = gainTimeSettings[curGainTimeSetting][1];
    if(rawValue < 100){
      if(rawValue < 50 && curGainTimeSetting >= 2){
        setGainTimeSetting(curGainTimeSetting - 2);
      }else{
        setGainTimeSetting(curGainTimeSetting - 1);
      }
    }else{
      setGainTimeSetting(curGainTimeSetting + 1);
    }
    Serial.printf("adjusting resolution to %d\n", curGainTimeSetting);
    sleepDispatch(oldIntegrationTime);
    sleepDispatch(gainTimeSettings[curGainTimeSetting][1] * 1.2);
    rawValue = light._readRegister(AMBIENT_LIGHT_DATA_REG);
  }
  if(powSaveDisabled){
    light.enablePowSave();
  }
  float lux = light._calculateLux(rawValue);
  if(lux > 1000){
    lux = light._luxCompensation(lux);
  }
  return lux;
}

// Wiring: PIN_MANUAL_SW - push button - GND
// PIN_MANUAL_SW is in INPUT_PULLUP mode
bool manualSwPressed(){
  return !digitalRead(PIN_MANUAL_SW);
}

bool readHallSensor(){
  return digitalRead(PIN_HALL_SENSOR_READ);
}

void setMotorDirection(int direction, int motorSpeed){
  switch(direction){
  case MOTOR_FORWARD:
    analogWrite(PIN_MOTOR_REVERSE, 0);
    analogWrite(PIN_MOTOR_FORWARD, motorSpeed);
    break;
  case MOTOR_REVERSE:
    analogWrite(PIN_MOTOR_FORWARD, 0);
    analogWrite(PIN_MOTOR_REVERSE, motorSpeed);
    break;
  case MOTOR_STOP:
    analogWrite(PIN_MOTOR_FORWARD, 0);
    analogWrite(PIN_MOTOR_REVERSE, 0);
    break;
  }
}

void applyCurrentState(int motorSpeed){
  Serial.println(__FUNCTION__);

  int positionDelta;
  int targetPosition;
  int motorDirection;
  if(currentCurtainState == CURTAIN_OPEN){
    targetPosition = openPosition;
  }else{
    targetPosition = 0;
  }

  if(targetPosition == currentCurtainPosition){
    return;
  }

  Serial.print("  currentCurtainPosition=");
  Serial.println(currentCurtainPosition);
  Serial.print("  targetPosition=");
  Serial.println(targetPosition);
  Serial.print("  positionDelta=");
  Serial.println(positionDelta);

  digitalWrite(PIN_HV_EN_RELAY, HIGH);
  delay(1000);

  if(targetPosition > currentCurtainPosition){
    setMotorDirection(MOTOR_DIR_CURTAIN_OPEN, motorSpeed);
    unsigned long rotationStartTime = millis();
    bool lastHallSensorState = readHallSensor();
    // When opening, go to targetposition+1 then come back to targetPosition
    // This way the motor always stops where it would be when rotating in the close direction.
    while(currentCurtainPosition < targetPosition + 1){
      bool newHallSensorState = readHallSensor();
      unsigned long currentTime = millis();
      digitalWrite(LED_BUILTIN, newHallSensorState);

      if(currentTime - rotationStartTime > ROTATION_SENSOR_TIMEOUT){
        Serial.println("HALT: MOTOR OR ROTATION SENSOR FAILED");
        halt(ERR_ROTATION_FAIL);
      }
      
      if(newHallSensorState == 1 && lastHallSensorState == 0){
        currentCurtainPosition++;
        Serial.print("  currentCurtainPosition=");
        Serial.println(currentCurtainPosition);
        rotationStartTime = millis();
      }
      lastHallSensorState = newHallSensorState;
    }
    delay(300); // make sure the magnet is clear of the hall sensor
  }

  if(targetPosition < currentCurtainPosition){
    setMotorDirection(MOTOR_DIR_CURTAIN_CLOSE, motorSpeed);
    unsigned long rotationStartTime = millis();
    bool lastHallSensorState = readHallSensor();
    // When opening, go to targetposition+1 then come back to targetPosition
    // This way the motor always stops near end of the position.
    while(currentCurtainPosition > targetPosition){
      bool newHallSensorState = readHallSensor();
      unsigned long currentTime = millis();
      digitalWrite(LED_BUILTIN, newHallSensorState);

      if(currentTime - rotationStartTime > ROTATION_SENSOR_TIMEOUT){
        Serial.println("HALT: MOTOR OR ROTATION SENSOR FAILED");
        halt(ERR_ROTATION_FAIL);
      }
      
      if(newHallSensorState == 1 && lastHallSensorState == 0){
        currentCurtainPosition--;
        Serial.print("  currentCurtainPosition=");
        Serial.println(currentCurtainPosition);
        rotationStartTime = millis();
      }
      lastHallSensorState = newHallSensorState;
    }
    delay(300); // make sure the magnet is clear of the hall sensor
  }

  setMotorDirection(MOTOR_STOP, 0);
  digitalWrite(PIN_HV_EN_RELAY, LOW);
}

void handleLightState(LightState state){
  if(state == currentLightState){
    return;
  }

  if(state == LIGHT){
    currentCurtainState = CURTAIN_OPEN;
  }else{
    currentCurtainState = CURTAIN_CLOSED;
  }

  currentLightState = state;
}

// Adjust the position for the CURTAIN_OPEN state.
// brighter -> less open
void adjustOpenPosition(float dbLux) {
  int minPos, maxPos;
  if(dbLux > 34){
    minPos = 2;
    maxPos = 2;
  }else if(dbLux > 32){
    minPos = 2;
    maxPos = 3;
  }else if(dbLux > 30){
    minPos = 3;
    maxPos = 3;
  }else if(dbLux > 28){
    minPos = 3;
    maxPos = ROTATIONS_TO_OPEN;
  }else{
    minPos = ROTATIONS_TO_OPEN;
    maxPos = ROTATIONS_TO_OPEN;
  }
  openPosition = max(minPos, min(maxPos, openPosition));
}

void doLightStat(float dbLux){
  int bucket = dbLux + DBLUX_STAT_BUCKET_OFFSET;
  if(bucket >= DBLUX_STAT_BUCKETS){
    bucket = DBLUX_STAT_BUCKETS - 1;
  }else if (bucket < 0){
    bucket = 0;
  }
  dbLuxStat[bucket]++;
  dbLuxStatSamples++;

  const unsigned long dayInMs = 24 * 3600 * 1000;
  if(millis() - lastThresholdAdjTime > dayInMs){
    int newThreshold = 0;
    int cumulativeSamples = 0;
    for(int i=0; i<DBLUX_STAT_BUCKETS; i++){
      cumulativeSamples += dbLuxStat[i];
      if(cumulativeSamples >= dbLuxStatSamples/2){
        newThreshold = i - DBLUX_STAT_BUCKET_OFFSET;
        break;
      }
    }

    if(newThreshold < DBLUX_MEDIAN_LIMIT){
      newThreshold = DBLUX_MEDIAN_LIMIT;
    }

    dbLuxThreshDark = dbLuxThreshDark * 0.5 + (newThreshold - 5) * 0.5;
    dbLuxThreshLight = dbLuxThreshLight * 0.5 + (newThreshold + 5) * 0.5;

    dbLuxStatSamples = 0;
    for(int i=0; i<DBLUX_STAT_BUCKETS; i++){
      dbLuxStat[i] = 0;
    }
    lastThresholdAdjTime += dayInMs;
  }
}

void handleDbLux(float dbLux){
  if(dbLux < dbLuxThreshDark){
    handleLightState(DARK);
  }else if(dbLux > dbLuxThreshLight){
    handleLightState(LIGHT);
  }
  adjustOpenPosition(dbLux);
  applyCurrentState(MOTOR_SPEED);

  doLightStat(dbLux);
}

void sleepDispatch(uint16_t sleepMs){
#if USE_SNOOZE
  /********************************************************
  * Set Low Power Timer wake up in milliseconds.
  ********************************************************/
  snoozeTimer.setTimer(sleepMs);// milliseconds
  /********************************************************
  * feed the sleep function its wakeup parameters. Then go 
  * to deepSleep.
  ********************************************************/
  Snooze.hibernate(snoozeConfig);
#else
  unsigned long t0 = millis();
  while(millis() - t0 < sleepMs){
    // simulate Snooze's wake on digital pin
    if(manualSwPressed()){
      break; 
    }
  }
#endif
}

// see K20P64M72SF1RM.pdf
// This may be useless. The highest LV warning threshold supported is 3.0V.
// This is below the min rated voltage for the L293D H-bridge and SS443A hall sensor.
// The system may error out and halt due to rotation failure before the 3V3 line drops to 3.0V.
void configureLowVoltageWarning(){
  // choose high detect threshold, enable LV reset
  PMC_LVDSC1 = PMC_LVDSC1_LVDV(1) | PMC_LVDSC1_LVDRE;
  // choose highest warning threshold
  PMC_LVDSC2 = PMC_LVDSC2_LVWV(3);
}

bool lowVoltageWarningTripped(){
  return !!(PMC_LVDSC2 & PMC_LVDSC2_LVWF);
}

void setup() {
  // Wait a bit before turning on low voltage detection.
  // Otherwise the LVD may trigger immediately and require a restart.
  delay(100);
  configureLowVoltageWarning();
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_HV_EN_RELAY, OUTPUT);
  pinMode(PIN_MOTOR_FORWARD, OUTPUT);
  pinMode(PIN_MOTOR_REVERSE, OUTPUT);
  pinMode(PIN_MANUAL_SW, INPUT_PULLUP);
  pinMode(PIN_HALL_SENSOR_READ, INPUT_PULLUP);
  analogWriteFrequency(PIN_MOTOR_FORWARD, 375000);
  
  Serial.begin(9600);

#if USE_SNOOZE
  snoozeDigital.pinMode(PIN_MANUAL_SW, INPUT_PULLUP, FALLING);
#endif

  Wire.begin();
  if(light.begin()){
    Serial.println("Ready to sense some light!");
  }else{
    Serial.println("Could not communicate with the sensor!");
    halt(ERR_SENSOR_INIT);
  }
}

void loop() {
  if(lowVoltageWarningTripped()){
    halt(ERR_LOW_BATT);
  }

  float lux, dbLux;
  
  digitalWrite(LED_BUILTIN, HIGH);


  // manual toggle
  if(manualSwPressed()){
    currentCurtainState = static_cast<CurtainState>(currentCurtainState ^ 1);
    applyCurrentState(255);
    goto endloop;
  }
  
  lux = readLuxAutoRange();
  dbLux = 10.0 * log10(max(lux, 0.0036));
  Serial.print("dbLux=");
  Serial.print(dbLux);
  Serial.print(" light=");
  Serial.print(dbLuxThreshLight);
  Serial.print(" dark=");
  Serial.println(dbLuxThreshDark);
  handleDbLux(dbLux);
  

endloop:
  digitalWrite(LED_BUILTIN, LOW);
  sleepDispatch(60000);
}
