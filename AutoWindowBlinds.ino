// using snooze breaks USB serial print
// disable snooze for debugging
#define USE_SNOOZE 1

#define ANALOG_BITS 16
#define ANALOG_FULLSCALE ((1 << ANALOG_BITS) - 1)

#define MOTOR_SPEED 240
#define ROTATIONS_TO_OPEN 4
#define ROTATION_SENSOR_TIMEOUT 12000

#define ERR_LOW_BATT 1
#define ERR_ROTATION_FAIL 2

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

// To save power, only send power to the photo resistor when reading it
#define PIN_PHOTORESISTOR_POWER 15
#define PIN_PHOTORESISTOR_READ A0 // aka pin 14
#define PHOTORESISTOR_PARTNER_R 10000

// push button for manually toggling open/close
#define PIN_MANUAL_SW 11

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
LightState currentLightState = DARK;

// default light and dark threshold
uint32_t resistanceThresholdDark = 50000 * 1.2;
uint32_t resistanceThresholdLight = 50000 * 0.8;

// brightness distribution in the last 24 hours.
// each bucket covers 1000 ohms of resistance.
// every 24 hours, the resistance thresholds are adjusted based on the brightness stat.
#define RESISTANCE_STAT_BUCKETS 100
#define RESISTANCE_STAT_BUCKET_SIZE 1000
int resistanceStat[100] = {0};
int resistanceStatSamples = 0;
unsigned long lastThresholdAdjTime = 0;

// When the blinds are open, it's dark outside and the lights are on, the photoresistor value is about 80k.
// The limit needs to be less than 80k / 1.2 due to the +/- 20% margin for the dark and light thresholds.
#define RESISTANCE_MEDIAN_LIMIT 60000

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

// Wiring: 3.3v - photoresistor - partner_r - GND
uint32_t readPhotoresistor(){
  digitalWrite(PIN_PHOTORESISTOR_POWER, HIGH);
  delayMicroseconds(1);
  uint64_t analogVal16 = 0;
  for(int i=0; i<16; i++){
    analogVal16 += analogRead(PIN_PHOTORESISTOR_READ);
  }
  digitalWrite(PIN_PHOTORESISTOR_POWER, LOW);
  uint32_t resistance = (ANALOG_FULLSCALE*16 - analogVal16) * PHOTORESISTOR_PARTNER_R / analogVal16;
  return resistance;
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

void setCurtainState(CurtainState state, int motorSpeed){
  Serial.println(__FUNCTION__);
  if(state == currentCurtainState){
    return;
  }

  int positionDelta;
  int targetPosition;
  int motorDirection;
  if(state == CURTAIN_OPEN){
    targetPosition = ROTATIONS_TO_OPEN;
  }else{
    targetPosition = 0;
  }

  if(targetPosition - currentCurtainPosition > 0){
    motorDirection = MOTOR_DIR_CURTAIN_OPEN;
    positionDelta = 1;
  }else if(targetPosition - currentCurtainPosition < 0){
    positionDelta = -1;
    motorDirection = MOTOR_DIR_CURTAIN_CLOSE;
  }else{
    goto done;
  }

  Serial.print("  motorDirection=");
  Serial.println(motorDirection);
  Serial.print("  currentCurtainPosition=");
  Serial.println(currentCurtainPosition);
  Serial.print("  targetPosition=");
  Serial.println(targetPosition);
  Serial.print("  positionDelta=");
  Serial.println(positionDelta);
  Serial.print("  rotations left = ");
  Serial.println((targetPosition - currentCurtainPosition) / positionDelta);
  if(targetPosition != currentCurtainPosition){
    digitalWrite(PIN_HV_EN_RELAY, HIGH);
    delay(1000);
    setMotorDirection(motorDirection, motorSpeed);

    bool lastHallSensorState = readHallSensor();
    unsigned long rotationStartTime = millis();
    while((targetPosition - currentCurtainPosition) / positionDelta > 0){
      bool newHallSensorState = readHallSensor();
      unsigned long currentTime = millis();
      digitalWrite(LED_BUILTIN, newHallSensorState);

      if(currentTime - rotationStartTime > ROTATION_SENSOR_TIMEOUT){
        Serial.println("HALT: MOTOR OR ROTATION SENSOR FAILED");
        halt(ERR_ROTATION_FAIL);
      }
      
      if(newHallSensorState == 1 && lastHallSensorState == 0){
        currentCurtainPosition += positionDelta;
        Serial.print("  currentCurtainPosition=");
        Serial.println(currentCurtainPosition);
        rotationStartTime = millis();
      }
      lastHallSensorState = newHallSensorState;
    }
    delay(300); // make sure the magnet is clear of the hall sensor

    setMotorDirection(MOTOR_STOP, 0);
    digitalWrite(PIN_HV_EN_RELAY, LOW);
  }

done:
  currentCurtainState = state;
}

void handleLightState(LightState state){
  if(state == currentLightState){
    return;
  }

  if(state == LIGHT){
    setCurtainState(CURTAIN_OPEN, MOTOR_SPEED);
  }else{
    setCurtainState(CURTAIN_CLOSED, MOTOR_SPEED);
  }

  currentLightState = state;
}

void doResistanceStat(uint32_t resistance){
  int bucket = resistance / RESISTANCE_STAT_BUCKET_SIZE;
  if(bucket >= RESISTANCE_STAT_BUCKETS){
    bucket = RESISTANCE_STAT_BUCKETS - 1;
  }
  resistanceStat[bucket]++;
  resistanceStatSamples++;

  const unsigned long dayInMs = 24 * 3600 * 1000;
  if(millis() - lastThresholdAdjTime > dayInMs){
    int newThreshold = 0;
    int cumulativeSamples = 0;
    for(int i=0; i<RESISTANCE_STAT_BUCKETS; i++){
      cumulativeSamples += resistanceStat[i];
      if(cumulativeSamples >= resistanceStatSamples/2){
        newThreshold = i * RESISTANCE_STAT_BUCKET_SIZE;
        break;
      }
    }

    if(newThreshold > RESISTANCE_MEDIAN_LIMIT){
      newThreshold = RESISTANCE_MEDIAN_LIMIT;
    }

    resistanceThresholdDark = resistanceThresholdDark * 0.5 + (newThreshold * 1.2) * 0.5;
    resistanceThresholdLight = resistanceThresholdLight * 0.5 + (newThreshold * 0.8) * 0.5;

    resistanceStatSamples = 0;
    for(int i=0; i<RESISTANCE_STAT_BUCKETS; i++){
      resistanceStat[i] = 0;
    }
    lastThresholdAdjTime += dayInMs;
  }
}

void handlePhotoresistorValue(uint32_t resistance){
  LightState newLightState;
  if(resistance > resistanceThresholdDark){
    newLightState = DARK;
  }else if(resistance < resistanceThresholdLight){
    newLightState = LIGHT;
  }else{
    return;
  }
  handleLightState(newLightState);

  doResistanceStat(resistance);
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
  configureLowVoltageWarning();
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_HV_EN_RELAY, OUTPUT);
  pinMode(PIN_MOTOR_FORWARD, OUTPUT);
  pinMode(PIN_MOTOR_REVERSE, OUTPUT);
  pinMode(PIN_PHOTORESISTOR_POWER, OUTPUT);
  pinMode(PIN_PHOTORESISTOR_READ, INPUT);
  pinMode(PIN_MANUAL_SW, INPUT_PULLUP);
  pinMode(PIN_HALL_SENSOR_READ, INPUT_PULLUP);
  analogReadResolution(ANALOG_BITS);
  analogWriteFrequency(PIN_MOTOR_FORWARD, 375000);
  
  Serial.begin(9600);

#if USE_SNOOZE
  snoozeDigital.pinMode(PIN_MANUAL_SW, INPUT_PULLUP, FALLING);
#endif
}

void loop() {
  if(lowVoltageWarningTripped()){
    halt(ERR_LOW_BATT);
  }
  
  uint32_t r;
  
  digitalWrite(LED_BUILTIN, HIGH);


  // manual toggle
  if(manualSwPressed()){
    setCurtainState(static_cast<CurtainState>(currentCurtainState ^ 1), 255);
    goto endloop;
  }
  
  r = readPhotoresistor();
  Serial.print("photoresistor R=");
  Serial.print(r);
  Serial.print(" light=");
  Serial.print(resistanceThresholdLight);
  Serial.print(" dark=");
  Serial.println(resistanceThresholdDark);
  handlePhotoresistorValue(r);
  

endloop:
  digitalWrite(LED_BUILTIN, LOW);
  sleepDispatch(60000);
}
