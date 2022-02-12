// --------------------
// CONFIGURABLE VALUES
// --------------------

#define VERBOSE

#define _ATMEGA328_

#ifdef _ATMEGA328_
const uint8_t MODE_SWITCH_IN_PIN = 2; // PD2 – digital in
const uint8_t POTENTIOMETER_IN_PIN = A0;  // analog in, motor power demand

const uint8_t STATUS_LED_OUT_PIN = 13; // PB5 - digital out; is on when motor is off, blinks while transitioning
const uint8_t MOTOR_OUT_PIN = 9; // PB1 - PWM
#endif 

const uint16_t MOTOR_MAX_VOLTAGE = 8000; // [mV]
const uint16_t MOTOR_LOW_THRESHOLD_VOLTAGE = 1000; // [mV] // below this voltage, the motor will not move

// Motor soft start and stop:
const uint16_t MOTOR_START_DURATION = 2000;  // [ms] duration from full stop to full throttle
const uint16_t MOTOR_STOP_DURATION = 1000;  // [ms] duration from full throttle to full stop

// --------------------
// DO NOT TOUCH THE VALUES OF THE FOLLOWING CONSTANTS
// --------------------

uint8_t statusLEDState = LOW;

const uint16_t ANALOG_IN_MIN = 0;     // Arduino constant
const uint16_t ANALOG_IN_MAX = 1023;  // Arduino constant

const uint8_t ANALOG_OUT_MIN = 0;     // Arduino constant
const uint8_t ANALOG_OUT_MAX = 255;   // Arduino constant
const uint8_t MOTOR_OUT_LOW_THRESHOLD = (long) ANALOG_OUT_MAX * MOTOR_LOW_THRESHOLD_VOLTAGE /  MOTOR_MAX_VOLTAGE;

// Control cycle: output values are set only once per cycle
const uint32_t CONTROL_CYCLE_DURATION = 100; // [ms]

enum MotorState {MOTOR_OFF, MOTOR_STARTING, MOTOR_ON, MOTOR_STOPPING};

MotorState motorState = MOTOR_OFF;

// Motor soft start and soft stop:
const uint16_t MOTOR_START_INCREMENT = (ANALOG_OUT_MAX - MOTOR_OUT_LOW_THRESHOLD) * CONTROL_CYCLE_DURATION / MOTOR_START_DURATION;
const uint16_t MOTOR_STOP_INCREMENT = (ANALOG_OUT_MAX - MOTOR_OUT_LOW_THRESHOLD) * CONTROL_CYCLE_DURATION / MOTOR_STOP_DURATION;

const uint16_t MOTOR_DUTY_VALUE_MIN_CHANGE = 5; // manual potentiometer changes are considered only if delta to current value >= this value

uint8_t motorTargetDutyValue = ANALOG_OUT_MIN; // potentiometer value read from input pin
uint8_t motorPreviousTargetDutyValue = ANALOG_OUT_MIN;
uint8_t motorActualDutyValue = ANALOG_OUT_MIN; // value actually set on output pin

uint32_t transitionBeginTime = 0;
uint8_t transitioningDutyValue = ANALOG_OUT_MIN; // incremented in discrete steps until motor is at its target speed or its low end

// "Debounce" Mode push button:
const uint32_t MIN_CONTROLLER_STATE_PERSISTENCE = 1000; // milliseconds
uint32_t lastMotorStateChangeTime = 0;


void setup() {
  configInputWithPullup(MODE_SWITCH_IN_PIN);
  configInput(POTENTIOMETER_IN_PIN);
  
  configOutput(STATUS_LED_OUT_PIN);
  configOutput(MOTOR_OUT_PIN);
  
  configInt0Interrupt(); // triggered by PD2 (mode switch)

  #ifdef VERBOSE
  // Setup Serial Monitor
  Serial.begin(9600);
  
  Serial.print("Motor out low threshold: ");
  Serial.println(MOTOR_OUT_LOW_THRESHOLD);
  #endif
  
  setMotorDutyValue(ANALOG_OUT_MIN);
  setStatusLED(HIGH);
}

void loop() {    
  if (motorTargetDutyValueChanged()) {
    handleTargetDutyValueChange(motorTargetDutyValue);
  }
  
  if (motorState == MOTOR_STARTING || motorState == MOTOR_STOPPING) {
    handleModeTransition(motorTargetDutyValue);
  }
    
  delay(CONTROL_CYCLE_DURATION);
}

//
// Mode Switch pressed
//
ISR (INT0_vect) {       // Interrupt service routine for INT0 on PB2
  uint32_t now = millis();
  if (now - lastMotorStateChangeTime < MIN_CONTROLLER_STATE_PERSISTENCE) {
    return;
  }
  
  #ifdef VERBOSE
    Serial.println("Mode Switch");
  #endif
  switch(motorState) {
    case MOTOR_OFF:
    case MOTOR_STOPPING:
      motorState = MOTOR_STARTING;
      transitioningDutyValue = MOTOR_OUT_LOW_THRESHOLD;
      #ifdef VERBOSE
        Serial.println("STARTING ...");
      #endif
      break;
    
    case MOTOR_ON:
    case MOTOR_STARTING:
      motorState = MOTOR_STOPPING;
      transitioningDutyValue = motorActualDutyValue;
      #ifdef VERBOSE
        Serial.println("STOPPING ...");
      #endif
      break;

    default:
      break;
  } 
  lastMotorStateChangeTime = now;
  transitionBeginTime = now;
}


bool motorTargetDutyValueChanged() {
  uint16_t potValue = readPotentiometer();
  uint8_t value = map(potValue, ANALOG_IN_MIN, ANALOG_IN_MAX, MOTOR_OUT_LOW_THRESHOLD , ANALOG_OUT_MAX); // Map the potentiometer value 
  // only act if target value moves by at least the min. change amount:
  if (abs((short int) value - (short int) motorPreviousTargetDutyValue) >= MOTOR_DUTY_VALUE_MIN_CHANGE) {
    motorTargetDutyValue = value;
    motorPreviousTargetDutyValue = value;
    return true;
  }
  return false;
}

void handleTargetDutyValueChange(uint8_t newTargetDutyValue) {
    if (motorState == MOTOR_ON) {
      setMotorDutyValue(newTargetDutyValue);
    }
    #ifdef VERBOSE
      Serial.print("New Duty Value: ");
      Serial.println(newTargetDutyValue);
    #endif
}

void handleModeTransition(uint8_t targetDutyValue) {
  switch (motorState) {
    case MOTOR_STARTING:
      // ensure we don't overrun the max value of uint8_t:
      if ((short int) transitioningDutyValue + (short int) MOTOR_START_INCREMENT <= (short int) targetDutyValue) {
        transitioningDutyValue += MOTOR_START_INCREMENT;
      } else {
        transitioningDutyValue = targetDutyValue;
      }
      setMotorDutyValue(transitioningDutyValue);
      invertStatusLED();
      #ifdef VERBOSE
        Serial.print("Starting: ");
        Serial.println(transitioningDutyValue);
      #endif
      
      if (transitioningDutyValue == targetDutyValue) {
        motorState = MOTOR_ON;
        transitionBeginTime = 0;
        transitioningDutyValue = ANALOG_OUT_MIN;
        setStatusLED(LOW);
        #ifdef VERBOSE
          Serial.println("MOTOR ON.");
        #endif
      }
      break;
    
    case MOTOR_STOPPING:
      // ensure transitioningDutyValue will not become < 0 (is an uint8_t!)
      if ((short int) transitioningDutyValue - (short int) MOTOR_STOP_INCREMENT > (short int) MOTOR_OUT_LOW_THRESHOLD) {
        transitioningDutyValue -= MOTOR_STOP_INCREMENT;
      } else {
        // new value is below MOTOR_OUT_LOW_THRESHOLD --> turn motor off completely:
        transitioningDutyValue = ANALOG_OUT_MIN;
      }
      setMotorDutyValue(transitioningDutyValue);
      invertStatusLED();
      #ifdef VERBOSE
        Serial.print("Stopping: ");
        Serial.println(transitioningDutyValue);
      #endif
      
      if (transitioningDutyValue == ANALOG_OUT_MIN) {
        motorState = MOTOR_OFF;
        transitionBeginTime = 0;
        transitioningDutyValue = ANALOG_OUT_MIN;
        setStatusLED(HIGH);
        #ifdef VERBOSE
          Serial.println("MOTOR OFF");
        #endif
      }
      break;

    default:
      break;
  }
}

uint16_t readPotentiometer() {
  #ifdef _ATMEGA328_
  return analogRead(POTENTIOMETER_IN_PIN);
  #endif
}

void setMotorDutyValue(uint8_t value) {
  motorActualDutyValue = value;
  analogWrite(MOTOR_OUT_PIN, motorActualDutyValue); // Send PWM signal
}

void setStatusLED(int value) {
  statusLEDState = value;
  digitalWrite(STATUS_LED_OUT_PIN, value);
}

void invertStatusLED() {
  setStatusLED(statusLEDState == HIGH ? LOW : HIGH);
}


void configInput(uint8_t pin) {
  #ifdef _ATMEGA328_
  pinMode(pin, INPUT);
  #endif
}

void configInputWithPullup(uint8_t pin) {
  #ifdef _ATMEGA328_
  pinMode(pin, INPUT);
  digitalWrite(pin, HIGH);              // Activate pull-up resistor on pin (input)
  #endif
}

void configOutput(uint8_t pin) {
  #ifdef _ATMEGA328_
  pinMode(pin, OUTPUT);
  #endif
}

void configInt0Interrupt() {
  #ifdef _ATMEGA328_
  EIMSK |= (1<<INT0);      // Enable INT0 (external interrupt) 
  EICRA |= (1<<ISC01);     // Configure as falling edge (pull-up resistor!)
  #endif
}
