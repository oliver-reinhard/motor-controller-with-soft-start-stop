// --------------------
// CONFIGURABLE VALUES
// --------------------

//#define VERBOSE

const uint8_t MODE_SWITCH_IN_PIN = 8; // digital in
const uint8_t POTENTIOMETER_IN_PIN = A0;  // analog in, motor power demand
const uint8_t STATUS_LED_OUT_PIN = 13; // digital out; is on when motor is of, blinks during transitioning
const uint8_t MOTOR_OUT_PIN = 9; // PWM

const uint16_t MOTOR_MAX_VOLTAGE = 8000; // [mV]
const uint16_t MOTOR_LOW_THRESHOLD_VOLTAGE = 1000; // [mV] // below this voltage, the motor will not move

// Motor soft start and stop:
const uint16_t MOTOR_START_DURATION = 2000;  // [ms] duration from full stop to full throttle
const uint16_t MOTOR_STOP_DURATION = 1000;  // [ms] duration from full throttle to full stop

// --------------------
// DO NOT TOUCH THE VALUES OF THE FOLLOWING CONSTANTS
// --------------------

uint8_t statusLEDState = LOW;

const uint16_t ANALOG_IN_MIN = 0;  // Arduino constant
const uint16_t ANALOG_IN_MAX = 1023;  // Arduino constant

const uint8_t ANALOG_OUT_MIN = 0;  // Arduino constant
const uint8_t ANALOG_OUT_MAX = 255;  // Arduino constant
const uint8_t MOTOR_OUT_LOW_THRESHOLD = (long) ANALOG_OUT_MAX * MOTOR_LOW_THRESHOLD_VOLTAGE /  MOTOR_MAX_VOLTAGE;

// Control cycle: output values are set only once per cycle
const uint32_t CONTROL_CYCLE_DURATION = 100; // [ms]
const uint32_t MODE_SWITCH_POLL_PERIOD = 25; // [ms]

uint32_t controlCycleStartTime = 0; // [ms]

enum MotorState {MOTOR_OFF, MOTOR_STARTING, MOTOR_ON, MOTOR_STOPPING};

MotorState motorState = MOTOR_OFF;

// Motor soft start and soft stop:
const uint16_t MOTOR_START_INCREMENT = (ANALOG_OUT_MAX - MOTOR_OUT_LOW_THRESHOLD) * CONTROL_CYCLE_DURATION / MOTOR_START_DURATION;
const uint16_t MOTOR_STOP_INCREMENT = (ANALOG_OUT_MAX - MOTOR_OUT_LOW_THRESHOLD) * CONTROL_CYCLE_DURATION / MOTOR_STOP_DURATION;

const uint16_t MOTOR_DUTY_VALUE_MIN_CHANGE = 5; // manual potentiometer changes are only acted upon if delta to current value >= this value

uint8_t motorTargetDutyValue = ANALOG_OUT_MIN; // potentiometer value read from input pin
uint8_t motorPreviousTargetDutyValue = ANALOG_OUT_MIN;
uint8_t motorActualDutyValue = ANALOG_OUT_MIN; // value actually set on output pin

uint32_t transitionBeginTime = 0;
uint8_t transitioningDutyValue = ANALOG_OUT_MIN; // incremented in discrete steps until motor is at its target speed or its low end

// Debounced Mode Switch
const uint32_t SWITCH_DEBOUNCE_DELAY = 50;    // the debounce time in [ms]; increase if the output flickers

uint32_t lastModeSwitchReadingTime = 0;  // the last time the input pin was toggled
uint8_t modeSwitchPreviousValue = LOW;   // the previously read modeSwitchValue from the input pin


void setup() {
  pinMode(MODE_SWITCH_IN_PIN, INPUT);
  pinMode(STATUS_LED_OUT_PIN, OUTPUT);
  pinMode(MOTOR_OUT_PIN, OUTPUT);

  #ifdef VERBOSE
  // Setup Serial Monitor
  Serial.begin(9600);
  
  Serial.print("Motor out low threshold: ");
  Serial.println(MOTOR_OUT_LOW_THRESHOLD);
  #endif
  
  controlCycleStartTime = millis();
  setMotorDutyValue(ANALOG_OUT_MIN);
  setStatusLED(HIGH);
}

void loop() {
  uint32_t now = millis();
  
  if (modeSwitchPressed(now)) {
    handleModeChange(now);
  }

  if (now - controlCycleStartTime >= CONTROL_CYCLE_DURATION) {
    controlCycleStartTime = now;
    
    if (motorTargetDutyValueChanged()) {
      handleTargetDutyValueChange(motorTargetDutyValue);
    }
    
    if (motorState == MOTOR_STARTING || motorState == MOTOR_STOPPING) {
      handleModeTransition(motorTargetDutyValue);
    }
  }
  
  delay(MODE_SWITCH_POLL_PERIOD);
}


bool modeSwitchPressed(uint32_t now) {
  if (now - lastModeSwitchReadingTime >= SWITCH_DEBOUNCE_DELAY) {
    uint8_t value = digitalRead(MODE_SWITCH_IN_PIN);
    lastModeSwitchReadingTime = now;

    // check to see if you just pressed the button (i.e. the input went from LOW to HIGH), 
    // and we waited long enough since the last press to ignore any noise:
    if (value != modeSwitchPreviousValue) {
      modeSwitchPreviousValue = value;
      return value == HIGH;
    }
  }
  return false;
}

void handleModeChange(uint32_t now) {
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
  transitionBeginTime = now;
}


bool motorTargetDutyValueChanged() {
  uint16_t potValue = analogRead(POTENTIOMETER_IN_PIN); // Read potentiometer value
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
