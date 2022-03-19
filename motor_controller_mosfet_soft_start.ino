
#include <avr/io.h>
#include <util/delay.h>
#include "io_util.h"

// --------------------
// CONFIGURABLE VALUES
//
// Voltages are always in [mV].
// Durations are always in [ms].
// --------------------

const millivolt_t MOTOR_MAX_VOLTAGE           = 6000;   // [mV]
const millivolt_t MOTOR_LOW_THRESHOLD_VOLTAGE = 2500;   // [mV] // below this voltage, the motor will not rotate

// Motor soft start and stop:
const millivolt_t MOTOR_START_DURATION = 2000;  // [ms] duration from full stop to full throttle
const millivolt_t MOTOR_STOP_DURATION = 1000;   // [ms] duration from full throttle to full stop

// --------------------
// DO NOT TOUCH THE VALUES OF THE FOLLOWING CONSTANTS
// --------------------

#if defined(__AVR_ATmega328P__)
  //#define VERBOSE
  const pin_t MODE_SWITCH_IN_PIN = 2;     // PD2 – digital in
  const pin_t POTENTIOMETER_IN_PIN = A0;  // analog in, motor power demand
  
  const pin_t MOTOR_OUT_PIN = 3;          // PD3 - PWM @ native frequency
  const pin_t STATUS_LED_OUT_PIN = 4;     // PD4 - digital out; is on when motor is off, blinks while transitioning
    
#elif defined(__AVR_ATtiny85__)
  const pin_t MODE_SWITCH_IN_PIN = PB2;
  const pin_t POTENTIOMETER_IN_PIN = PB4; // analog in, motor power demand
  
  const pin_t MOTOR_OUT_PIN = PB1;        // PWM @ 25 kHz
  const pin_t STATUS_LED_OUT_PIN = PB0;   // digital out; is on when motor is off, blinks while transitioning
#endif 

//
// CONTROLLER STATES
//

typedef enum {MOTOR_OFF, MOTOR_STARTING, MOTOR_ON, MOTOR_STOPPING} MotorState;

MotorState motorState = MOTOR_OFF;

// Control cycle: output values are set only once per cycle
const uint32_t CONTROL_CYCLE_DURATION = 100; // [ms]


//
// ANALOG IN
//
const uint16_t ANALOG_IN_MIN = 0;     // Arduino constant
#if defined(__AVR_ATmega328P__)
  const uint16_t ANALOG_IN_MAX = 1023;  // Arduino constant
    
#elif defined(__AVR_ATtiny85__)
  const uint16_t ANALOG_IN_MAX = 255;  // Arduino constant
  uint16_t lastAnalogInValue = 0;
#endif

//
// ANALOG OUT
//
#if defined(__AVR_ATmega328P__)
  const uint8_t ANALOG_OUT_MIN = 0;        // Arduino constant
  const uint8_t ANALOG_OUT_MAX = 255;      // PWM control
    
#elif defined(__AVR_ATtiny85__)
// PWM frequency = 1 MHz / 1 / 200 = 5 kHz 
  const uint8_t TIMER1_PRESCALER = 1;     // divide by 1
  const uint8_t TIMER1_CTC_COUNT_TO = 200;    // count to 255
  
  const uint8_t ANALOG_OUT_MIN = 0;                 // Arduino constant
  const uint8_t ANALOG_OUT_MAX = TIMER1_CTC_COUNT_TO;   // PWM control
#endif

const uint8_t MOTOR_OUT_LOW_THRESHOLD = (uint32_t) ANALOG_OUT_MAX * MOTOR_LOW_THRESHOLD_VOLTAGE /  MOTOR_MAX_VOLTAGE;

// Motor soft start and soft stop:
const uint16_t MOTOR_START_INCREMENT = (ANALOG_OUT_MAX - MOTOR_OUT_LOW_THRESHOLD) * CONTROL_CYCLE_DURATION / MOTOR_START_DURATION;
const uint16_t MOTOR_STOP_INCREMENT = (ANALOG_OUT_MAX - MOTOR_OUT_LOW_THRESHOLD) * CONTROL_CYCLE_DURATION / MOTOR_STOP_DURATION;

const uint16_t MOTOR_DUTY_VALUE_MIN_CHANGE = 5; // manual potentiometer changes are considered only if delta to current value >= this value

uint8_t motorTargetDutyValue = ANALOG_OUT_MIN; // potentiometer value read from input pin
uint8_t motorPreviousTargetDutyValue = ANALOG_OUT_MIN;
uint8_t motorActualDutyValue = ANALOG_OUT_MIN; // value actually set on output pin
uint32_t transitionBeginTime = 0;
uint8_t transitioningDutyValue = ANALOG_OUT_MIN; // incremented in discrete steps until motor is at its target speed or its low end

//
// DIGITAL IN
//

// "Debounce" Mode-switch button:
uint32_t lastMotorStateChangeTime = 0;
const uint32_t MIN_CONTROLLER_STATE_PERSISTENCE = 1000; // milliseconds

//
// DIGITAL OUT
//
boolean statusLEDState = LOW;

//
// SETUP
//
void setup() {
  configInputWithPullup(MODE_SWITCH_IN_PIN);
  configInput(POTENTIOMETER_IN_PIN);
  
  configOutput(STATUS_LED_OUT_PIN);
  configOutput(MOTOR_OUT_PIN);
  
  configInt0Interrupt(); // triggered by PD2 (mode switch)
  configAnalogDigitalConversion0();
  sei();
  configPWM1();

  #ifdef VERBOSE
    // Setup Serial Monitor
    Serial.begin(9600);
    
    Serial.print("Motor out low threshold: ");
    Serial.println(MOTOR_OUT_LOW_THRESHOLD);
  #endif
  
  setMotorDutyValue(ANALOG_OUT_MIN);
  setStatusLED(HIGH);
}

//
// LOOP
//
void loop() {    
  if (motorTargetDutyValueChanged()) {
    handleTargetDutyValueChange(motorTargetDutyValue);
  }
  
  if (motorState == MOTOR_STARTING || motorState == MOTOR_STOPPING) {
    handleModeTransition(motorTargetDutyValue);
  }
    
  _delay_ms(CONTROL_CYCLE_DURATION);
}

//
// Mode Switch pressed
//
void handleInputPressed(uint32_t now) {
  if (now - lastMotorStateChangeTime < MIN_CONTROLLER_STATE_PERSISTENCE) {
    #ifdef VERBOSE
      Serial.println("Mode Switch pressed - ignored");
    #endif
    return;
  }
  
  #ifdef VERBOSE
    Serial.println("Mode Switch pressed");
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

//
// Target value modified via potentiometer
//
bool motorTargetDutyValueChanged() {
  uint16_t potValue = readPotentiometer();
  uint8_t value = map(potValue, ANALOG_IN_MIN, ANALOG_IN_MAX, MOTOR_OUT_LOW_THRESHOLD , ANALOG_OUT_MAX); // Map the potentiometer value 
  // Only act if target value moves by at least the min. change amount:
  if (abs((short int) value - (short int) motorPreviousTargetDutyValue) >= MOTOR_DUTY_VALUE_MIN_CHANGE) {
    // with this rule, we may never get to the extremes, MOTOR_OUT_LOW_THRESHOLD and ANALOG_OUT_MAX --> additional rules below
    motorTargetDutyValue = value;
    motorPreviousTargetDutyValue = value;
    return true;
  } else if (value < MOTOR_OUT_LOW_THRESHOLD + MOTOR_DUTY_VALUE_MIN_CHANGE) {
    motorTargetDutyValue = MOTOR_OUT_LOW_THRESHOLD;
    motorPreviousTargetDutyValue = MOTOR_OUT_LOW_THRESHOLD;
    return true;
  } else if (value > ANALOG_OUT_MAX - MOTOR_DUTY_VALUE_MIN_CHANGE) {
    motorTargetDutyValue = ANALOG_OUT_MAX;
    motorPreviousTargetDutyValue = ANALOG_OUT_MAX;
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

//
// State controller
//
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

//
// Utility Routines
//
void setMotorDutyValue(uint8_t value) {
  motorActualDutyValue = value;
  #if defined(__AVR_ATmega328P__)
    analogWrite(MOTOR_OUT_PIN, value); // Send PWM signal
    
  #elif defined(__AVR_ATtiny85__)
    OCR1A = value;
  #endif
}

void setStatusLED(boolean value) {
  statusLEDState = value;
  digitalWrite(STATUS_LED_OUT_PIN, value);
}

void invertStatusLED() {
  setStatusLED(statusLEDState == HIGH ? LOW : HIGH);
}

uint16_t readPotentiometer() {
  #if defined(__AVR_ATmega328P__)
    return analogRead(POTENTIOMETER_IN_PIN);
    
  #elif defined(__AVR_ATtiny85__)
    ADCSRA|=(1<<ADSC);                     // Start ADC conversion
    loop_until_bit_is_clear(ADCSRA, ADSC); // Wait until done
    uint16_t adcValue = ADCH;              // read left-adjusted 8 bits --> 0…255
    return adcValue;
  #endif
}

void configInt0Interrupt() {
  #ifdef __AVR_ATmega328P__
    EIMSK |= (1<<INT0);      // Enable INT0 (external interrupt) 
    EICRA |= (1<<ISC01);     // Configure as falling edge (pull-up resistor!)
    
  #elif defined(__AVR_ATtiny85__)
    GIMSK |= (1<<INT0);      // Enable INT0 (external interrupt) 
    MCUCR |= (1<<ISC01);     // Configure as falling edge (pull-up resistor!)
  #endif
}

ISR (INT0_vect) {       // Interrupt service routine for INT0 on PB2
  uint32_t now = millis();
  handleInputPressed(now);
}

void configAnalogDigitalConversion0() {
  #if defined(__AVR_ATmega328P__)
    // nothing --> analogRead
    
  #elif defined(__AVR_ATtiny85__)
    // | REFS1 | REFS0 | ADLAR | REFS2 | MUX[3:0] |
    // |  1    |  1    |  1    |  1    |  4       | ->  #bits
    
    // Clear all MUX bits:
    // REFS[0:2] = 000: reference voltage == VCC 
    // ADLAR = 0 -->  Right-adjust
    // MUX = 0000 --> ADC0 (PB5)
    ADMUX = B00000000; 
    
    // Set Left-Adjust Result -> read only the 8 bits from ADCH, do not read ADCL
    ADMUX |= (1<<ADLAR);
    
    // Set PB4:
    ADMUX |= (1<<MUX1);
  
    // Prescaler wants to run at 50..200 KHz
    // Based on INTERNAL CLK 1 MHz --> prescale factor 8 --> 125 KHz
    
    ADCSRA |= (1<<ADPS1) | (1<<ADPS0);     // ADC clock prescaler /8 */
    ADCSRA |= (1<<ADEN);                   // enable ADC */
  #endif
}

void configPWM1() {
  #if defined(__AVR_ATmega328P__)
    // nothing --> use analogWrite as is
    // No specific PWM frequency
    
  #elif defined(__AVR_ATtiny85__)
    // Configure Timer/Counter1 Control Register 1 (TCR1) 
    // | CTC1 | PWM1A | COM1A | CS |
    // |  1   |  1    |  2    | 4  |  ->  #bits
    //
    // CTC1 - Clear Timer/Counter on Compare Match: When set (==1), TCC1 is reset to $00 in the CPU clock cycle after a compare match with OCR1C register value.
    // PWM1A - Pulse Width Modulator A Enable: When set (==1), enables PWM mode based on comparator OCR1A in TC1 and the counter value is reset to $00 in the CPU clock cycle after a compare match with OCR1C register value.
    // COM1A - Comparator A Output Mode: determines output-pin action following a compare match with compare register A (OCR1A) in TC1
    // CS - Clock Select Bits: defines the prescaling factor of TC1
  
    // Clear all TCCR1 bits:
    TCCR1 &= B00000000;      // Clear 
  
    // Clear Timer/Counter on Compare Match (CTC): count from 0, 1, 2 .. OCR1C, 0, 1, 2 .. ORC1C, etc
    TCCR1 |= (1<<CTC1);
    
    // Enable PWM A based on OCR1A
    TCCR1 |= (1<<PWM1A);
    
    // On Compare Match with OCR1A (counter == OCR1A): Clear the output line (-> LOW), set on $00
    TCCR1 |= (1<<COM1A1);
  
    // Configure PWM frequency:
    TCCR1 |= TIMER1_PRESCALER;  // Prescale factor
    OCR1C = TIMER1_CTC_COUNT_TO;    // Count 0,1,2..compare-match,0,1,2..compare-match, etc
  
    // Determines Duty Cycle: OCR1A / OCR1C e.g. value of 50 / 200 --> 25%,  value of 50 --> 0%
    OCR1A = 0;
  #endif
}
