#ifndef IO_UTIL_H_INCLUDED
  #define IO_UTIL_H_INCLUDED
 
  #include <Arduino.h>

  typedef uint8_t pin_t;
  
  typedef uint16_t time16_ms_t;
  typedef uint32_t time32_ms_t;
  typedef uint16_t time16_s_t;

  typedef int16_t duration16_ms_t;
  typedef int32_t duration32_ms_t;
  
  typedef uint16_t millivolt_t;
  
  const duration16_ms_t SWITCH_DEBOUNCE_WAIT_MS = 10;
  
  void configInput(pin_t pin);
  
  void configInputWithPullup(pin_t pin);
  
  void configOutput(pin_t pin);

  void turnOnLED(pin_t pin, duration32_ms_t duration);
    
  void flashLED(pin_t pin, uint8_t times);
  
  void debounceSwitch();
  
#endif
