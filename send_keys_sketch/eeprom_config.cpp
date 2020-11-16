#include <Arduino.h>
#include <EEPROM.h>

#include "keyboard_hid_data.h"
#include "eeprom_config.h"

uint8_t pinsToWatch = 0; 
uint8_t pinsLast = 0;

void formatEeprom() {
  Serial.println(F("Formatting EEPROM"));
  
  EEPROM.update(0, EEPROM_CHECK_BYTE_1);
  EEPROM.update(1, EEPROM_CHECK_BYTE_2);
  
  for (uint8_t pin = FIRST_INPUT_PIN; pin <= LAST_INPUT_PIN; pin++) {
    uint16_t eepromOffset = EEPROM_OFFSET(pin);
    
    if (pin == DEFAULT_INPUT_PIN) {
      for (char *defaultString = DEFAULT_INPUT_STRING; *defaultString; defaultString++) {
        uint8_t code, modifier;
        code = asciiToKey(*defaultString, &modifier);
        EEPROM.update(eepromOffset++, modifier);
        EEPROM.update(eepromOffset++, code);
      }      
    } 
    
    EEPROM.update(eepromOffset, 0);
    EEPROM.update(eepromOffset + 1, 0);
  }
}

void readAndProcessConfig() {
  if (EEPROM.read(0) != EEPROM_CHECK_BYTE_1 || EEPROM.read(1) != EEPROM_CHECK_BYTE_2)
    formatEeprom();

  pinsToWatch = 0;
    
  for (uint8_t pin = FIRST_INPUT_PIN; pin <= LAST_INPUT_PIN; pin++) {
    uint16_t eepromOffset = EEPROM_OFFSET(pin);

    Serial.print("Pin ");
    Serial.print(pin);
    Serial.print(": ");
    
    if (!EEPROM.read(eepromOffset + 1)) {
      Serial.println("off");
      continue;
    }

    for (uint8_t keystrokeIdx = 0; keystrokeIdx < MAX_KEYSTROKES; keystrokeIdx++) {
      uint8_t modifier = EEPROM.read(eepromOffset + (keystrokeIdx * 2));
      uint8_t code     = EEPROM.read(eepromOffset + (keystrokeIdx * 2) + 1);
      
      if (!code)
        break;
        
      if (keystrokeIdx > 0)
        Serial.print(" ");
        
      if (modifier < 16) 
        Serial.print("0");
      Serial.print(modifier, HEX);
      
      Serial.print(" ");
      
      if (code < 16) 
        Serial.print("0");
      Serial.print(code, HEX);
    }
    Serial.println("");

    pinMode(pin, INPUT_PULLUP);    
    pinsToWatch |= 1 << (pin - FIRST_INPUT_PIN);
    pinsLast |= 1 << (pin - FIRST_INPUT_PIN);
  }  
}

uint8_t checkPinChange(uint8_t pin, uint8_t *newValue) {
  uint8_t pinRead = digitalRead(pin);
  uint8_t pinNew = pinRead << (pin - FIRST_INPUT_PIN);
  uint8_t pinOld = pinsLast & (1 << (pin - FIRST_INPUT_PIN));

  pinsLast = (pinsLast & (~(1 << (pin - FIRST_INPUT_PIN)))) | pinNew;
  *newValue = pinRead; 

  return pinNew != pinOld;
} 
