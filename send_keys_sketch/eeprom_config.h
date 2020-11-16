#ifndef eeprom_config_h
#define eeprom_config_h

#define EEPROM_CHECK_BYTE_1 0x65
#define EEPROM_CHECK_BYTE_2 0x89
#define EEPROM_HEADER_SIZE 2
#define EEPROM_SIZE      (E2END + 1)

#define FIRST_INPUT_PIN  4      // Set to first input pin that can be used to trigger a set of keystrokes
#define MAX_KEYSTROKES   64     // Set to the maximum number of keystrokes available to be sent for each input
#define MAX_INPUT_PINS   min((((EEPROM_SIZE - EEPROM_HEADER_SIZE) / (MAX_KEYSTROKES * 2))|0), 8)
#define LAST_INPUT_PIN   (FIRST_INPUT_PIN + MAX_INPUT_PINS - 1)

#define DEFAULT_INPUT_PIN 7
#define DEFAULT_INPUT_STRING "Hello World!"

#define EEPROM_OFFSET(pin) (2 + ((pin - FIRST_INPUT_PIN) * (MAX_KEYSTROKES * 2)))
#define WATCH_PIN(pin) (pinsToWatch & (1 << (pin - FIRST_INPUT_PIN)))

extern uint8_t pinsToWatch; 
extern uint8_t pinsLast;

void formatEeprom();
void readAndProcessConfig();
uint8_t checkPinChange(uint8_t pin, uint8_t *newValue);

#endif 
