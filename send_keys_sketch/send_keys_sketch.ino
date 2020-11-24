/*
  Demonstrate using the the atmega16u2-usbtoserial-and-keyboard firmware 
  on the atmega16u2 in the Arduino Uno R3 to both have a serial console 
  that can be used in sketches and to load new sketches from the Arduino 
  IDE as well as sending keyboard codes via the USB Keyboard HID 

  This sketch allows you to program input pins 2-7 so that when they
  are pulled to ground they will send up to 64 keystrokes via USB 
  HID. By default input pin 7 will send the string 'Hello World!'
*/

#include <EEPROM.h>
#include "eeprom_config.h"
#include "keyboard_hid_data.h"
#include "keyboard_hid_defines.h"

char inChar;        // character received from serial port

static char *all_major_char_str = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789#%^&*!*()\t -_=+[]{}\\|`~;:'\",.<>/?";

void setup() {
  Serial.begin(115200);                 // initialize serial port and set baud rate to 9600

  Serial.println(F("Arduino keyboard sender (https://github.com/shaun4477/arduino-uno-r3-usb-keyboard)"));
  readAndProcessConfig();
    
  delay(100);
}

/* Dump the ascii to keyboard HID event map */
void printAsciiToKey() {
  for (uint8_t i = 0; i < strlen(all_major_char_str); i++) {
    uint8_t modifier = 0;
    uint8_t code = asciiToKey(all_major_char_str[i], &modifier);
    Serial.print("Character '");
    Serial.print(all_major_char_str[i]);
    Serial.print("' = ");
    Serial.print(code, HEX);
    Serial.print(" modifier = ");
    Serial.print(modifier);
    Serial.println("");
  }
}

void serialPrintHex(long num) {
  if (num < 16) 
    Serial.print("0");
  Serial.print(num, HEX);
}

/* Send all major keys to the keyboard USB device */
void sendAllMajorKeys() {
  for (uint8_t i = 0; i < strlen(all_major_char_str); i++) {
    uint8_t chr = all_major_char_str[i];

    // Don't send these two as they may cause the serial monitor to re-trigger
    if (chr == '\n' || chr == '\t')
      continue;
      
    uint8_t modifier = 0;
    uint8_t code = asciiToKey(all_major_char_str[i], &modifier);
    sendKey(modifier, code, 0x0);
    delay(100);
  }
}

void loop() {  
  for (uint8_t pin = FIRST_INPUT_PIN; pin <= LAST_INPUT_PIN; pin++) {
    if (!WATCH_PIN(pin))
      continue;

    uint8_t pinChanged, pinValue;
    pinChanged = checkPinChange(pin, &pinValue);

    if (!pinChanged)
      continue;
    
    Serial.print("Pin ");
    Serial.print(pin);
    Serial.print(" change: ");   
    Serial.println(pinValue);   

    uint16_t eepromOffset = EEPROM_OFFSET(pin);
    
    if (!pinValue) {
      for (uint8_t keystrokeIdx = 0; keystrokeIdx < MAX_KEYSTROKES; keystrokeIdx++) {
        uint8_t modifier = EEPROM.read(eepromOffset + (keystrokeIdx * 2));
        uint8_t code     = EEPROM.read(eepromOffset + (keystrokeIdx * 2) + 1);
        
        if (!code)
          break;

        Serial.print("Send key ");
        serialPrintHex(modifier);
        Serial.print(" ");
        serialPrintHex(code);
        Serial.println("");
        
        sendKey(modifier, code, 0x0);
      }
    }
  }
  delay(5);
}

#define SERIAL_TIMEOUT_MS 500

int serialTimedPeek() {
  int c;
  unsigned long _startMillis; 
  
  _startMillis = millis();
  do {
    if (Serial.available()) {
      c = Serial.peek();
      return c;
    }
  } while(millis() - _startMillis < SERIAL_TIMEOUT_MS);
  return -1;     // -1 indicates timeout
}

int serialTimedSkipWhitespace(char *terminator) {
  while (1) {
    char nextChar = serialTimedPeek();
    if (nextChar == -1)
      return -1;
    else if (nextChar == ' ' || nextChar == '\t')
      Serial.read();
    else {
      if (terminator)
        *terminator = nextChar;
      return 0;
    }
  }  
}

int serialTimedReadNum(uint8_t *out, char *terminator, bool hex) {
  bool firstChar = true; 
  
  if (serialTimedSkipWhitespace(NULL) == -1) {
    Serial.println("Timeout 2");
    return -1;
  }

  *out = 0;

  while (1) {
    char nextChar = serialTimedPeek();
    if (nextChar == -1) {
      return -1;
    }

    uint8_t charVal;

    if (nextChar >= '0' && nextChar <= '9')
      charVal = nextChar - '0';
    else if (hex && nextChar >= 'a' && nextChar <= 'f')
      charVal = nextChar - 'a' + 10;
    else if (hex && nextChar >= 'A' && nextChar <= 'F')
      charVal = nextChar - 'A' + 10;
    else {
      if (firstChar)
        return -1;
      *terminator = nextChar;
      return 0;
    }

    Serial.read();

    *out *= hex ? 16 : 10;
    *out += charVal;
    firstChar = false;
  }  
}

int readPinUpdate() {
  uint8_t pin;
  int rc;
  char terminator;
  
  if (rc = serialTimedReadNum(&pin, &terminator, false) || terminator != ' ') {
    Serial.println("Invalid pin"); 
    return -1; 
  }

  if (pin < 0 || pin < FIRST_INPUT_PIN || pin > LAST_INPUT_PIN) {
    Serial.print("Invalid input pin ");
    Serial.print(pin);
    Serial.println("");
    return -1;
  }

  Serial.print("Updating pin ");
  Serial.println(pin);

  uint8_t keystrokeIdx = 0;
  while (keystrokeIdx < MAX_KEYSTROKES - 1) {
    uint8_t modifier, keycode;

    rc = serialTimedSkipWhitespace(&terminator);
    if (rc) {
      Serial.println("Timeout looking for modifier");
      return -1;
    } else if (terminator == ';') {
      Serial.read();
      break;      
    }

    rc = serialTimedReadNum(&modifier, &terminator, true);
    if (rc) {
      Serial.println("Timeout reading modifier");
      return;
    } else if (terminator != ' ') {
      Serial.println("No space after modifier");
      return -1;      
    }

    rc = serialTimedReadNum(&keycode, &terminator, true);
    if (rc) {
      Serial.println("Timeout reading keycode");
      return;
    } else if (!(terminator == ' ' || terminator == ';')) {
      Serial.println("No terminator after keycode");
      return -1;      
    }

    Serial.print("Read modifier ");
    serialPrintHex(modifier);
    Serial.print(" ");

    Serial.print("keycode ");
    serialPrintHex(keycode);
    Serial.println();

    EEPROM.update(EEPROM_OFFSET(pin) + (keystrokeIdx * 2), modifier);
    EEPROM.update(EEPROM_OFFSET(pin) + (keystrokeIdx * 2) + 1, keycode);
    keystrokeIdx++;    
  }  

  if (keystrokeIdx < MAX_KEYSTROKES - 1)
    EEPROM.update(EEPROM_OFFSET(pin) + (keystrokeIdx * 2), 0);
    EEPROM.update(EEPROM_OFFSET(pin) + (keystrokeIdx * 2) + 1, 0);  

  Serial.println("Updated keycode, reloaded config");
  readAndProcessConfig();
}


void serialEvent() {
  if (Serial.available()) {         
    inChar = Serial.read();         

    // Serial.print("Received: ");  
    // Serial.println(inChar);         

    switch (inChar) {
      case 's':
        // Send a keystroke, the actual key data should never 
        // appear in the console since it will become a HID key
        Serial.print("key:");       // print the string "key:"
        sendKey(0x00, 0x1d, 0);
        Serial.println(""); 
        break;
      case 'S':
        // Send all major keys to the Keyboard USB device
        sendAllMajorKeys();
        break;
      case 'i':
        // Send an incomplete keystroke to show it doesn't block
        Serial.print("incomplete: ");       // print the string "key:"
        Serial.write(keyboardMessage, 7);
        delay(500);
        Serial.println(""); 
        Serial.println("done");
        break;
      case 'd':
        // Dump ascii char to keycode data
        printAsciiToKey();
        break;
      case 'F':
        // Format EEPROM then re-read config
        formatEeprom();
      case 'l':
        // Read and list config
        readAndProcessConfig();
        break;
      case 'u':
        // Update a pin to trigger some keystrokes
        readPinUpdate();
        break;        
      case '\n':
      case '\r':
      case ' ':
        // Ignore whitespace
        break;
      default:
        Serial.print(F("Unknown command '"));
        Serial.print(inChar);
        Serial.println("'");
    }
  }
}
