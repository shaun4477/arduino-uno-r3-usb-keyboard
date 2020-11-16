#include <Arduino.h>
#include "keyboard_hid_data.h"
#include "keyboard_hid_defines.h"

/* Keyboard keyboardMessages are inline with the serial connection, a 4 byte 
 * sentinel followed by 1 byte of modifiers, 2 bytes of keys then 
 * 1 byte of checksum
 */
byte keyboardMessage[] = {0xC0, 0x6C, 0x57, 0xCC, 0x00, 0x00, 0x00, 0x00};

void sendKey(uint8_t modifier, uint8_t key, uint8_t key2) {
  keyboardMessage[4] = modifier;
  keyboardMessage[5] = key;
  keyboardMessage[6] = key2;
  keyboardMessage[7] = keyboardMessage[0] ^ keyboardMessage[1] ^ keyboardMessage[2] ^ keyboardMessage[3] ^ 
                       keyboardMessage[4] ^ keyboardMessage[5] ^ keyboardMessage[6];
  Serial.write(keyboardMessage, sizeof(keyboardMessage));
}

/* Send an ASCII string to the Keyboard USB device */
void sendString(unsigned char *str) {
  while (unsigned char p = *str++) {
    uint8_t modifier;
    uint8_t key = asciiToKey(p, &modifier);
    sendKey(modifier, key, 0x0);
  }
}

uint8_t asciiToKey(unsigned char p, uint8_t *modifier) {
  uint8_t discardModifier; 
  if (!modifier)
    modifier = & discardModifier; 
  else
    *modifier = 0;
    
  if (p >= 'A' && p <= 'Z') {
    *modifier = HID_KEYBOARD_MODIFIER_LEFTSHIFT;
    return HID_KEYBOARD_SC_A + (p - 'A');
  } else if (p >= '#' && p <= '%') {
    *modifier = HID_KEYBOARD_MODIFIER_LEFTSHIFT;
    return HID_KEYBOARD_SC_3_AND_HASHMARK + (p - '#');
  } else if (p == '^') {
    *modifier = HID_KEYBOARD_MODIFIER_LEFTSHIFT;
    return HID_KEYBOARD_SC_6_AND_CARET;
  } else if (p == '&') {
    *modifier = HID_KEYBOARD_MODIFIER_LEFTSHIFT;
    return HID_KEYBOARD_SC_7_AND_AMPERSAND;
  } else if (p == '*') {
    *modifier = HID_KEYBOARD_MODIFIER_LEFTSHIFT;
    return HID_KEYBOARD_SC_8_AND_ASTERISK;
  } else if (p == '!') {
    *modifier = HID_KEYBOARD_MODIFIER_LEFTSHIFT;
    return HID_KEYBOARD_SC_1_AND_EXCLAMATION;
  } else if (p == '@') {
    *modifier = HID_KEYBOARD_MODIFIER_LEFTSHIFT;
    return HID_KEYBOARD_SC_2_AND_AT;
  } else if (p >= '(' && p <= ')') {
    *modifier = HID_KEYBOARD_MODIFIER_LEFTSHIFT;
    return HID_KEYBOARD_SC_9_AND_OPENING_PARENTHESIS + (p - '(');
  } else if (p >= 'a' && p <= 'z') 
    return HID_KEYBOARD_SC_A + (p - 'a');
  else if (p == '0')
    return HID_KEYBOARD_SC_0_AND_CLOSING_PARENTHESIS;
  else if (p >= '1' && p <= '9')
    return HID_KEYBOARD_SC_1_AND_EXCLAMATION + (p - '1');
  else if (p == '\n')
    return HID_KEYBOARD_SC_ENTER;
  else if (p == '\x1b')
    return HID_KEYBOARD_SC_ESCAPE;
  else if (p == '\x08')
    return HID_KEYBOARD_SC_BACKSPACE;
  else if (p == '\t')
    return HID_KEYBOARD_SC_TAB;
  else if (p == ' ')
    return HID_KEYBOARD_SC_SPACE;
  else if (p == '-')
    return HID_KEYBOARD_SC_MINUS_AND_UNDERSCORE;
  else if (p == '_') {
    *modifier = HID_KEYBOARD_MODIFIER_LEFTSHIFT;
    return HID_KEYBOARD_SC_MINUS_AND_UNDERSCORE;
  } else if (p == '=')
    return HID_KEYBOARD_SC_EQUAL_AND_PLUS;
  else if (p == '+') {
    *modifier = HID_KEYBOARD_MODIFIER_LEFTSHIFT;
    return HID_KEYBOARD_SC_EQUAL_AND_PLUS;
  } else if (p == '[')
    return HID_KEYBOARD_SC_OPENING_BRACKET_AND_OPENING_BRACE;
  else if (p == ']')
    return HID_KEYBOARD_SC_CLOSING_BRACKET_AND_CLOSING_BRACE;
  else if (p == '{') {
    *modifier = HID_KEYBOARD_MODIFIER_LEFTSHIFT;
    return HID_KEYBOARD_SC_OPENING_BRACKET_AND_OPENING_BRACE;
  } else if (p == '}') {
    *modifier = HID_KEYBOARD_MODIFIER_LEFTSHIFT;
    return HID_KEYBOARD_SC_CLOSING_BRACKET_AND_CLOSING_BRACE;
  } else if (p == '\\')
    return HID_KEYBOARD_SC_BACKSLASH_AND_PIPE;
  else if (p == '|') {
    *modifier = HID_KEYBOARD_MODIFIER_LEFTSHIFT;
    return HID_KEYBOARD_SC_BACKSLASH_AND_PIPE;   
  } else if (p == '`')
    return HID_KEYBOARD_SC_GRAVE_ACCENT_AND_TILDE;
  else if (p == '~') {
    *modifier = HID_KEYBOARD_MODIFIER_LEFTSHIFT;
    return HID_KEYBOARD_SC_GRAVE_ACCENT_AND_TILDE;
  } else if (p == ';')
    return HID_KEYBOARD_SC_SEMICOLON_AND_COLON;
  else if (p == ':') {
    *modifier = HID_KEYBOARD_MODIFIER_LEFTSHIFT;
    return HID_KEYBOARD_SC_SEMICOLON_AND_COLON;
  } else if (p == '\'')
    return HID_KEYBOARD_SC_APOSTROPHE_AND_QUOTE;
  else if (p == '"') {
    *modifier = HID_KEYBOARD_MODIFIER_LEFTSHIFT;
    return HID_KEYBOARD_SC_APOSTROPHE_AND_QUOTE;
  } else if (p == ',')
    return HID_KEYBOARD_SC_COMMA_AND_LESS_THAN_SIGN;
  else if (p == '<') {
    *modifier = HID_KEYBOARD_MODIFIER_LEFTSHIFT;
    return HID_KEYBOARD_SC_COMMA_AND_LESS_THAN_SIGN;
  } else if (p == '.')
    return HID_KEYBOARD_SC_DOT_AND_GREATER_THAN_SIGN;
  else if (p == '>') {
    *modifier = HID_KEYBOARD_MODIFIER_LEFTSHIFT;
    return HID_KEYBOARD_SC_DOT_AND_GREATER_THAN_SIGN;
  } else if (p == '/')
    return HID_KEYBOARD_SC_SLASH_AND_QUESTION_MARK;
  else if (p == '?') {
    *modifier = HID_KEYBOARD_MODIFIER_LEFTSHIFT;
    return HID_KEYBOARD_SC_SLASH_AND_QUESTION_MARK;
  } else 
    return HID_KEYBOARD_SC_A;
}
