#ifndef keyboard_hid_data_h
#define keyboard_hid_data_h

extern byte keyboardMessage[];
void sendString(unsigned char *str);
uint8_t asciiToKey(unsigned char p, uint8_t *modifier);
void sendKey(uint8_t modifier, uint8_t key, uint8_t key2);

#endif
