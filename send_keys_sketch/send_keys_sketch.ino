/*
  Demonstrate using the the atmega16u2-usbtoserial-and-keyboard firmware 
  on the atmega16u2 in the Arduino Uno R3 to both have a serial console 
  that can be used in sketches and to load new sketches from the Arduino 
  IDE as well as sending keyboard codes via the USB Keyboard HID 
*/

char inChar;        // character received from serial port
int counter = 0;  

int buttonPin = 7;  // pin that will trigger a keyboard event when sent low
int buttonLast = 1; // last state of the button used to trigger on transition

/* Keyboard messages are inline with the serial connection, a 4 byte 
 * sentinel followed by 1 byte of modifiers, 2 bytes of keys then 
 * 1 byte of checksum
 */
byte message[] = {0xC0, 0x6C, 0x57, 0xCC, 0x00, 0x00, 0x00, 0x00};

/* Defines used to send various HID keystrokes */
#define HID_KEYBOARD_MODIFIER_LEFTGUI                     (1 << 3)
#define HID_KEYBOARD_MODIFIER_LEFTSHIFT                   (1 << 1)
#define HID_KEYBOARD_SC_LEFT_GUI                          0xE3
#define HID_KEYBOARD_SC_LEFT_SHIFT                        0xE1
#define HID_KEYBOARD_SC_A                                 0x04
#define HID_KEYBOARD_SC_0_AND_CLOSING_PARENTHESIS         0x27
#define HID_KEYBOARD_SC_1_AND_EXCLAMATION                 0x1E
#define HID_KEYBOARD_SC_SPACE                             0x2C

void setup() {
  Serial.begin(9600);                 // initialize serial port and set baud rate to 9600

  pinMode(buttonPin, INPUT_PULLUP);

  delay(100);
  Serial.println("Arduino keyboard sender ready");
}

void sendString(unsigned char *str) {
  while (unsigned char p = *str++) {
    if (p >= 'A' && p <= 'Z')
      sendKey(HID_KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEYBOARD_SC_A + (p - 'A'), 0x0);
    else if (p >= 'a' && p <= 'z') 
      sendKey(0x0, HID_KEYBOARD_SC_A + (p - 'a'), 0x0);
    else if (p == '0')
      sendKey(0x0, HID_KEYBOARD_SC_0_AND_CLOSING_PARENTHESIS, 0x0);
    else if (p >= '1' && p <= '9')
      sendKey(0x0, HID_KEYBOARD_SC_1_AND_EXCLAMATION + (p - '1'), 0x0);
    else if (p == ' ')
      sendKey(0x0, HID_KEYBOARD_SC_SPACE, 0x0);
    else 
      sendKey(0x0, HID_KEYBOARD_SC_A, 0x0);    
  }
}

void sendKey(uint8_t modifier, uint8_t key, uint8_t key2) {
  message[4] = modifier;
  message[5] = key;
  message[6] = key2;
  message[7] = message[0] ^ message[1] ^ message[2] ^ message[3] ^ 
               message[4] ^ message[5] ^ message[6];
  Serial.write(message, sizeof(message));
}

void loop() {  
  uint8_t buttonNew = digitalRead(buttonPin);
  if (buttonNew != buttonLast) {
    
    if (buttonNew == 0) {
      sendString("Hello");  
    }

    if (serialConsoleConnected) {
      Serial.print("button:");   
      Serial.println(buttonNew);    
    }
 
    buttonLast = buttonNew;
  }
      
  delay(50);                // wait 3000ms to avoid cycling too fast
  counter++;                  // variable "counter" increases 1
}

void serialEvent() {
  if (Serial.available()) {         // judge whether the data has been received
    inChar = Serial.read();         // read one character

    Serial.print("UNO received:");  // print the string "UNO received:"
    Serial.println(inChar);         // print the received character

    if (inChar == 's') {
      // Send a keystroke, the actual key data should never 
      // appear in the console since it will become a HID key
      Serial.print("key:");       // print the string "key:"
      sendKey(0x00, 0x1d, 0);
      Serial.println(""); 
    } else if (inChar == 'i') {
      // Send an incomplete keystroke to show it doesn't block
      Serial.print("incomplete: ");       // print the string "key:"
      Serial.write(message, sizeof(message) - 1);
      delay(500);
      Serial.println(""); 
      Serial.println("done");
    }
  }
}
