Firmware and a sample sketch to allow an Arduino Uno R3 to be 
used as a USB Keyboard with keys being sent from the sketch. 

Includes firmware for the atmega16u2 (USB to Serial chip) on the
Uno R3 board so that it can be *both* a normal Arduino serial 
connection used for uploading sketches and input/output while
also transparently interpreting special embedded messages from 
the sketch that cause keycodes to be sent as keyboard messages. 
