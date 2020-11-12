# USB Keyboard and USB to Serial firmware for the atmega16u2

## Description

This is firmware to run on the atmega16u2 that is used as the USB to Serial 
adaptor in the Arduino Uno R3. The original firmware is pretty simple:

- Create a set of USB communications device class (CDC) endpoints that 
represent the equivalent of a serial connection. On Mac these endpoints 
cause a /dev/cu.usbmodem\* block device to be created that can be used 
to communicate to the device 
 
- Any time the USB serial connection is opened by the USB host and the 
DTR pin is set high (which it always is by the Arduino IDE serial monitor or 
sketch upload) it sends a reset to the main microprocessor in the 
Arduino (the atmega328p). 

- It then simply copies bytes from the serial connection to the atmega328p
to the USB host and vica versa

This firmware does the above, but also presents a Keyboard HID endpoint
that represents a USB keyboard connection. To drive this keyboard it 
processes special sequences in the serial data from the atmega328p that 
represent data to be sent as Keyboard messages. 

- Keyboard data is sent inline as 4 bytes of sentinel ("\xC0\x6C\x57\xCC")
1 byte of modifier, 2 bytes of keystroke, 1 of checksum (XOR of other bytes)

- At least one byte of the sequence must be received every timer0 overflow 
(0.004096 seconds) otherwise the data will timeout and be flushed 

## Acknowledgements 

This code is based on example code from LUFA (http://www.lufa-lib.org) by 
Dean Camera and the arduino-usbserial code from the Arduino-avr source code

## Building 

You'll need LUFA (https://github.com/abcminiuser/lufa) version 170418. 

Simply run make with LUFA_PATH set:

`LUFA_PATH="$HOME/src/lufa/LUFA" make`

This will create VirtualSerialAndKeyboard.hex which is firmware ready to 
be loaded on to the chip

## Programming / Loading on the atmega16u2 in the Arduino Uno R3

To load on the atmega16u2

- Install dfu-programmer (on Mac with homebrew installed you can just use
`brew install dfu-programmer`)

- Reset the chip in to bootloader mode by shorting (with a wire)) the two
pins on the Arduino closest to the USB connector and the red reset button

- Erase the current firmware

`dfu-programmer atmega16u2 erase`

- Write the new firmware

`dfu-programmer atmega16u2 flash VirtualSerialAndKeyboard.hex`

- The new firmware is now ready, remove and replug the USB cable to 
reset the device and it should be ready to go 
