// vim: noexpandtab:ts=8:sw=8:sts&
/** \file
 * 
 *  This is firmware for the atmega16u2 that is both a USB to Serial 
 *  adaptor that can drive the USB to UART chip in the Arduino Uno R3 in 
 *  a manner compatible with the Arduino IDE as well as a Keyboard HID 
 *  device that can transparently convert events embedded in the serial 
 *  output stream in to HID reports to the host. It can be used 
 *  as a firmware compatible with the Arduino programming environment 
 *  for the U3 while also letting Arduino programs send virtual 
 *  keyboard events. 
 *  
 *  To communicate serially we expect DTR to be asserted by the USB 
 *  host. In this mode we first reset the atmega328p by switching 
 *  Pin7 on PORTD from high to low. We then copy all incoming/outgoing
 *  bytes in both directions (except for keyboard events which are 
 *  transparently intercepted). If USB host does not have DTR asserted
 *  and we receive serial bytes (except embedded keyboard events), we 
 *  drop them
 *
 *  It's based on the VirtualSerial demo from the LUFA library by Dean 
 *  Camera, the USBtoSerial project from LUFA and the 
 *  Arduino-usbserial.c file from the ArduinoCore-avr source code 
 *  (which doesn't have a copyright attribution except the one for LUFA / 
 *  Dean Camera)
 */

/*
             LUFA Library
     Copyright (C) Dean Camera, 2019.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2019  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaims all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

#include "VirtualSerialAndKeyboard.h"

/** Cheat function for the RingBuffer implementation in LUFA that lets you peek
 *  beyond the first character in the buffer */
static inline uint8_t RingBuffer_Peek_Ahead(RingBuffer_t* const Buffer, uint8_t ahead) ATTR_WARN_UNUSED_RESULT ATTR_NON_NULL_PTR_ARG(1);
static inline uint8_t RingBuffer_Peek_Ahead(RingBuffer_t* const Buffer, uint8_t ahead)
{
	return *(Buffer->Start + (((Buffer->Out - Buffer->Start) + ahead) % Buffer->Size));
}

#define USBSerialConnected() (!(AVR_RESET_LINE_PORT & AVR_RESET_LINE_MASK))

#define BUFFER_SIZE 128
#define BUFFER_ALMOST_FULL 96

/** Circular buffer to hold data from the host before it is sent to the device via the serial port. */
static RingBuffer_t USBtoUSART_Buffer;

/** Underlying data buffer for \ref USBtoUSART_Buffer, where the stored bytes are located. */
static uint8_t      USBtoUSART_Buffer_Data[BUFFER_SIZE];

/** Circular buffer to hold data from the serial port before it is sent to the host. */
static RingBuffer_t USARTtoUSB_Buffer;

/** Underlying data buffer for \ref USARTtoUSB_Buffer, where the stored bytes are located. */
static uint8_t      USARTtoUSB_Buffer_Data[BUFFER_SIZE];

/** Pulse generation counters to keep track of the number of milliseconds remaining for each pulse type */
volatile struct
{
	uint8_t TxLEDPulse; /**< Ticks remaining for data Tx LED pulse */
	uint8_t RxLEDPulse; /**< Ticks remaining for data Rx LED pulse */
} PulseMSRemaining;

#define TX_RX_LED_PULSE_TICKS 3

/** Toggle the RX LED periodically after X USB message processing loops. Used to show that USB messages
* are still being processed */
#define USB_TASK_FLASH_RX_LED 20000 

// #define PULSE

/** Circular buffer to hold data to be sent from the Keyboard HID to the host */
static RingBuffer_t Keyboard_Buffer;

/** Underlying data buffer for \ref Keyboard_Buffer, where the stored bytes are located. */
static uint8_t      Keyboard_Buffer_Data[30];

/** Search the incoming USART data for keyboard events to be sent via the Keyboard HID */
static uint8_t      Keyboard_Send_Detector_Matched = 0;
static uint8_t      Keyboard_Send_Detector_Checksum = 0;

/** Keyboard data is sent inline in the USART to USB stream as 4 bytes of sentinel, 
    1 byte of modifier, 2 bytes of keystroke, 1 checksum. At least one byte of the sequence
    must be received every timer0 overflow (0.004096 seconds) otherwise the data will timeout 
    and be flushed */
static uint8_t      Sentinel[] = { 0xC0u, 0x6Cu, 0x57u, 0xCCu };

/** LUFA CDC Class driver interface configuration and state information. This structure is
 *  passed to all CDC Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface =
	{
		.Config =
			{
				.ControlInterfaceNumber   = INTERFACE_ID_CDC_CCI,
				.DataINEndpoint           =
					{
						.Address          = CDC_TX_EPADDR,
						.Size             = CDC_TXRX_EPSIZE,
						.Banks            = 1,
					},
				.DataOUTEndpoint =
					{
						.Address          = CDC_RX_EPADDR,
						.Size             = CDC_TXRX_EPSIZE,
						.Banks            = 1,
					},
				.NotificationEndpoint =
					{
						.Address          = CDC_NOTIFICATION_EPADDR,
						.Size             = CDC_NOTIFICATION_EPSIZE,
						.Banks            = 1,
					},
			},
	};

/** Buffer to hold the previously generated Keyboard HID report, for comparison purposes inside the HID class driver. */
static uint8_t PrevKeyboardHIDReportBuffer[sizeof(USB_KeyboardReport_Data_t)];

/** LUFA HID Class driver interface configuration and state information. This structure is
 *  passed to all HID Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another. This is for the keyboard HID
 *  interface within the device.
 */
USB_ClassInfo_HID_Device_t Keyboard_HID_Interface =
	{
		.Config =
			{
				.InterfaceNumber              = INTERFACE_ID_Keyboard,
				.ReportINEndpoint             =
					{
						.Address              = KEYBOARD_IN_EPADDR,
						.Size                 = HID_EPSIZE,
						.Banks                = 1,
					},
				.PrevReportINBuffer           = PrevKeyboardHIDReportBuffer,
				.PrevReportINBufferSize       = sizeof(PrevKeyboardHIDReportBuffer),
			},
	};

/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
	LEDs_SetAllLEDs(0);
	SetupHardware();

	RingBuffer_InitBuffer(&USBtoUSART_Buffer, USBtoUSART_Buffer_Data, sizeof(USBtoUSART_Buffer_Data));
	RingBuffer_InitBuffer(&USARTtoUSB_Buffer, USARTtoUSB_Buffer_Data, sizeof(USARTtoUSB_Buffer_Data));
	RingBuffer_InitBuffer(&Keyboard_Buffer,   Keyboard_Buffer_Data,   sizeof(Keyboard_Buffer_Data));

	GlobalInterruptEnable();

#ifdef USB_TASK_FLASH_RX_LED
	uint16_t UsbProcessed = 0;
#endif

	for (;;)
	{
		/* Only try to read in bytes from the CDC interface if the transmit buffer is not full */
		if (!(RingBuffer_IsFull(&USBtoUSART_Buffer)))
		{
			int16_t ReceivedByte = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);

			/* Store received byte into the USART transmit buffer */
			if (!(ReceivedByte < 0))
				RingBuffer_Insert(&USBtoUSART_Buffer, ReceivedByte);
		}

		uint16_t BufferCount = RingBuffer_GetCount(&USARTtoUSB_Buffer);

		if (CheckForKeyboardEvent(&BufferCount)) {
			/* If we added at least one more byte to our processing of 
			* keyboard events embedded in the USART to USB stream, 
			* reset the timer and come back around */
			TIFR0 |= (1 << TOV0);
			TCNT0 = 0; 
		}
		else if (((TIFR0 & (1 << TOV0)) || (BufferCount > BUFFER_ALMOST_FULL)))
		{
			/* The UART receive buffer flush timer has expired (by overflow) or the buffer is nearly full,
			so we will flush data to the USB side, even if we have a partial keyboard event match */

			/* Reset buffer flush timer */
			TIFR0 |= (1 << TOV0);
			TCNT0 = 0; 

#ifdef PULSE
			if (BufferCount) {
				LEDs_TurnOnLEDs(LEDMASK_TX);
				PulseMSRemaining.TxLEDPulse = TX_RX_LED_PULSE_TICKS;
			}
#endif

			/* Read bytes from the USART receive buffer into the USB IN endpoint */

			/* If no usb serial connection is active (i.e. has DTR asserted), drain any bytes. Otherwise 
			* check if a packet is already enqueued to the host - if so, we shouldn't try to send more 
			* data until it completes as there is a chance nothing is listening and a lengthy timeout 
			* could occur */
			if (!USBSerialConnected()) {
				while (BufferCount--) {
					RingBuffer_Remove(&USARTtoUSB_Buffer);
					Keyboard_Send_Detector_Matched = 0;

					/* If the next bytes look like the start of a keyboard event stop */
					if (CheckForKeyboardEvent(&BufferCount))
					  break;
				}
			} else if (Endpoint_IsINReady()) {
				/* Never send more than one bank size less one byte to the host at a time, so that we don't block
				* while a Zero Length Packet (ZLP) to terminate the transfer is sent if the host isn't listening */
				uint16_t BytesToSend = MIN(BufferCount, (CDC_TXRX_EPSIZE - 1));

				/* Read bytes from the USART receive buffer into the USB IN endpoint */
				while (BytesToSend--)
				{
					/* Try to send the next byte of data to the host, abort if there is an error without dequeuing */
					if (CDC_Device_SendByte(&VirtualSerial_CDC_Interface,
						RingBuffer_Peek(&USARTtoUSB_Buffer)) != ENDPOINT_READYWAIT_NoError)
					{
						break;
					}

					/* Dequeue the already sent byte from the buffer now we have confirmed that no transmission error occurred */
					// LEDs_TurnOnLEDs(LEDMASK_RX);
					RingBuffer_Remove(&USARTtoUSB_Buffer);
					Keyboard_Send_Detector_Matched = 0;

					/* If the next bytes look like the start of a keyboard event stop */
					BufferCount--;
					if (CheckForKeyboardEvent(&BufferCount))
					  break;
				}
			}

#ifdef PULSE
			/* Turn off TX LED(s) once the TX pulse period has elapsed */
			if (PulseMSRemaining.TxLEDPulse && !(--PulseMSRemaining.TxLEDPulse))
			  LEDs_TurnOffLEDs(LEDMASK_TX);

			/* Turn off RX LED(s) once the RX pulse period has elapsed */
			if (PulseMSRemaining.RxLEDPulse && !(--PulseMSRemaining.RxLEDPulse))
			  LEDs_TurnOffLEDs(LEDMASK_RX);
#endif
		}

		/* Load the next byte from the USART transmit buffer into the USART if transmit buffer space is available */
		if (Serial_IsSendReady() && !(RingBuffer_IsEmpty(&USBtoUSART_Buffer))) {
			Serial_SendByte(RingBuffer_Remove(&USBtoUSART_Buffer));
#ifdef PULSE
			LEDs_TurnOnLEDs(LEDMASK_RX);
			PulseMSRemaining.RxLEDPulse = TX_RX_LED_PULSE_TICKS;
#endif
		}

		CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
		HID_Device_USBTask(&Keyboard_HID_Interface);
		USB_USBTask();

#ifdef USB_TASK_FLASH_RX_LED
		UsbProcessed++;
		if (UsbProcessed >= USB_TASK_FLASH_RX_LED) {
			LEDs_ToggleLEDs(LEDMASK_RX);
			UsbProcessed = 0;
		}
#endif
	}
}

/** See if there is a partial or complete keyboard send event in the 
 *  USART input stream. A keyboard event is preceded by a 4 byte 
 *  sentinel then 1 byte of modifier, 2 bytes of keys then 1 byte 
 *  of XOR checksum. 
 * 
 *  Returns true if one or more _new_ bytes have been found that
 *  match an incomplete keyboard event. Returns false if no new 
 *  bytes have been found that match
 */
uint8_t CheckForKeyboardEvent(uint16_t *BufferCount) {
	// Indicator that one or more new bytes have been found
	// that match an incomplete keyboard event
	uint8_t match = 0;
	// uint8_t SendKey = HID_KEYBOARD_SC_C;

	while (*BufferCount > Keyboard_Send_Detector_Matched) {
		uint8_t CheckByte = RingBuffer_Peek_Ahead(&USARTtoUSB_Buffer, Keyboard_Send_Detector_Matched);
		switch (Keyboard_Send_Detector_Matched) {
			case 0:
				Keyboard_Send_Detector_Checksum = 0;
			case 1:
			case 2:
			case 3:
				Keyboard_Send_Detector_Checksum ^= CheckByte;
				match = CheckByte == Sentinel[Keyboard_Send_Detector_Matched];
				break;
			case 4:
			case 5:
			case 6:
				Keyboard_Send_Detector_Checksum ^= CheckByte;
				match = 1;
				break;
			case 7:
				match = (CheckByte ^ Keyboard_Send_Detector_Checksum) == 0;
			break;
		}

		if (match) {
			if (++Keyboard_Send_Detector_Matched == 8) {
				/* Got a whole keyboard event, send it */
				for (uint8_t i = 0; i < 8; i++) {
					(*BufferCount)--;
					uint8_t ProcessChar = RingBuffer_Remove(&USARTtoUSB_Buffer);
					if (i >= 4 && i <= 6) {
						if (!RingBuffer_IsFull(&Keyboard_Buffer))
							RingBuffer_Insert(&Keyboard_Buffer, ProcessChar);
					}
				}

				// LEDs_TurnOnLEDs(LEDMASK_RX);
				Keyboard_Send_Detector_Matched = 0;
				// Swing back round and see if we match more 
				match = 0;
			}
		} else {
			Keyboard_Send_Detector_Matched = 0;
			break;
		}
	}

	return match;
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
#if (ARCH == ARCH_AVR8)
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);
#endif

	/* Hardware Initialization */

	/* Listen for Serial communication immediately */
	Serial_Init(9600, false);

	LEDs_Init();
	USB_Init();

	/* Use TIMER1 for 'I'm still alive messages'. 
	   Set with pre-scale of 1024 so it ticks every 16000000/256 seconds.
	   Since the timer is 16 bits wide, it will overflow 
	   every 1/(16000000/256)*(2^16) = 1.048576 seconds or so */
	TCNT1 = 0;
	TIFR1 |= (1<<OCF1A) ; // Clear timer1 overflow flag and prepare for next overflow
	// TCCR1B = (1<<CS11);  // (Clock / 8)   = 1/(16000000/8)*(2^16)   = 0.032768 seconds overflow
	// TCCR1B = (1<<CS10);  // (Clock)       = 1/(16000000/1)*(2^16)   = 0.004096 seconds overflow
	TCCR1B = (1<<CS12);  // (Clock / 256) = 1/(16000000/256)*(2^16) = 1.048576 seconds overflow

	/* Enable the TIMER1 overflow interrupt so we get an interrupt on each overflow of TIMER1 */
	TIMSK1 = (1 << TOIE1); 

	/* Use TIMER0 for a send buffer timeout. 
	   Set with pre-scale of Clk / 256 so it ticks every 16000000/256 seconds, 0.000016 seconds. 
	   Since the timer counter is 8 bits it will overflow every 0.000016 * 256 seconds = 0.004096 seconds */
	TCCR0B = (1 << CS01) | (1 << CS00); // 1/(16000000/64)*(2^8) = 0.001024 seconds overflow
	TCCR0B = (1 << CS02); // 1/(16000000/256)*(2^8) = 0.004096 seconds

	/* Pull target /RESET line high */
	AVR_RESET_LINE_PORT |= AVR_RESET_LINE_MASK;
	AVR_RESET_LINE_DDR  |= AVR_RESET_LINE_MASK;

	/* Enable serial interrupts immediately so we will listen for incoming data. 
           The original USBtoSerial code does not do this so incoming serial data 
           is ignored until the USB CDC line is configured by the host. */
        UCSR1B |= (1 << RXCIE1);
}

/** ISR triggered every second or so to send some data to the 
 *  serial port to show we're still alive */
ISR(TIMER1_OVF_vect, ISR_BLOCK) 
{
	/* Reset count and overflow flag */
	TCNT1 = 0;
	TIFR1 |= (1<<OCF1A) ; // Clear timer1 overflow flag 

	/* Toggle the LED to show we're still here */
	// LEDs_ToggleLEDs(LEDMASK_TX);

	/* Send a key to the keyboard HID */
	// RingBuffer_Insert(&Keyboard_Buffer, HID_KEYBOARD_SC_B);
}

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
	// LEDs_SetAllLEDs(LEDMASK_USB_ENUMERATING);
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
	// LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	ConfigSuccess &= CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);
	ConfigSuccess &= HID_Device_ConfigureEndpoints(&Keyboard_HID_Interface);

	// Fire the USB Start of Frame event once every millisecond in sync with the USB bus
	USB_Device_EnableSOFEvents();

	// LEDs_SetAllLEDs(LEDMASK_TX|LEDMASK_RX);
	// LEDs_SetAllLEDs(ConfigSuccess ? LEDMASK_USB_READY : LEDMASK_USB_ERROR);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
	HID_Device_ProcessControlRequest(&Keyboard_HID_Interface);
}

/** Event handler for the USB device Start Of Frame event. */
void EVENT_USB_Device_StartOfFrame(void)
{
	HID_Device_MillisecondElapsed(&Keyboard_HID_Interface);
}

/** ISR to manage the reception of data from the serial port, placing received bytes into a circular buffer
 *  for later transmission to the host.
 */
ISR(USART1_RX_vect, ISR_BLOCK)
{
	uint8_t ReceivedByte = UDR1;

	// Indicate that we've received some data on the serial port 
	LEDs_TurnOnLEDs(LEDMASK_RX);

	if (((USB_DeviceState == DEVICE_STATE_Configured)) && !(RingBuffer_IsFull(&USARTtoUSB_Buffer))) {
		RingBuffer_Insert(&USARTtoUSB_Buffer, ReceivedByte);
	}
}

/** CDC class driver callback function the processing of changes to the virtual
 *  control lines sent from the host..
 *
 *  \param[in] CDCInterfaceInfo  Pointer to the CDC class interface configuration structure being referenced
 */
void EVENT_CDC_Device_ControLineStateChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo)
{
	/* Check state of Data Terminal Ready (DTR) 
	   When the Arduino IDE serial monitor / avrdude etc connects it will assert DTR and keep 
	   it high. When it disconnects it will go low. */
	bool CurrentDTRState = (CDCInterfaceInfo->State.ControlLineStates.HostToDevice & CDC_CONTROL_LINE_OUT_DTR);

	/* Pin7 on PORTD is connected to the reset pin on the atmega328p, it will reset 
	   the AVR when it transitions from high to low  for longer than the minimum pulse length 
	   (https://www.arduino.cc/en/uploads/Main/Arduino_Uno_Rev3-schematic.pdf) */
	if (CurrentDTRState) {
		/* Send reset line low, will cause the atmega328p to reset */
		AVR_RESET_LINE_PORT &= ~AVR_RESET_LINE_MASK;
		LEDs_TurnOnLEDs(LEDMASK_TX); // Show we are ready to communicate with the atmega328p
	} else {
		/* Take reset line high */
		AVR_RESET_LINE_PORT |= AVR_RESET_LINE_MASK;
		LEDs_TurnOffLEDs(LEDMASK_TX); // Show we are not connected right now 
	}
}

/** Event handler for the CDC Class driver Line Encoding Changed event.
 *
 *  \param[in] CDCInterfaceInfo  Pointer to the CDC class interface configuration structure being referenced
 */
void EVENT_CDC_Device_LineEncodingChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo)
{
	uint8_t ConfigMask = 0;

	switch (CDCInterfaceInfo->State.LineEncoding.ParityType)
	{
		case CDC_PARITY_Odd:
			ConfigMask = ((1 << UPM11) | (1 << UPM10));
			break;
		case CDC_PARITY_Even:
			ConfigMask = (1 << UPM11);
			break;
	}

	if (CDCInterfaceInfo->State.LineEncoding.CharFormat == CDC_LINEENCODING_TwoStopBits)
	  ConfigMask |= (1 << USBS1);

	switch (CDCInterfaceInfo->State.LineEncoding.DataBits)
	{
		case 6:
			ConfigMask |= (1 << UCSZ10);
			break;
		case 7:
			ConfigMask |= (1 << UCSZ11);
			break;
		case 8:
			ConfigMask |= ((1 << UCSZ11) | (1 << UCSZ10));
			break;
	}

	/* Keep the TX line held high (idle) while the USART is reconfigured */
	PORTD |= (1 << 3);

	/* Must turn off USART before reconfiguring it, otherwise incorrect operation may occur */
	UCSR1B = 0;
	UCSR1A = 0;
	UCSR1C = 0;

	/* Special case 57600 baud for compatibility with the ATmega328 bootloader. */	
	UBRR1  = (CDCInterfaceInfo->State.LineEncoding.BaudRateBPS == 57600)
			 ? SERIAL_UBBRVAL(CDCInterfaceInfo->State.LineEncoding.BaudRateBPS)
			 : SERIAL_2X_UBBRVAL(CDCInterfaceInfo->State.LineEncoding.BaudRateBPS);	

	UCSR1C = ConfigMask;
	UCSR1A = (CDCInterfaceInfo->State.LineEncoding.BaudRateBPS == 57600) ? 0 : (1 << U2X1);

	/* Switch on send and receive (with interrupts) */
	UCSR1B = ((1 << RXCIE1) | (1 << TXEN1) | (1 << RXEN1));

	/* Release the TX line after the USART has been reconfigured */
	PORTD &= ~(1 << 3);
}

/** HID class driver callback function for the creation of HID reports to the host.
 *
 *  \param[in]     HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
 *  \param[in,out] ReportID    Report ID requested by the host if non-zero, otherwise callback should set to the generated report ID
 *  \param[in]     ReportType  Type of the report to create, either HID_REPORT_ITEM_In or HID_REPORT_ITEM_Feature
 *  \param[out]    ReportData  Pointer to a buffer where the created report should be stored
 *  \param[out]    ReportSize  Number of bytes written in the report (or zero if no report is to be sent)
 *
 *  \return Boolean \c true to force the sending of the report, \c false to let the library determine if it needs to be sent
 */
bool CALLBACK_HID_Device_CreateHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
					 uint8_t* const ReportID,
					 const uint8_t ReportType,
					 void* ReportData,
					 uint16_t* const ReportSize)
{
	USB_KeyboardReport_Data_t* KeyboardReport = (USB_KeyboardReport_Data_t*)ReportData;

	if (RingBuffer_GetCount(&Keyboard_Buffer) >= 3) {
		KeyboardReport->Modifier   = RingBuffer_Remove(&Keyboard_Buffer);
		KeyboardReport->KeyCode[0] = RingBuffer_Remove(&Keyboard_Buffer);
		KeyboardReport->KeyCode[1] = RingBuffer_Remove(&Keyboard_Buffer);
	}

	*ReportSize = sizeof(USB_KeyboardReport_Data_t);
	return false;
}

/** HID class driver callback function for the processing of HID reports from the host.
 *
 *  \param[in] HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
 *  \param[in] ReportID    Report ID of the received report from the host
 *  \param[in] ReportType  The type of report that the host has sent, either HID_REPORT_ITEM_Out or HID_REPORT_ITEM_Feature
 *  \param[in] ReportData  Pointer to a buffer where the received report has been stored
 *  \param[in] ReportSize  Size in bytes of the received HID report
 */
void CALLBACK_HID_Device_ProcessHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
					  const uint8_t ReportID,
					  const uint8_t ReportType,
					  const void* ReportData,
					  const uint16_t ReportSize)
{
#if 0
	uint8_t  LEDMask   = LEDS_NO_LEDS;
	uint8_t* LEDReport = (uint8_t*)ReportData;

	if (*LEDReport & HID_KEYBOARD_LED_NUMLOCK)
	  LEDMask |= LEDS_LED1;

	if (*LEDReport & HID_KEYBOARD_LED_CAPSLOCK)
	  LEDMask |= LEDS_LED3;

	if (*LEDReport & HID_KEYBOARD_LED_SCROLLLOCK)
	  LEDMask |= LEDS_LED4;

	LEDs_SetAllLEDs(LEDMask);
#endif
}

