/*
 * Copyright (C) 2023  ES factory
 *
 * DigiCombo is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * DigiCombo is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with DigiCombo.  If not, see https://www.gnu.org/licenses/.
 */

#include "cmdline_defs.h"
#include "DigiComboKBM_C.h"
#include "usbconfig.h"
#include "usbdrv/usbdrv.h"
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <util/delay.h>
#include <stdint.h>

uint8_t lastReportId = 0;
uint8_t report_buffer[BUFFER_SIZE];

uint8_t idle_rate = 20 / 4;  // see HID1_11.pdf sect 7.2.4
uint8_t protocol_version = 0; // see HID1_11.pdf sect 7.2.6

void usbBegin() {
	cli();

    clock_prescale_set(clock_div_1);

	// fake a disconnect to force the computer to re-enumerate
	PORTB &= ~(_BV(USB_CFG_DMINUS_BIT) | _BV(USB_CFG_DPLUS_BIT));
	usbDeviceDisconnect();
	_delay_ms(250);
	usbDeviceConnect();

	// start the USB driver
	usbInit();
	sei();
}

void usbPollWrapper() {
	usbPoll();
}

void usbReportSend(uint8_t size) {
	while (!usbInterruptIsReady()) {
        usbPoll(); // this needs to be called at least once every 10 ms
        _delay_ms(10);
    }
    lastReportId = report_buffer[0];
    usbSetInterrupt((uint8_t*)report_buffer, size); // send
}

const PROGMEM char usbHidReportDescriptor[USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH] = {
  0x05, 0x01,					 // USAGE_PAGE (Generic Desktop)
  0x09, 0x02,					 // USAGE (Mouse)
  0xa1, 0x01,					 // COLLECTION (Application)
  0x85, REPORT_ID_MOUSE, 	     //  REPORT_ID (Touch)
  0x09, 0x01,					 //	 USAGE_PAGE (Pointer)
  0xa1, 0x00,					 //	 COLLECTION (Physical)
  0x05, 0x09,					 //	   USAGE_PAGE (Button)
  0x19, 0x01,					 //	   USAGE_MINIMUM (Button 1)
  0x29, 0x03,					 //	   USAGE_MAXIMUM (Button 3)
  0x15, 0x00,					 //	   LOGICAL_MINIMUM (0)
  0x25, 0x01,					 //	   LOGICAL_MAXIMUM (1)
  0x95, 0x03,					 //	   REPORT_COUNT (3)
  0x75, 0x01,					 //	   REPORT_SIZE (1)
  0x81, 0x02,					 //	   INPUT (Data,Var,Abs)
  0x95, 0x01,					 //	   REPORT_COUNT (1)
  0x75, 0x05,					 //	   REPORT_SIZE (5)
  0x81, 0x01,					 //	   Input(Cnst)
  0x05, 0x01,					 //	   USAGE_PAGE(Generic Desktop)
  0x09, 0x30,					 //	   USAGE(X)
  0x09, 0x31,					 //	   USAGE(Y)
  0x15, 0x81,					 //	   LOGICAL_MINIMUM (-127)
  0x25, 0x7f,					 //	   LOGICAL_MAXIMUM (127)
  0x75, 0x08,					 //	   REPORT_SIZE (8)
  0x95, 0x02,					 //	   REPORT_COUNT (2)
  0x81, 0x06,					 //	   INPUT (Data,Var,Rel)
  0x09, 0x38,					 //	   Usage (Wheel)
  0x95, 0x01,					 //	   Report Count (1),
  0x81, 0x06,					 //	   Input (Data, Variable, Relative)
  0xc0,							 //  END_COLLECTION
  0xc0,							 // END_COLLECTION
  ///////////////////////////////////////////////////////////////////////

  0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
  0x09, 0x06,                    // USAGE (Keyboard)
  0xa1, 0x01,                    // COLLECTION (Application)
  0x85, REPORT_ID_KEYBOARD,      //   REPORT_ID (Keyboard)
  0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
  0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)
  0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard Right GUI)
  0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
  0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
  0x75, 0x01,                    //   REPORT_SIZE (1)
  0x95, 0x08,                    //   REPORT_COUNT (8)
  0x81, 0x02,                    //   INPUT (Data,Var,Abs)
  0x95, MAX_HELD_KEYS,           //   REPORT_COUNT (Max simultaneous keystrokes)
  0x75, 0x08,                    //   REPORT_SIZE (8)
  0x25, 0x65,                    //   LOGICAL_MAXIMUM (101)
  0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event indicated))
  0x29, 0x65,                    //   USAGE_MAXIMUM (Keyboard Application)
  0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
  0xc0,                          // END_COLLECTION
  // with this declaration a data packet must be sent as:
  // byte 1   -> "modifier"  (SHIFT, ALT, CTRL, GUI)
  // byte 2   -> "key"       (Remained keys)
};

// see http://vusb.wikidot.com/driver-api
// constants are found in usbdrv.h
usbMsgLen_t usbFunctionSetup(uint8_t data[8]) {
	// see HID1_11.pdf sect 7.2 and http://vusb.wikidot.com/driver-api
	usbRequest_t *rq = (void *)data;

	if ((rq->bmRequestType & USBRQ_TYPE_MASK) != USBRQ_TYPE_CLASS)
		return 0; // ignore request if it's not a class specific request

	// see HID1_11.pdf sect 7.2
	switch (rq->bRequest) {
		case USBRQ_HID_GET_IDLE:
			usbMsgPtr = (usbMsgPtr_t) &idle_rate; // send data starting from this byte
			return 1; // send 1 byte
		case USBRQ_HID_SET_IDLE:
			idle_rate = rq->wValue.bytes[1]; // read in idle rate
			return 0; // send nothing
		case USBRQ_HID_GET_PROTOCOL:
			usbMsgPtr = (usbMsgPtr_t) &protocol_version; // send data starting from this byte
			return 1; // send 1 byte
		case USBRQ_HID_SET_PROTOCOL:
			protocol_version = rq->wValue.bytes[1];
			return 0; // send nothing
		case USBRQ_HID_GET_REPORT:
			usbMsgPtr = (usbMsgPtr_t) report_buffer; // send the report data
			// report_buffer[0] = rq->wValue.bytes[0];
			// report_buffer[1] = report_buffer[2] = report_buffer[3] = report_buffer[4] = report_buffer[5] = report_buffer[6] = report_buffer[7] = 0; // clear the report
			// determine the return data length based on which report ID was requested
			if (rq->wValue.bytes[0] == REPORT_ID_MOUSE)    return REPORT_SIZE_MOUSE;
			if (rq->wValue.bytes[0] == REPORT_ID_KEYBOARD) return REPORT_SIZE_KEYBOARD;
			return BUFFER_SIZE; // default
		default: // do not understand data, ignore
			return 0; // send nothing
	}
}

#if defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny25__)
/* ------------------------------------------------------------------------- */
/* ------------------------ Oscillator Calibration ------------------------- */
/* ------------------------------------------------------------------------- */
// section copied from EasyLogger
/* Calibrate the RC oscillator to 8.25 MHz. The core clock of 16.5 MHz is
 * derived from the 66 MHz peripheral clock by dividing. Our timing reference
 * is the Start Of Frame signal (a single SE0 bit) available immediately after
 * a USB RESET. We first do a binary search for the OSCCAL value and then
 * optimize this value with a neighboorhod search.
 * This algorithm may also be used to calibrate the RC oscillator directly to
 * 12 MHz (no PLL involved, can therefore be used on almost ALL AVRs), but this
 * is wide outside the spec for the OSCCAL value and the required precision for
 * the 12 MHz clock! Use the RC oscillator calibrated to 12 MHz for
 * experimental purposes only!
 */
void calibrateOscillator(void) {
    uchar       step = 128;
    uchar       trialValue = 0, optimumValue;
    int         x, optimumDev, targetValue = (unsigned)(1499 * (double)F_CPU / 10.5e6 + 0.5);

    /* do a binary search: */
    do {
        OSCCAL = trialValue + step;
        x = usbMeasureFrameLength();    /* proportional to current real frequency */
        if(x < targetValue)             /* frequency still too low */
            trialValue += step;
        step >>= 1;
    } while(step > 0);
    /* We have a precision of +/- 1 for optimum OSCCAL here */
    /* now do a neighborhood search for optimum value */
    optimumValue = trialValue;
    optimumDev = x; /* this is certainly far away from optimum */
    for (OSCCAL = trialValue - 1; OSCCAL <= trialValue + 1; OSCCAL++) {
        x = usbMeasureFrameLength() - targetValue;
        if(x < 0)
            x = -x;
        if(x < optimumDev){
            optimumDev = x;
            optimumValue = OSCCAL;
        }
    }
    OSCCAL = optimumValue;
}
/*
Note: This calibration algorithm may try OSCCAL values of up to 192 even if
the optimum value is far below 192. It may therefore exceed the allowed clock
frequency of the CPU in low voltage designs!
You may replace this search algorithm with any other algorithm you like if
you have additional constraints such as a maximum CPU clock.
For version 5.x RC oscillators (those with a split range of 2x128 steps, e.g.
ATTiny25, ATTiny45, ATTiny85), it may be useful to search for the optimum in
both regions.
*/
#endif
