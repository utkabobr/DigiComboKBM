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

#include "DigiComboKBM.h"
#include "DigiComboKBM_C.h"
#include "scancode_ascii_table.h"

#include <Arduino.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <util/delay.h>

Digi_ComboKBM DigiComboKBM;

void Digi_ComboKBM::begin()
{
    usbBegin();
    delay(1000);
}

void Digi_ComboKBM::poll()
{
    usbPollWrapper();
}

void Digi_ComboKBM::delay(unsigned long durationMs)
{
    unsigned long last = millis();
    while (durationMs > 0) {
        unsigned long now = millis();
        durationMs -= now - last;
        last = now;
        poll();
    }
}

void Digi_ComboKBM::pressKey(uint8_t keycode, uint8_t modifiers)
{
    report_buffer[0] = REPORT_ID_KEYBOARD;
    report_buffer[1] = modifiers;
    report_buffer[2] = keycode;
    usbReportSend(REPORT_SIZE_KEYBOARD);
}

void Digi_ComboKBM::sendKeyStroke(uint8_t keycode, uint8_t modifiers)
{
    pressKey(keycode, modifiers);
    pressKey(0, 0);
}

void Digi_ComboKBM::move(char deltaX, char deltaY, char deltaS, unsigned char buttons)
{
	if (deltaX == -128) deltaX = -127;
	if (deltaY == -128) deltaY = -127;
	if (deltaS == -128) deltaS = -127;
	report_buffer[0] = REPORT_ID_MOUSE;
	report_buffer[1] = buttons;
	report_buffer[2] = *(reinterpret_cast<unsigned char *>(&deltaX));
	report_buffer[3] = *(reinterpret_cast<unsigned char *>(&deltaY));
	report_buffer[4] = *(reinterpret_cast<unsigned char *>(&deltaS));
	usbReportSend(REPORT_SIZE_MOUSE);
}

size_t Digi_ComboKBM::write(uint8_t ascii)
{
    uint8_t data = pgm_read_byte_near(ascii_to_scan_code_table + (ascii - 8));
    sendKeyStroke(data & 0b01111111, data >> 7 ? MOD_RIGHT_SHIFT : 0);
    return 1;
}
