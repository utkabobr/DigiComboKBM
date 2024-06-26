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

#ifndef _DIGICOMBOC_H_
#define _DIGICOMBOC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define MAX(a,b) (a>b?a:b)

#define REPORT_ID_SIZE      1
#define REPORT_ID_MOUSE     0x1
#define REPORT_ID_KEYBOARD  0x2

#define MAX_HELD_KEYS             6
#define REPORT_SIZE_MOUSE         REPORT_ID_SIZE + 1 + 2 + 1         // Masked buttons + X/Y + Wheel
#define REPORT_SIZE_KEYBOARD      REPORT_ID_SIZE + 1 + MAX_HELD_KEYS // Modifiers + keys

#define BUFFER_SIZE MAX(REPORT_SIZE_MOUSE, REPORT_SIZE_KEYBOARD)

extern uint8_t lastReportId;
extern uint8_t report_buffer[];

void usbBegin();
void usbPollWrapper();
void usbReportSend(uint8_t size);

#ifdef __cplusplus
}
#endif

#endif
