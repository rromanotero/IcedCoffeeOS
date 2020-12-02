/**
*   This file is part of IcedCoffeeOS
*   (https://github.com/rromanotero/IcedCoffeeOS).
*
*   Copyright (c) 2020 Rafael Roman Otero.
*
*   This program is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program.  If not, see <https://www.gnu.org/licenses/>.
*
**/

/* Copyright (C) 2007-2015 Goswin von Brederlow <goswin-v-b@web.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or (at
 * your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef KERNEL_KPRINTF_H
#define KERNEL_KPRINTF_H


typedef void (*vcprintf_callback_t)(char c, void *state);

typedef struct {
    char *pos;
    size_t size;
} BufferState;

typedef struct Flags {
    _Bool plus:1;	// Always include a '+' or '-' sign
    _Bool left:1;	// left justified
    _Bool alternate:1;	// 0x prefix
    _Bool space:1;	// space if plus
    _Bool zeropad:1;	// pad with zero
    _Bool sign:1;	// unsigned/signed number
    _Bool upper:1;	// use UPPER case
} Flags;


#endif // #ifndef PRINTF_H
