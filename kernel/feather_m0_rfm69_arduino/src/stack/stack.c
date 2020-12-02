/**
*   This file is part of IcedCoffeeOS
*   (https://github.com/rromanotero/IcedCoffeeOS).
*
*   and adapted from MiniOS:
*   (https://github.com/rromanotero/minios).
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

/**
 * @file	stack.c
 * @author
 * @version
 *
 * @brief Stack module. This is a generic module to handle allocation of memory in the stack.
 *		  It assumes a full-descendant stack, and ensures 8-byte alignment on allocation.
 *		  This module must be ported when porting MiniOS
 *
 *		  Careful with mixing integer arithmetic with pointer arithmetic.
 */

static void alloc(uint32_t);
static void align_to_eight_byte_boundary(void);

static uint32_t* stack;
static bool initialized = false;

void stack_init(uint32_t* address){
	stack = address;
	initialized = true;
}

uint32_t* stack_top(void){
	if( !initialized )
		faults_kernel_panic( "Reading uninit stack" );

	return stack;
}

void stack_alloc(uint32_t elements){
	if( !initialized )
		faults_kernel_panic( "Allocation on uninit stack" );

	alloc(elements);
	align_to_eight_byte_boundary();
}

static void alloc(uint32_t elements){
	stack = stack - elements;
}


static void align_to_eight_byte_boundary(void){
	uint32_t address = (uint32_t)stack;

	while( address % 8 != 0 ){
		address--;
	}

	stack = (uint32_t*)address;
}
