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

void spin_lock_acquire(tLock* lock){
  //I need an actual spin lock here....
  //disabling interrupts is outrageous =P
  //
  //LAter can aslo add a MUTEX so the thread goes to sleep
  //instead of waiting.... pros and cons i guess...
  uint32_t primask;
	__asm volatile (
		"mrs	%0, PRIMASK\n\t"
		"cpsid	i\n\t"
		: "=r" (primask)
  );

	lock->primask = primask; 
}

void spin_lock_release(tLock* lock)
{
  __asm volatile (
		"msr	PRIMASK, %0\n\t"
		:
    : "r" (lock->primask)
  );
}
