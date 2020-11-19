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

/*  There's now way I was getting that PendSV Handler code right
 *  by myself. Thanks to Adam Heinrich. So here's the License.
 *    - Rafael
 *
 * This file is part of os.h.
 *
 * Copyright (C) 2016 Adam Heinrich <adam@adamh.cz>
 *
 * Os.h is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Os.h is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with os.h.  If not, see <http://www.gnu.org/licenses/>.
 */

#define CONTEXT_SIZE    16
#define INITIAL_APSR    (1 << 24) //Bit 24 is the Thumb bit
#define OFFSET_LR       13
#define OFFSET_PC       14
#define OFFSET_APSR     15

tMiniProcess* active_proc;		//The active process

/*
*   Context Switcher Init
*/
static uint32_t tick_frequency;

void context_switcher_init(uint32_t tick_freq){
  tick_frequency = tick_freq;

  //Active process is the null process
  //A null process (used to mark the lack of an active process)
  static tMiniProcess null_proc;
  null_proc.name = "null",
  null_proc.state = ProcessStateNull;

	active_proc = &null_proc;


  //Inits Low Pty Int
	hal_cpu_lowpty_softint_start( low_pty_callback );
}


/*
* Triggers a context switch
*/
void context_switcher_trigger(){
    //trigger low pty software interrupt
    hal_cpu_lowpty_softint_trigger();
}

/*
*   Begin context switching
*
*  Here's where the System Timer begins ticking
*/
void context_switcher_begin_ticking(){
    hal_cpu_systimer_start( tick_frequency, tick_callback );
}

/*
*	The tick callback
*
*/
void tick_callback(void){

	//Context switch happens in the low pty interrupt
	//
	//This is to prevent the issue of a tick interrupting
	//an interrupt handler and then giving it back
	//to a thread (instead of the handler that had it).
	hal_cpu_lowpty_softint_trigger();
}


/*
*	Low Pty interrupt callback
*
*  Context switch takes place here.
*/
__attribute__((naked)) void low_pty_callback(void){
	// This is adapted from both MiniOS's and Heinrich's os.h
	// Cortex M0 context switcher.
	//
	// https://github.com/adamheinrich/os.h/blob/master/src/os.c

	__asm volatile(
    "cpsid	i        \n" \
      :::
    );

	//save software context
  __asm volatile(
    "mrs	r0, psp      \n"  \
    "sub	r0, #16      \n"  \
    "stmia	r0!,{r4-r7}\n"  \
    "mov	r4, r8       \n"  \
    "mov	r5, r9       \n"  \
    "mov	r6, r10      \n"  \
    "mov	r7, r11      \n"  \
    "sub	r0, #32      \n"  \
    "stmia	r0!,{r4-r7}\n"  \
    "sub	r0, #16      \n" \
		"msr psp, r0"			//<<--- Needed so SP points to the top of the stack
      :::							// New SP (FIgure 12.2 MiniOS Book)
    );

  //Not the null process?
	//(this'll skiip hal_cpu_get_psp() on the very first tick)
	if( active_proc->state != ProcessStateNull ){
		//save SP
		active_proc->sp = (uint32_t*)hal_cpu_get_psp();
	}

	//get next active process
	active_proc = scheduler_proc_next(active_proc);

	//restore SP
	hal_cpu_set_psp( (uint32_t)active_proc->sp );

	//restore software context
  __asm volatile(
      "ldmia	r0!,{r4-r7} \n"  \
      "mov	r8, r4        \n"  \
      "mov	r9, r5        \n"  \
      "mov	r10, r6       \n"  \
      "mov	r11, r7       \n"  \
      "ldmia	r0!,{r4-r7} \n"  \
      "msr	psp, r0       "   //<<--- Needed so SP points to the top of the stack
      :::											// New SP (FIgure 12.2 MiniOS Book)
    );

		//give CPU to active process
    __asm volatile(
      "ldr r0, =0xFFFFFFFD\n" \
      "cpsie i       			\n" \
      "bx r0"
        :::
      );	//0xFFFFFFFD is USER_MODE_EXEC_VALUE. It tells
					//the CPU to return the exception in User (thread) mode
					//and using the PSP.
}
