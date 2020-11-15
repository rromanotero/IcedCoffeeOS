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
 *  by myself. So here's the License.
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

#define TICK_FREQ				SYS_SCHED_CONTEXT_SWITCH_FREQ
#define CONTEXT_SIZE    16
#define INITIAL_APSR    (1 << 24) //Bit 24 is the Thumb bit
#define OFFSET_LR       13
#define OFFSET_PC       14
#define OFFSET_APSR     15

extern void process_thread_delete(void);
extern void idle_process_thread(void);

tProcessList proc_list;				// process list
tMiniProcess* wait_list[10];	//waiting list
tMiniProcess* active_proc;		//The active process
uint32_t proc_count = 0;

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
	active_proc = scheduling_policy_next( active_proc, &proc_list ); //&(proc_list.list[1]);

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


/*
*	Scheduler Init
*
*   Initializes the scheduler. The system timer is not started here.
*/
void scheduler_init(void){
	//Init process stack
	static tMemRegion stack_memreg;
	hal_memreg_read( MemRegUserStack, &stack_memreg );
	stack_init( (uint32_t*)stack_memreg.base );	//stack_init( epstack )

	//Inits Low Pty Int
	hal_cpu_lowpty_softint_start( low_pty_callback );

	//Active process is the null process
  //A null process (used to mark the lack of an active process)
  static tMiniProcess null_proc;
  null_proc.name = "null",
  null_proc.state = ProcessStateNull;

	active_proc = &null_proc;

	//No processes
	proc_count = 0;

	//Create idle process/thread
	//(we need one!)
	scheduler_thread_create( idle_process_thread, "idle process thread", 256 );
}

/*
*	Scheduler Process Create
*
*	Creates a process from a binary in nvmem. Ticking here begins!
*/
uint32_t scheduler_process_create( uint8_t* binary_file_name, const char* name, uint32_t* loader_rval ){
	tMemRegion proc_memregion;
	uint32_t stack_sz;

	//Load app binary
	//uint32_t rval = loader_load_app( binary_file_name, &proc_memregion, &stack_sz );
//	if(  rval != LOADER_LOAD_SUCCESS ){
//		*loader_rval = rval;				//populate loader error
//		return SCHEDULER_PROCESS_CREATE_FAILED;
//	}

	//Set process info
	proc_list.list[proc_list.count].name = name;
	proc_list.list[proc_list.count].state = ProcessStateReady;

	//Allocate space for "fake" context
	stack_alloc( CONTEXT_SIZE );

	//Allocate space for remaining stack
	proc_list.list[proc_list.count].sp = stack_top();       //set SP
	stack_alloc( stack_sz - CONTEXT_SIZE );					//make space

	//Insert "fake" context
	proc_list.list[proc_list.count].sp[OFFSET_LR] =     ((uint32_t) (process_thread_delete +1));
	proc_list.list[proc_list.count].sp[OFFSET_PC] =     ((uint32_t) (proc_memregion.base +1));
	proc_list.list[proc_list.count].sp[OFFSET_APSR] =   ((uint32_t) INITIAL_APSR);

	//Increment counter in list
	proc_list.count++;

	//Start ticking on first process (idle thread is process/thread 1)
	if( proc_list.count == 2 ){
		hal_cpu_set_psp( (uint32_t)proc_list.list[0].sp );						//or else the first tick fails
		hal_cpu_systimer_start( TICK_FREQ, tick_callback );
	}

	return SCHEDULER_PROCESS_CREATE_SUCCESS;
}

/*
*	Scheduler Thread Create
*
*	Creates a Thread
*
*   It's really a process, but processes are so simple, that there's
*   no  difference between a process and a thread.
*
*/
uint32_t scheduler_thread_create( void(*thread_code)(void), const char* name, uint32_t stack_sz ){

	//Set process info
	proc_list.list[proc_list.count].name = name;
	proc_list.list[proc_list.count].state = ProcessStateReady;

	//Allocate space for "fake" context
	stack_alloc( CONTEXT_SIZE );

	//Allocate space for remaining stack
	proc_list.list[proc_list.count].sp = stack_top();       //set SP
	stack_alloc( stack_sz - CONTEXT_SIZE );					//make space

	//Insert "fake" context
	proc_list.list[proc_list.count].sp[OFFSET_LR] =     ((uint32_t) (process_thread_delete +1));
	proc_list.list[proc_list.count].sp[OFFSET_PC] =     ((uint32_t) (thread_code +1));
	proc_list.list[proc_list.count].sp[OFFSET_APSR] =   ((uint32_t) INITIAL_APSR);

	//Increment counter in list
	proc_list.count++;

  //Start ticking on first process (idle thread is process/thread 1)
  if( proc_list.count == 2 ){
  	  hal_cpu_set_psp( (uint32_t)proc_list.list[0].sp );						//or else the first tick fails
  	  hal_cpu_systimer_start( TICK_FREQ, tick_callback );
  }

	return SCHEDULER_PROCESS_CREATE_SUCCESS;
}

void scheduler_process_current_stop(void){
	//changes thread state to dead
	//(so it's not scheduled anymore)
	active_proc->state =  ProcessStateDead;

	//context switches
	hal_cpu_lowpty_softint_trigger();
}
