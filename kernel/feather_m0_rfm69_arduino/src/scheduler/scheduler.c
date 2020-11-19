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

#define TICK_FREQ				SYS_SCHED_CONTEXT_SWITCH_FREQ

extern void process_thread_delete(void);
extern void idle_process_thread(void);

static tProcessList proc_list;				// process list
static tMiniProcess* wait_list[10];	//waiting list
static uint32_t proc_count = 0;

/*
*	Scheduler Init
*
*   Initializes the scheduler. The system timer is not started here.
*/
void scheduler_init(void){
	//Init context switcher
	context_switcher_init(TICK_FREQ);

	//Init process stack
	static tMemRegion stack_memreg;
	hal_memreg_read( MemRegUserStack, &stack_memreg );
	stack_init( (uint32_t*)stack_memreg.base );	//stack_init( epstack )

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
		context_switcher_begin_ticking();
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
  	  context_switcher_begin_ticking();
  }

	return SCHEDULER_PROCESS_CREATE_SUCCESS;
}

/*
*       I believe this is round robin! It jsut looks different thatn what
*		it appears on textsbooks.
*/
static uint32_t process_mark = 0;

tMiniProcess* scheduler_process_next(){
		//Increment process mark
		//(skip dead processes)
		uint32_t count=0;
		do{
			count++;
			process_mark = (process_mark + 1) % proc_list.count;
		}while( proc_list.list[process_mark].state == ProcessStateDead );


		//Return process at process mark
		return &(proc_list.list[process_mark]);
}


void scheduler_process_current_stop(void){
	//changes thread state to dead
	//(so it's not scheduled anymore)
	active_proc->state =  ProcessStateDead;

	//context switches
	context_switcher_trigger();
}
