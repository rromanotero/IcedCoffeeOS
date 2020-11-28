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

#define TICK_FREQ		SYS_SCHED_CONTEXT_SWITCH_FREQ

extern void process_thread_delete(void);
extern void idle_process_thread(void);

//This is where thread/processes data strutures
//are allocated memory. All thread pointers passe around
//ultimately point to this.
static tThreadPool thread_pool;

static uint32_t proc_started_count = 0;	//How many processes have been started
																				//since the beginning of time
/*
*	Scheduler Init
*
*   Initializes the scheduler. The system timer is not started here.
*/
void scheduler_init(void){
	//Init context switcher
	context_switcher_init(TICK_FREQ);

	//Init thread pool
	for(uint32_t i=0; i<SCHEDULER_THREAD_POOL_SIZE; i++){
		tMiniProcess proc;
		proc.state = ProcessStateFree;
		thread_pool.list[i] = proc;
		thread_pool.count++;
	}


	//Init ready and blocked queues
	proc_queue_init();

	//Init process stack
	static tMemRegion stack_memreg;
	hal_memreg_read( MemRegUserStack, &stack_memreg );
	stack_init( (uint32_t*)stack_memreg.base );	//stack_init( epstack )

	//No processes
	proc_started_count = 0;

	//Create idle process/thread
	//(we need one!)
	scheduler_thread_create( idle_process_thread, "idle process thread", 256, ProcQueueReadyIdle );
}

/*
*	Scheduler Process Create
*
*	Creates a process from a binary in nvmem. Ticking here begins!
*/
uint32_t scheduler_process_create( uint8_t* binary_file_name, const char* name, uint32_t* loader_rval, tProcQueueId destination_queue_id ){
	//tMemRegion proc_memregion;

	//Load app binary
	//uint32_t rval = loader_load_app( binary_file_name, &proc_memregion, &stack_sz );
	//	if(  rval != LOADER_LOAD_SUCCESS ){
	//		*loader_rval = rval;				//populate loader error
	//		return SCHEDULER_PROCESS_CREATE_FAILED;
	//	}

	//LOAD BIN ... THEN CALL scheduler_thread_create


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
uint32_t scheduler_thread_create( void(*thread_code)(void), const char* name, uint32_t stack_sz, tProcQueueId destination_queue_id ){
	tMiniProcess* proc = thread_pool_get_one();

	if(proc == NULL)
			faults_kernel_panic("Failed to schedule. Thread pool exhausted.");

	//Set process info
	proc->name = name;
	proc->state = ProcessStateReady;
	proc->queue_id = destination_queue_id;

	//Allocate space for "fake" context
	stack_alloc( CONTEXT_SIZE );

	//Allocate space for remaining stack
	proc->sp = stack_top();       //set SP
	stack_alloc( stack_sz - CONTEXT_SIZE );					//make space

	//Insert "fake" context
	proc->sp[OFFSET_LR] =     ((uint32_t) (process_thread_delete +1));
	proc->sp[OFFSET_PC] =     ((uint32_t) (thread_code +1));
	proc->sp[OFFSET_APSR] =   ((uint32_t) INITIAL_APSR);

	//Increment counter in list
	proc_started_count++;

	//Send to ready queue
	proc_queue_enqueue(destination_queue_id, proc);

	//Start ticking on first process
	// Proc 1 = Idle thread
	// Proc 2 = this thread (FIRST actual process)
  if( proc_started_count == 2 ){
		tMiniProcess* idle_thread = proc_queue_peek(ProcQueueReadyIdle);
		hal_cpu_set_psp( (uint32_t)idle_thread->sp );	//or else the first tick fails

		context_switcher_begin_ticking();
  }

	return SCHEDULER_PROCESS_CREATE_SUCCESS;
}

/*
*		 Scheduler Next Process
*
*    Slect next process to run from all the ready queues
*		 as per some priority
*/
static tProcQueueId queue_ids[] = {
	ProcQueueReadyRealTime,	//priority 1
	ProcQueueReadySystem,		//priority 2
	ProcQueueReadyUser,		  //priority 3
	ProcQueueReadyBatch,	  //...
	ProcQueueReadyIdle
};

tMiniProcess* scheduler_proc_next(tMiniProcess* active_proc){

	if(active_proc->state != ProcessStateNull){
		//Queue active process back to where it was
		proc_queue_enqueue(active_proc->queue_id, active_proc);	//enqueue(active)
	}

	//Find next
	tMiniProcess* next;

	for(uint32_t i=0; i<PROC_QUEUE_TOTAL_NUM_OF_READY_QUEUES; i++){
			if( proc_queue_size(queue_ids[i]) > 0 ){
				next = proc_queue_dequeue(queue_ids[i]);	//next = dequeue()
				break;
			}
	}

	if(next == NULL)
			faults_kernel_panic("Failed to schedule. No process found in ready queues.");

	return next;
}


void scheduler_process_current_stop(void){
	//changes thread state to dead
	//(so it's not scheduled anymore)
	//active_proc.state =  ProcessStateDead;

	//context switches
	context_switcher_trigger();
}


tMiniProcess* thread_pool_get_one(){
	for(uint32_t i=0; i<SCHEDULER_THREAD_POOL_SIZE; i++){
		if( thread_pool.list[i].state == ProcessStateFree )
			return &(thread_pool.list[i]);
	}

	return NULL; //We're out of threads
}
