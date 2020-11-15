#define TICK_FREQ		3

/*
*	The infamous tick callback
*
*	Context switch takes place here.
*/
void tick_callback(void){

	// Trigger PendSV which performs the actual context switch
	SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
}


/*
*	Scheduler Init
*
*   Initializes the scheduler. The system timer is not started here.
*/
void scheduler_init(void){

	hal_cpu_set_psp_active();
	hal_cpu_set_psp();
	hal_cpu_systimer_start(TICK_FREQ, tick_callback);

	while(1); //Should nto return

}
