
/*	Puts the processor to sleep. executes in user mode  */
__attribute__((naked)) void idle_process_thread(){
     __asm volatile(
        "wfi\n" \
        "b =idle_process_thread"
      );
}

void process_thread_delete(){
  while(1);
/*	Deletes the current process/thread. this function will be
 accessed in user mode when a thread "returns" hence the syscall  */
  // svc 29 /* thread_stop system call see syscalls.c */
   //b .		/* execution won't get here */
}
