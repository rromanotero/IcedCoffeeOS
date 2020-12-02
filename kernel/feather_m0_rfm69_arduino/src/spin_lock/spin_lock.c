



void spin_lock_init(){
  //nothing to do
}

void spin_lock_acquire(){
  //I need an actual spin lock here....
  //disabling interrupts is outrageous =P
  __asm volatile ("cpsid  i");
}

void spin_lock_release()
{
  __asm volatile ("cpsie  i");
}
