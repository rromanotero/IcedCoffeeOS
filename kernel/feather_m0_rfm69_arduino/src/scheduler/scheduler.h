#ifndef SCHEDULER_H_
#define SCHEDULER_H_

enum tProcessState { ProcessStateReady = 0, ProcessStateRunning, ProcessStateDead, ProcessStateNull  };	

typedef struct{
	const char* name;
	uint32_t* sp;
	tProcessState state;
}tMiniProcess;


#endif /* SCHEDULER_H_ */
