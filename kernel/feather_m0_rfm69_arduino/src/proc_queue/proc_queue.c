
typedef struct{
	tMiniProcess* list[SCHEDULER_MAX_NUM_PROCESSES];
	uint32_t capacity;
	uint32_t head;
	uint32_t tail;
}tQueue;

//The different queues
static tQueue q_ready_real_time;
static tQueue q_blocked_real_time;
static tQueue q_ready_system;
static tQueue q_blocked_system;
static tQueue q_ready_user;
static tQueue q_blocked_user;
static tQueue q_ready_batch;
static tQueue q_blocked_batch;
static tQueue q_ready_idle;

static tQueue* queue_map[PROC_QUEUE_TOTAL_NUM_OF_QUEUES];

void proc_queue_init(){

  queue_map[ProcQueueReadyRealTime] = &q_ready_real_time;
  queue_map[ProcQueueReadySystem] = &q_ready_system;
  queue_map[ProcQueueReadyUser] = &q_ready_user;
  queue_map[ProcQueueReadyBatch] = &q_ready_batch;
	queue_map[ProcQueueReadyIdle] = &q_ready_idle;
  queue_map[ProcQueueBlockedRealTime] = &q_blocked_real_time;
  queue_map[ProcQueueBlockedSystem] = &q_blocked_system;
  queue_map[ProcQueueBlockedUser] = &q_blocked_user;
  queue_map[ProcQueueBlockedBatch] = &q_blocked_batch;

	for(uint32_t i=0; i<PROC_QUEUE_TOTAL_NUM_OF_QUEUES; i++){
		queue_map[i]->head = 0;
		queue_map[i]->tail = 0;
		queue_map[i]->capacity = SCHEDULER_MAX_NUM_PROCESSES;
	}

}

void proc_queue_enqueue(tProcQueueId queue_id, tMiniProcess* proc){
  tQueue* q = queue_map[queue_id];

  q->list[q->tail] = proc;
  q->tail = (q->tail + 1) % q->capacity;
}

tMiniProcess* proc_queue_dequeue(tProcQueueId queue_id){
  tQueue* q = queue_map[queue_id];
  tMiniProcess* proc;

  proc =  q->list[q->head];
  q->head = (q->head + 1) % q->capacity;

  return proc;
}

tMiniProcess* proc_queue_peek(tProcQueueId queue_id){
  tQueue* q = queue_map[queue_id];

  return q->list[q->head];
}

uint32_t proc_queue_size(tProcQueueId queue_id){
  tQueue* q = queue_map[queue_id];

  return q->head - q->tail;
}
