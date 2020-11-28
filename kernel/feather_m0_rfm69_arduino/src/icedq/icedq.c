

#define ICEDQ_SUSCRIPTION_POOL_SIZE  ICEDQ_MAX_NUM_SUSCRIPTIONS

typedef struct{
	tIcedQSuscription list[ICEDQ_SUSCRIPTION_POOL_SIZE];
	uint32_t count;
}tSuscriptionPool;

tSuscriptionPool suscription_pool;
tIcedQSuscription* active_subscriptions[ICEDQ_SUSCRIPTION_POOL_SIZE];
uint32_t num_of_active_suscriptions;

/*
*   Init IcedQ
**/
void icedq_init(){

  //init suscription pool
  for(uint32_t i=0; i<ICEDQ_SUSCRIPTION_POOL_SIZE; i++){
    suscription_pool.list[i].free = true;
  }

}


/*
*   Publish to topic
**/
uint32_t icedq_publish(tIcedQTopic* topic, const char* routing_key, uint8_t* raw_message_bytes, uint32_t message_len_in_bytes){

  //route to suscribed queues
  for(int i=0; i<num_of_active_suscriptions; i++){
      //TODO:
      //    Replace for a hash table lookup
      if( strcmp(topic->name, active_subscriptions[i]->topic->name) == 0 ){

        //TODO:
        //  Change so routing keys do not require exact matches
        //if( strcmp(routing_key, active_subscriptions[i]->routing_key) == 0 ){

            //copy over raw bytes
            for(int j=0; j<message_len_in_bytes; j++){
                active_subscriptions[i]->registered_queue->queue[active_subscriptions[i]->registered_queue->tail] = raw_message_bytes[j];
            		active_subscriptions[i]->registered_queue->tail = (active_subscriptions[i]->registered_queue->tail + 1) % active_subscriptions[i]->registered_queue->capacity;
						}
        //}
      }
  }
}

/*
*   Subscribe
**/
uint32_t icedq_subscribe(tIcedQTopic* topic, const char* routing_key, tIcedQQueue* queue){

    tIcedQSuscription* suscription = suscriptions_pool_get_one();

    if(suscription ==  NULL)
        return ICEDQ_NO_MORE_SUSCRIPTIONS_AVAILABLE;

    suscription->topic = topic;
    suscription->routing_key = routing_key;
    suscription->registered_queue = queue;

    active_subscriptions[num_of_active_suscriptions++] = suscription;

    return ICEDQ_SUCCESS;
}

tIcedQSuscription* suscriptions_pool_get_one(void){
  	for(uint32_t i=0; i<ICEDQ_SUSCRIPTION_POOL_SIZE; i++){
  		if( suscription_pool.list[i].free ){
          suscription_pool.list[i].free = false;  //No longer free
    			return &(suscription_pool.list[i]);
      }
  	}
  	return NULL; //We're out of threads
}
