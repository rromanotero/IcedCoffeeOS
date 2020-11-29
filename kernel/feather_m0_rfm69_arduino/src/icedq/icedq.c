

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

					tIcedQQueue* q = active_subscriptions[i]->registered_queue;
					volatile uint32_t tail = q->tail;
					volatile uint32_t head = q->head;

					int32_t spaced_used;
			    if(head > tail){
			      //tail went around
			      spaced_used = (q->capacity-head) + tail;
			    }
			    else{
			      spaced_used = (tail - head);
			    }

					//Serial.print("Space used:");
					//Serial.println(spaced_used);

					//Serial.println("PRODUCER: tail:");
		      //Serial.println(tail);

		      //Serial.println("PRODUCER: head");
		      //Serial.println(head);


					if( spaced_used + message_len_in_bytes < q->capacity ){

						//Serial.println("PRODUCER: producing the content: ");

						//for(int j=0; j<message_len_in_bytes; j++){
						//	Serial.write(raw_message_bytes[j]);
						//}
						//Serial.println("");

						//if there's space,
						//copy over raw bytes
						for(int j=0; j<message_len_in_bytes; j++){
								q->queue[q->tail] = raw_message_bytes[j];
								q->tail = (q->tail + 1) % q->capacity;
						}

						//Serial.println("PRODUCER: NEW value of tail:");
			      //Serial.println(q->tail);

					}else{
						Serial.println("Queue FULL");
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
