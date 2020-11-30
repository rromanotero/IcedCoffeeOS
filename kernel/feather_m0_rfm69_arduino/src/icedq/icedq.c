/**
*   This file is part of IcedCoffeeOS
*   (https://github.com/rromanotero/IcedCoffeeOS).
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

#define ICEDQ_SUSCRIPTION_POOL_SIZE  ICEDQ_MAX_NUM_SUSCRIPTIONS

typedef struct{
	tIcedQSuscription list[ICEDQ_SUSCRIPTION_POOL_SIZE];
	uint32_t count;
}tSuscriptionPool;

tSuscriptionPool suscription_pool;
tIcedQSuscription* active_subscriptions[ICEDQ_SUSCRIPTION_POOL_SIZE];
uint32_t num_of_active_suscriptions;


/*
*   IcedQ Init
**/
void icedq_init(){
	//init suscription pool
	for(uint32_t i=0; i<ICEDQ_SUSCRIPTION_POOL_SIZE; i++)
	  suscription_pool.list[i].free = true;

	//Init active subscriptions
	for(uint32_t i=0; i<ICEDQ_SUSCRIPTION_POOL_SIZE; i++)
		active_subscriptions[i] = NULL;
}


/*
*   IcedQ Publish to topic
**/
void icedq_publish(const char* topic, uint8_t* raw_message_bytes, uint32_t message_len_in_bytes){
	//TODO:
	//	- Replace linear looking for topic for a table lookup
	//	- Add Routing Key
	//

  for(int i=0; i<num_of_active_suscriptions; i++){

			Serial.print("MATCHING:");
			Serial.print(topic);
			Serial.print(active_subscriptions[i]->topic);

			//Look for suscriptors
      if( active_subscriptions[i] != NULL && strcmp(topic, active_subscriptions[i]->topic) == 0 ){

					//Found one. Pubished to its queue.
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

					Serial.print("Space used:");
					Serial.println(spaced_used);

					Serial.println("PRODUCER: tail:");
					Serial.println(tail);

					Serial.println("PRODUCER: head");
					Serial.println(head);

					if( spaced_used + message_len_in_bytes < q->capacity ){

						Serial.println("PRODUCER: producing the content: ");

						for(int j=0; j<message_len_in_bytes; j++){
							Serial.write(raw_message_bytes[j]);
						}
						Serial.println("");


						//if there's space,
						//copy over raw bytes
						for(int j=0; j<message_len_in_bytes; j++){
								q->queue[q->tail] = raw_message_bytes[j];
								q->tail = (q->tail + 1) % q->capacity;
						}

						Serial.println("PRODUCER: NEW value of tail:");
						Serial.println(q->tail);

					}else{
						//Queue full. Silently skip it.
					}
      }
  }
}

/*
*   IcedQ Subscribe
**/
uint32_t icedq_subscribe(const char* topic, tIcedQQueue* queue){

    tIcedQSuscription* suscription = suscriptions_pool_get_one();

    if(suscription ==  NULL)
        return ICEDQ_NO_MORE_SUSCRIPTIONS_AVAILABLE;

    suscription->topic = topic;
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
