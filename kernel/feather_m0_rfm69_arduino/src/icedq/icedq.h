#ifndef ICED_Q_H_
#define ICED_Q_H_

#define ICEDQ_MAX_BYTESTRING_LEN              32
#define ICEDQ_MAX_NUM_SUSCRIPTIONS            100

#define ICEDQ_SUCCESS                         1
#define ICEDQ_NO_MORE_SUSCRIPTIONS_AVAILABLE  2

typedef enum tIcedEncoding {
  IcedEncodingInteger = 0,
  IcedEncodingString = 0
};

typedef struct{
  const char* name;
  tIcedEncoding encoding;
}tIcedQTopic;

typedef struct{
	volatile uint8_t* queue;
	volatile uint32_t capacity;
	volatile uint32_t head;  //These 2 are how processes synchronize, so make them visible to
	volatile uint32_t tail;  //each other with volatile i.e. tell the compiler to not optimize them
}tIcedQQueue;

typedef struct{
  tIcedQTopic* topic;
  const char* routing_key;
  tIcedQQueue* registered_queue;
  bool free;
}tIcedQSuscription;


#endif /* ICED_Q_H_ */
