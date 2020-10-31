#include <RHReliableDatagram.h>
#include <RH_RF69.h>
#include <SPI.h>

#define BASE_STATION_ADDR 111 //arbitrary, jus tkeep it consistent

// Singleton instance of the radio driver
RH_RF69 driver(8, 3); // Adafruit Feather M0 with RFM95

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, BASE_STATION_ADDR);

int incomingByte = 0;   // for incoming serial data
char start_symbol  = '+';
char end_symbol    = '-';

#define LED_PIN  13
#define MAX_FILE_LEN 20

void setup() 
{
  Serial.begin(9600);
  
  while( !Serial );
  
  if (!manager.init()){}
  else{}
  
  if (!driver.setFrequency(915.0)){}

  //set power high
  driver.setTxPower(19);

  //set address  
  manager.setThisAddress( BASE_STATION_ADDR );
}

// Dont put this on the stack:
uint8_t data[30];
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
char file[ MAX_FILE_LEN ]; 

void loop()
{
     //Request from App uploader client
     char c;
     if( (c = Serial.read()) == start_symbol ){ 
         Serial.println( "(From base station) Start symbol found" );
       
         //Read file
         read_file( file, MAX_FILE_LEN );
         int len = strlen( file );
         Serial.println( "(From base station): File received" );
         
         char buf[70];
         sprintf( buf, "(From base station): Length: %d", len );
         Serial.println( buf );
         
         //Transmit file over radio
         bool acked = manager.sendtoWait( (uint8_t*)file, strlen(file), 19 );
         
         //Respond to client
         if( acked )
            Serial.println( "(From base station): File trasnmitted \n" );
         else
            Serial.println( "(From base station): File trasmission failed. May god help them.\n" );
           
         //Terminate connection
         Serial.print( "-" );
     }
       
}

/*
*  Reads data from the UART and places it in request.
*  It will read until the end_symbol found or buf_size is reached.
*
*/
void read_file( char* file, int buf_size ){
    int i = 0;             //Start with an empty buffer
    char c;
  
    //Put UART data to buffer until end_of_symbol is received
    do{        
         //Read char
         //    Ignores non-ascii, or odd characters that are 
         //    read when no char is available at rx pin. Why no use .available() ?
         //    Because it won't work. Apparently it's a bug fixed in more recent versions.
         do{
            c = Serial.read();
         }while( c > 127 || c < 0 );
            
         //Append to request buffer
         file[ i++ ] = c;
         
     }while( c != end_symbol && i < buf_size ); 
     
     //Append null char (replaces the end symbol)
     file[ i-1 ] = '\0';
}

 /*
*    Signal init sucess bu blinking the red LED 3 times.
*/
void signal_init_sucess(){
  
    for( int i=0; i<3; i++){
      digitalWrite( LED_PIN, HIGH );
      delay(1500);
      digitalWrite( LED_PIN, LOW );
      delay(1500);
    }
}
