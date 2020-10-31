#include <RHReliableDatagram.h>
#include <RH_RF69.h>
#include <SPI.h>

#define MY_ADDRESS 17

// Singleton instance of the radio driver
RH_RF69 driver(8, 3); // Adafruit Feather M0 with RFM95

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, MY_ADDRESS);

void setup() 
{
  Serial.begin(9600);
  
  while( !Serial );
  
  if (!manager.init()){}
  else{}
  
  if (!driver.setFrequency(915.0)){}

  driver.setTxPower(6);
}

// Dont put this on the stack:
uint8_t data[30];
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];

void loop()
{
  if (manager.available())
  {
    // Read message
    uint8_t len = sizeof(buf);
    uint8_t from;
    
    if (manager.recvfromAck(buf, &len, &from))
    {
      sprintf( (char*)data, "%d", -driver.lastRssi() );
      Serial.println((char*)data);
    }   
      
  }
}

