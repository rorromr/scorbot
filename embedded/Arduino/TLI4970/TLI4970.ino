
// inslude the SPI library:
#include <SPI.h>


// set pin 10 as the slave select Arduino Mega
#define CURRENT_CS 10
SPISettings current_settings(500000, MSBFIRST, SPI_MODE1);

void setup() {
  // set the slaveSelectPin as an output:
  // initialize SPI:
  SPI.begin(CURRENT_CS);
  Serial.begin(115200);
}

void loop() {
  union
  {
    uint16_t val;
    struct
    {
      uint8_t lsb;
      uint8_t msb;
    };
  } current;
  SPI.beginTransaction(CURRENT_CS, current_settings);
  current.msb = SPI.transfer(CURRENT_CS, 0x00, SPI_CONTINUE);
  current.lsb = SPI.transfer(CURRENT_CS, 0x00, SPI_LAST);
  SPI.endTransaction();
  frameParser(current.val);
  delay(50);
}

void frameParser(uint16_t value)
{
  union
  {
    uint16_t val;
    struct
    {
      unsigned raw_current: 13;
      unsigned ocd        : 1 ;
      unsigned par        : 1 ;
      unsigned stat       : 1 ;
    };
  } sensor;
  sensor.val = value;
  
  if (sensor.stat)
  {
    Serial.println("Status package");
    return;
  }   
  float current = (int32_t)(sensor.raw_current-4096)*0.00625f;
  Serial.println(current);
  
}

