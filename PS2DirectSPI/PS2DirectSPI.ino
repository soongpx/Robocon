#include <SPI.h>
#define SPI_CLK 52
#define SPI_MISO 50
#define SPI_MOSI 51
#define SlaveSelect 53
#define SlaveAck 2
#define BufferSize 11
uint8_t SPI_Packet[BufferSize]={0};
static byte ReadAllData[]={0x01,0x42,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

void setup() 
{
  Serial.begin(115200);
  SPI.begin();
  SPI.beginTransaction(SPISettings(100000, LSBFIRST, SPI_MODE3));
  pinMode(SPI_MISO, INPUT_PULLUP);
  pinMode(SlaveAck, INPUT_PULLUP);
  pinMode(SPI_CLK, OUTPUT); //configure ports
  pinMode(SPI_MOSI, OUTPUT);
  pinMode(SlaveSelect, OUTPUT);
  digitalWrite(SlaveSelect, HIGH);
}


void loop() {
  uint8_t i = 0;
  digitalWrite(SlaveSelect, LOW);   // Set Attention Line Low at Start of Packet
  while(i < 11)
  {
    SPI_Packet[i] = SPI.transfer(ReadAllData[i]);
    i = i + 1;
  } 
  digitalWrite(SlaveSelect, HIGH);  // Set Attention Line High after End of Packet
  
  for(i = 0; i < BufferSize; i++)
  {
    //Serial.print(SPI_Packet[i],HEX);      // Not use due to variable Print size
    Serial.print((SPI_Packet[i] & 0xF0) >> 4, HEX);
    Serial.print(SPI_Packet[i] & 0x0F, HEX);
    if (i < (BufferSize-1) ) Serial.print(",");
    else Serial.println("");
  }
  
  delay(100);
}
