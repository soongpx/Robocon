#include <SPI.h>
#define SPI_CLK 52
#define SPI_MISO 50
#define SPI_MOSI 51
#define SlaveSelect 53
#define SlaveAck 2
#define BufferSize 21
uint8_t SPI_Packet[BufferSize]={0};
//static byte ReadAllData[]={0x01,0x42,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
static byte Enter_Config[]={0x01,0x43,0x00,0x01,0x00};
static byte Turn_ON_Analog_Mode[]={0x01,0x44,0x00,0x01,0x03,0x00,0x00,0x00,0x00};
static byte Turn_ON_Pressure[]={0x01,0x4F,0x00,0xFF,0xFF,0x03,0x00,0x00,0x00};
static byte Maps_Motor[]={0x01,0x4D,0x00,0x00,0x01};
static byte Exit_Config[]={0x01,0x43,0x00,0x00,0x5A,0x5A,0x5A,0x5A,0x5A};
static byte type_read[]={0x01,0x45,0x00,0x5A,0x5A,0x5A,0x5A,0x5A,0x5A};


//#define BufferSize 21
static byte ReadAllData[]={0x01,0x42,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
void setup() 
{
  Serial.begin(115200);
  SPI.begin();
  SPI.beginTransaction(SPISettings(200000, LSBFIRST, SPI_MODE3)); //5 Microseconds per bit
  pinMode(SPI_MISO, INPUT_PULLUP);
  pinMode(SlaveAck, INPUT_PULLUP);
  pinMode(SPI_CLK, OUTPUT); //configure ports
  pinMode(SPI_MOSI, OUTPUT);
  pinMode(SlaveSelect, OUTPUT);
  digitalWrite(SlaveSelect, HIGH);

  PS2_SPI_SEND(Enter_Config,5);
  PS2_SPI_SEND(type_read,9);
  PS2_SPI_SEND(Turn_ON_Analog_Mode,9);
  PS2_SPI_SEND(Maps_Motor,5);
  PS2_SPI_SEND(Turn_ON_Pressure,9);
  PS2_SPI_SEND(Exit_Config,9);
  
  PS2_SPI_SEND(ReadAllData,BufferSize);
  
  PS2_SPI_SEND(Enter_Config,5);
  PS2_SPI_SEND(type_read,9);
  PS2_SPI_SEND(Turn_ON_Analog_Mode,9);
  PS2_SPI_SEND(Maps_Motor,5);
  PS2_SPI_SEND(Turn_ON_Pressure,9);
  PS2_SPI_SEND(Exit_Config,9);
}


void loop() {
  PS2_SPI_SEND(ReadAllData,BufferSize);
  
  for(uint8_t i = 0; i < BufferSize; i++) 
  {
    //Serial.print(SPI_Packet[i],HEX);      // Not use due to variable Print size
    Serial.print((SPI_Packet[i] & 0xF0) >> 4, HEX);
    Serial.print(SPI_Packet[i] & 0x0F, HEX);
    if (i < (BufferSize-1) ) Serial.print(",");
    else Serial.println("");
  }
  
  delay(100);
}

void PS2_SPI_SEND(byte* Data, uint8_t DataSize)
{
  uint8_t i = 0;
  digitalWrite(SlaveSelect, LOW);   // Set Attention Line Low at Start of Packet
  while(i < DataSize)             // Block for 450 Microseconds ( (40 + 10) x 9)
  {
    SPI_Packet[i] = SPI.transfer(Data[i]);
    delayMicroseconds(16);          //Delay 10 Microseconds (Seems Compulsory)
    i = i + 1;
  } // Please take note PS2 data retrieval takes 450 microseconds , may intefere Stepper motor control timing.
  digitalWrite(SlaveSelect, HIGH);  // Set Attention Line High after End of Packet  
}
