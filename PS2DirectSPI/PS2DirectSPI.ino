#include <SPI.h>
#define SPI_CLK 52
#define SPI_MISO 50
#define SPI_MOSI 51
#define SlaveSelect 53
#define SlaveAck 2
#define BufferSize 9
uint8_t SPI_Packet[BufferSize] = {0};
static byte ReadAllData[] = {0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static byte Enter_Config[] = {0x01, 0x43, 0x00, 0x01, 0x00};
static byte Turn_ON_Analog_Mode[] = {0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00};
static byte Maps_Motor[] = {0x01, 0x4D, 0x00, 0x00, 0x01};
static byte Exit_Config[] = {0x01, 0x43, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A};

#define PS4_SHARE                 0x01
#define PS4_UNKNOWN1              0x02
#define PS4_UNKNOWN2              0x04
#define PS4_OPTION                0x08
#define DPAD_UP                   0x10
#define DPAD_RIGHT                0x20
#define DPAD_DOWN                 0x40
#define DPAD_LEFT                 0x80
#define L2_PRESSED                0x01
#define R2_PRESSED                0x02
#define L1_PRESSED                0x04
#define R1_PRESSED                0x08
#define GREEN_TRIANGLE_PRESSED    0x10
#define RED_CIRCLE_PRESSED        0x20
#define BLUE_CROSS_PRESSED        0x40
#define PINK_SQUARE_PRESSED       0x80

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
  PS2_SPI_SEND(Enter_Config, 5);
  PS2_SPI_SEND(Turn_ON_Analog_Mode, 9);
  //  PS2_SPI_SEND(Maps_Motor,5);       // PS4 dont support this?
  //  PS2_SPI_SEND(Turn_ON_Pressure,9); // This is useless for ps4
  PS2_SPI_SEND(Exit_Config, 9);
}


void loop() {
  PS2_SPI_SEND(ReadAllData, BufferSize);
  if (SPI_Packet[2] != 0x5A) Serial.println("Invalid Packet");
  else 
  {
    if(SPI_Packet[1] == 0X41) Serial.println("Digital Mode");
    else if(SPI_Packet[1] == 0X73) Serial.println("Analog Mode");
    else Serial.println("Unknown Mode");
    
    if(SPI_Packet[3] ^ 0xFF) 
    {
      if (~SPI_Packet[3] & DPAD_UP) Serial.println("DPAD UP");
      if (~SPI_Packet[3] & DPAD_DOWN) Serial.println("DPAD DOWN");
      if (~SPI_Packet[3] & DPAD_LEFT) Serial.println("DPAD LEFT");
      if (~SPI_Packet[3] & DPAD_RIGHT) Serial.println("DPAD RIGHT");
      if (~SPI_Packet[3] & PS4_SHARE) Serial.println("SHARE BUTTON pressed");
      if (~SPI_Packet[3] & PS4_OPTION) Serial.println("OPTION BUTTON pressed");
    }
    if(SPI_Packet[4] ^ 0xFF) 
    {
      if (~SPI_Packet[4] & L1_PRESSED) Serial.println("L1 pressed");
      if (~SPI_Packet[4] & L2_PRESSED) Serial.println("L2 pressed");
      if (~SPI_Packet[4] & R1_PRESSED) Serial.println("R1 pressed");
      if (~SPI_Packet[4] & R2_PRESSED) Serial.println("R2 pressed");
      if (~SPI_Packet[4] & GREEN_TRIANGLE_PRESSED) Serial.println("GREEN Triangle pressed");
      if (~SPI_Packet[4] & RED_CIRCLE_PRESSED) Serial.println("Red Circle pressed");
      if (~SPI_Packet[4] & BLUE_CROSS_PRESSED) Serial.println("Blue Cross pressed");
      if (~SPI_Packet[4] & PINK_SQUARE_PRESSED) Serial.println("Pink Square pressed");
    }
    if(SPI_Packet[5] <0x7E) {Serial.print("Right Joystick Left "); Serial.println(128-SPI_Packet[5]);}
    else if(SPI_Packet[5] >0x82 ) {Serial.print("Right Joystick Right "); Serial.println(SPI_Packet[5]-128);}
    if(SPI_Packet[6] <0x7E) {Serial.print("Right Joystick Up "); Serial.println(128-SPI_Packet[6]);}
    else if(SPI_Packet[6] >0x82 ) {Serial.print("Right Joystick Down "); Serial.println(SPI_Packet[6]-128);}
    if(SPI_Packet[7] <0x7E) {Serial.print("Left Joystick Left "); Serial.println(128-SPI_Packet[7]);}
    else if(SPI_Packet[7] >0x82 ) {Serial.print("Left Joystick Right "); Serial.println(SPI_Packet[7]-128);}
    if(SPI_Packet[8] <0x7E) {Serial.print("Left Joystick Up "); Serial.println(128-SPI_Packet[8]);}
    else if(SPI_Packet[8] >0x82 ) {Serial.print("Left Joystick Down "); Serial.println(SPI_Packet[8]-128);}
    {
      
    }
  }
//  for (uint8_t i = 0; i < BufferSize; i++)
//  {
//    //Serial.print(SPI_Packet[i],HEX);      // Not use due to variable Print size
//    Serial.print((SPI_Packet[i] & 0xF0) >> 4, HEX);
//    Serial.print(SPI_Packet[i] & 0x0F, HEX);
//    if (i < (BufferSize - 1) ) Serial.print(",");
//    else Serial.println("");
//  }
  delay(100);
}


void PS2_SPI_SEND(byte* Data, uint8_t DataSize)
{
  uint8_t i = 0;
  digitalWrite(SlaveSelect, LOW);   // Set Attention Line Low at Start of Packet
  while (i < DataSize)            // Block for 450 Microseconds ( (40 + 10) x 9)
  {
    SPI_Packet[i] = SPI.transfer(Data[i]);
    delayMicroseconds(16);          //Delay 10 Microseconds (Seems Compulsory)
    i = i + 1;
  } // Please take note PS2 data retrieval takes 450 microseconds , may intefere Stepper motor control timing.
  digitalWrite(SlaveSelect, HIGH);  // Set Attention Line High after End of Packet
}
