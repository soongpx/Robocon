#include <PID_v1.h>
//dont use pin 10-13
//Locomotion
#define wheel1 2 //upper left
#define wheel2 3 //upper right
#define wheel3 4 //lower left
#define wheel4 7 //lower right
#define dir_1 0 //upper left direction
#define dir_2 1 //upper right direction
#define dir_3 2 //lower left direction
#define dir_4 3 //lower right direction

#define lifting_pwm_pin 9
#define lifting_dir_pin 5
#define lifting_speed 200 

#define loading_pwm_pin 10
#define loading_dir_pin 6
  
#define min_speed
#define max_speed
int motor_Speed;

#define sampletime 0 //sample time  in ms
double Kp = 3.05, Ki = 0, Kd = 0.5;
double Input, Output,setpoint;
double yaw_angle;
int suss,fail;

unsigned char Re_buf[11], counter = 0;
unsigned char sign = 0;
bool aligned = false;
bool stopped = false;
bool moving = false;
bool self_align_on = true;
/*********/

#include <PS4BT.h>
#include <usbhub.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>
//USB Set UP
USB Usb;
USBHub Hub1(&Usb); // Some dongles have a hub inside
BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so

/* You can create the instance of the PS4BT class in two ways */
// This will start an inquiry and then pair with the PS4 controller - you only have to do this once
// You will need to hold down the PS and Share button at the same time, the PS4 controller will then start to blink rapidly indicating that it is in pairing mode
PS4BT PS4(&Btd, PAIR);

// After that you can simply create the instance like so and then press the PS button on the device
//PS4BT PS4(&Btd);

bool printAngle, printTouch;
uint8_t oldL2Value, oldR2Value;

int lx = 0;
int ly = 0;

PID myPID(&Input, &Output, &setpoint, Kp, Ki, Kd, DIRECT);

int capPWMValue(int final_pwm_value)
{
  if(final_pwm_value <=0 )
    return 0;
  else
    return (final_pwm_value > 255)? 255 : final_pwm_value;
}

void dpadmovement(int motor_Speed)
{
      if (PS4.getButtonPress(UP)) { //when Up button is pressed(leftside of the controller), the light of the controller will light up in red colour
        Forward(capPWMValue(motor_Speed));
        //Serial.println("Forward");
      }else if (PS4.getButtonPress(RIGHT)) { //when Right button is pressed, the light of the controller will light up in blue colour
        Right(capPWMValue(motor_Speed));
        //Serial.println("Right");
      }else if (PS4.getButtonPress(DOWN)) { //when Down button is pressed, the light of the controller will light up in yellow colour
         Backward(capPWMValue(motor_Speed));
         //Serial.println("Backward");
      }else if (PS4.getButtonPress(LEFT)) { //when left button is pressed, the light of the controller will light up in green colour
        Left(capPWMValue(motor_Speed));
        //Serial.println("Left");
      }else if (PS4.getButtonPress(L1)){ //when the L1 button is pressed, it will serial print "L1"
        RotateLeft(capPWMValue(motor_Speed));
      }else if (PS4.getButtonPress(R1)){ //when the R1 button is pressed, it will serial print "R1"
        RotateRight(capPWMValue(motor_Speed));
      }else if (PS4.getButtonPress(CROSS)){ 
        Lifting(200);
      }else if (PS4.getButtonPress(CIRCLE)){ 
        Lower(200);
      }else if (PS4.getButtonPress(SQUARE)){ 
        loading();
      }
      else if (PS4.getButtonPress(TRIANGLE)){ 
        resetYawAngle();
      }else{
        MotorStopping();
        unload();
      }   
}

void joyStickMovement()
{
  if(PS4.getButtonPress(L2))
  {
    if ( PS4.getAnalogHat(LeftHatY) <= 117) {
      ly = map( PS4.getAnalogHat(LeftHatY), 117, 0, 0, 230);
    }
    else if ( PS4.getAnalogHat(LeftHatY) >= 137){
      ly = map( PS4.getAnalogHat(LeftHatY), 137, 255, 0, -230);
    }
    else{
      ly = 0;
    }
    if ( PS4.getAnalogHat(LeftHatX) <= 117) {
      lx = map( PS4.getAnalogHat(LeftHatX), 117, 0, 0, -230);
    }
    else if ( PS4.getAnalogHat(LeftHatX) >= 137){
      lx = map( PS4.getAnalogHat(LeftHatX), 137, 255, 0, 230);
    }
    else{
      lx = 0;
    }
    if (ly > 0 && lx < 10 && lx > -10 )
    {
      Forward(ly);
    }
  
    else if (ly < 0 && lx < 10 && lx > -10 )
    {
      Backward(-ly);
    }
  
    else if (lx > 0 && ly < 10 && ly > -10 )
    {
      Right(lx);
    }
  
    else if (lx < 0 && ly < 10 && ly > -10 )
    {
      Left(-lx);
    }
  
    else if (lx > 10 && ly > 10)
    {
      int pwm_out = lx > ly? lx:ly;
      ForwardRight(pwm_out);
    }
  
    else if (lx < -10 && ly > 10)
    {
      int pwm_out = -lx > ly? -lx:ly;
      ForwardLeft(pwm_out);
    }
  
    else if (lx < -10 && ly < -10)
    {
      int pwm_out = -lx > -ly? -lx:-ly;
      BackwardLeft(pwm_out);
    }
  
    else if (lx > 10 && ly < -10)
    {
      int pwm_out = lx > -ly? lx:-ly;
      BackwardRight(pwm_out);
    }
  
    else if (lx == 0 && ly == 0)
    {
      MotorStopping();
    }
  }
}


void RotateLeft(int motor_speed)
{
  PORTA &= ~(1 << dir_1);
  PORTA &= ~(1 << dir_2);
  PORTA &= ~(1 << dir_3);
  PORTA &= ~(1 << dir_4);
  analogWrite(wheel1, motor_speed);
  analogWrite(wheel2, motor_speed);
  analogWrite(wheel3, motor_speed);
  analogWrite(wheel4, motor_speed);
}

void RotateRight(int motor_speed)
{
  PORTA |= (1 << dir_1);
  PORTA |= (1 << dir_2);
  PORTA |= (1 << dir_3);
  PORTA |= (1 << dir_4);
  analogWrite(wheel1, motor_speed);
  analogWrite(wheel2, motor_speed);
  analogWrite(wheel3, motor_speed);
  analogWrite(wheel4, motor_speed);
}

void Forward(int pwm)
{
  PORTA |= (1 << dir_1);
  PORTA |= (1 << dir_2);
  PORTA |= (1 << dir_3);
  PORTA |= (1 << dir_4);
  //aligned = false;

  if (Output > 0)//deviate to right
  {
    analogWrite(wheel1, capPWMValue(pwm - Output));
    analogWrite(wheel2, capPWMValue(pwm + Output));
    analogWrite(wheel3, capPWMValue(pwm - Output));
    analogWrite(wheel4, capPWMValue(pwm + Output));
  }
  else if (Output < 0)//deviate to left
  {
    analogWrite(wheel1, capPWMValue(pwm - Output));
    analogWrite(wheel2, capPWMValue(pwm + Output));
    analogWrite(wheel3, capPWMValue(pwm - Output));
    analogWrite(wheel4, capPWMValue(pwm + Output));
  }
  else //Move forward without turning sides
  {
    analogWrite(wheel1, pwm);
    analogWrite(wheel2, pwm);
    analogWrite(wheel3, pwm);
    analogWrite(wheel4, pwm);
  }
}

void Backward(int pwm)
{
  PORTA &= ~(1 << dir_1);
  PORTA &= ~(1 << dir_2);
  PORTA &= ~(1 << dir_3);
  PORTA &= ~(1 << dir_4);
  //aligned = false;

  if (Output > 0)//deviate to right
  {
    analogWrite(wheel1, capPWMValue(pwm + Output));
    analogWrite(wheel2, capPWMValue(pwm - Output));
    analogWrite(wheel3, capPWMValue(pwm + Output));
    analogWrite(wheel4, capPWMValue(pwm - Output));
  }
  else if (Output < 0)//deviate to left
  {
    analogWrite(wheel1, capPWMValue(pwm + Output));
    analogWrite(wheel2, capPWMValue(pwm - Output));
    analogWrite(wheel3, capPWMValue(pwm + Output));
    analogWrite(wheel4, capPWMValue(pwm - Output));
  }

//Move forward 0 degree.
  else
  {
    analogWrite(wheel1, pwm);
    analogWrite(wheel2, pwm);
    analogWrite(wheel3, pwm);
    analogWrite(wheel4, pwm);
  }
}

void Left(int pwm)
{
  PORTA &= ~(1 << dir_1);
  PORTA |= (1 << dir_2);
  PORTA |= (1 << dir_3);
  PORTA &= ~(1 << dir_4);
  //aligned = false;

  if (Output > 0)//deviate to right
  {
    analogWrite(wheel1, capPWMValue(pwm + Output));
    analogWrite(wheel2, capPWMValue(pwm + Output));
    analogWrite(wheel3, capPWMValue(pwm - Output));
    analogWrite(wheel4, capPWMValue(pwm - Output));
  }
  else if (Output < 0)//deviate to left
  {
    analogWrite(wheel1, capPWMValue(pwm + Output));
    analogWrite(wheel2, capPWMValue(pwm + Output));
    analogWrite(wheel3, capPWMValue(pwm - Output));
    analogWrite(wheel4, capPWMValue(pwm - Output));
  }

//Move forward 0 degree.
  else
  {
    analogWrite(wheel1, pwm);
    analogWrite(wheel2, pwm);
    analogWrite(wheel3, pwm);
    analogWrite(wheel4, pwm);
  }
}

void Right(int pwm)
{
  PORTA |= (1 << dir_1);
  PORTA &= ~(1 << dir_2);
  PORTA &= ~(1 << dir_3);
  PORTA |= (1 << dir_4);
  //aligned = false;

  if (Output > 0)//deviate to right
  {
    analogWrite(wheel1, capPWMValue(pwm - Output));
    analogWrite(wheel2, capPWMValue(pwm - Output));
    analogWrite(wheel3, capPWMValue(pwm + Output));
    analogWrite(wheel4, capPWMValue(pwm + Output));
  }
  else if (Output < 0)//deviate to left
  {
    analogWrite(wheel1, capPWMValue(pwm - Output));
    analogWrite(wheel2, capPWMValue(pwm - Output));
    analogWrite(wheel3, capPWMValue(pwm + Output));
    analogWrite(wheel4, capPWMValue(pwm + Output));
  }

//Move forward 0 degree.
  else
  {
    analogWrite(wheel1, pwm);
    analogWrite(wheel2, pwm);
    analogWrite(wheel3, pwm);
    analogWrite(wheel4, pwm);
  }
}

void ForwardLeft(int pwm)
{
  PORTA |= (1 << dir_1);
  PORTA &= ~(1 << dir_2);
  PORTA |= (1 << dir_3);
  PORTA &= ~(1 << dir_4);
  //aligned = false;

  if (Output > 0)//deviate to right
  {
    analogWrite(wheel1, 0);
    analogWrite(wheel2, capPWMValue(pwm + Output));
    analogWrite(wheel3, capPWMValue(pwm - Output));
    analogWrite(wheel4, 0);
  }
  else if (Output < 0)//deviate to left
  {
    analogWrite(wheel1, 0);
    analogWrite(wheel2, capPWMValue(pwm + Output));
    analogWrite(wheel3, capPWMValue(pwm - Output));
    analogWrite(wheel4, 0);
  }

//Move forward 0 degree.
  else
  {
    analogWrite(wheel1, 0);
    analogWrite(wheel2, pwm);
    analogWrite(wheel3, pwm);
    analogWrite(wheel4, 0);
  }
}

void ForwardRight(int pwm)
{
  PORTA |= (1 << dir_1);
  PORTA &= ~(1 << dir_2);
  PORTA |= (1 << dir_3);
  PORTA &= ~(1 << dir_4);
  //aligned = false;

  if (Output > 0)//deviate to right
  {
    analogWrite(wheel1, capPWMValue(pwm - Output));
    analogWrite(wheel2, 0);
    analogWrite(wheel3, 0);
    analogWrite(wheel4, capPWMValue(pwm + Output));
  }
  else if (Output < 0)//deviate to left
  {
    analogWrite(wheel1, capPWMValue(pwm - Output));
    analogWrite(wheel2, 0);
    analogWrite(wheel3, 0);
    analogWrite(wheel4, capPWMValue(pwm + Output));
  }

//Move forward 0 degree.
  else
  {
    analogWrite(wheel1, pwm);
    analogWrite(wheel2, 0);
    analogWrite(wheel3, 0);
    analogWrite(wheel4, pwm);
  }
}

void BackwardLeft(int pwm)
{
  PORTA &= ~(1 << dir_1);
  PORTA &= ~(1 << dir_2);
  PORTA &= ~(1 << dir_3);
  PORTA |= (1 << dir_4);
  //aligned = false;

  if (Output > 0)//deviate to right
  {
    analogWrite(wheel1, capPWMValue(pwm + Output));
    analogWrite(wheel2, 0);
    analogWrite(wheel3, 0);
    analogWrite(wheel4, capPWMValue(pwm - Output));
  }
  else if (Output < 0)//deviate to left
  {
    analogWrite(wheel1, capPWMValue(pwm + Output));
    analogWrite(wheel2, 0);
    analogWrite(wheel3, 0);
    analogWrite(wheel4, capPWMValue(pwm - Output));
  }

  else
  {
    analogWrite(wheel1, pwm);
    analogWrite(wheel2, 0);
    analogWrite(wheel3, 0);
    analogWrite(wheel4, pwm);
  }
}

void BackwardRight(int pwm)
{
  PORTA &= ~(1 << dir_1);
  PORTA |= (1 << dir_2);
  PORTA &= ~(1 << dir_3);
  PORTA |= (1 << dir_4);
  //aligned = false;

  if (Output > 0)//deviate to right
  {
    analogWrite(wheel1, 0);
    analogWrite(wheel2, capPWMValue(pwm - Output));
    analogWrite(wheel3, capPWMValue(pwm + Output));
    analogWrite(wheel4, 0);
  }
  else if (Output < 0)//deviate to left
  {
    analogWrite(wheel1, 0);
    analogWrite(wheel2, capPWMValue(pwm - Output));
    analogWrite(wheel3, capPWMValue(pwm + Output));
    analogWrite(wheel4, 0);
  }

//Move forward 0 degree.
  else
  {
    analogWrite(wheel1, 0);
    analogWrite(wheel2, pwm);
    analogWrite(wheel3, pwm);
    analogWrite(wheel4, 0);
  }
}

void MotorStopping()
{
  analogWrite(wheel1, 0);
  analogWrite(wheel2, 0);
  analogWrite(wheel3, 0);
  analogWrite(wheel4, 0);
  analogWrite(lifting_pwm_pin, 0);
}

void Lifting(int pwm)
{
    PORTA |= (1 << lifting_dir_pin);
    analogWrite(lifting_pwm_pin, pwm);
}

void Lower(int pwm)
{
    PORTA &= ~ (1 << lifting_dir_pin);
    analogWrite(lifting_pwm_pin, pwm);
}

void loading()
{
   PORTA |= (1 << loading_dir_pin);
   Serial.println("hi");
}

void unload()
{
   PORTA &= ~(1 << loading_dir_pin);
   Serial.println("unload");
}

//IMU
void serialEvent1()
{
  while (Serial1.available())
  {
    Re_buf[counter] = (unsigned char)Serial1.read();
    // The fist byte is not the packet header,
    // skip to wait till the packet header to arrive
    if (counter == 0 && Re_buf[0] != 0x55) return;
    counter++;
    if (counter == 11) // All packet received, bool sign = true
    {
      counter = 0;
      sign = 1;
    }
  }
}

void calYawAngle() {
  if (sign) {
    sign = 0;
    if (Re_buf[0] == 0x55)
    {
      uint16_t sum = 0;

      for (int x = 0; x < 10; x++)
      {
        sum += Re_buf[x];   
      }

      while (sum > 256)
      {
        sum -= 256;
      }
      
      if ( sum == Re_buf[10] && Re_buf[1] == 83 ) {                       //0x53
        yaw_angle = (short(Re_buf [7] << 8 | Re_buf [6]) / 32768.0 * 180);
        Serial.print("Current Angle:");
        Serial.println(yaw_angle);
        suss++;
        //Serial.print("Failed: ");
        //Serial.println(fail / suss * 100);
        return;
      }
      else if (sum != Re_buf[10])
      {
        suss++;
        fail++;
        //Serial.print("Failed:  ");
        //Serial.println(fail / suss * 100);
        return;
      }
    }
  }
}

void resetYawAngle() {
  MotorStopping();
  Serial1.write(0xFF);
  Serial1.write(0xAA);
  Serial1.write(0x52);
}

void PS4Setup(){
  Serial.begin(115200);

#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1)
  {
    Serial.print(F("\r\nOSC did not start"));
    while (1); // Halt
  }
  Serial.print(F("\r\nPS4 Bluetooth Library Started"));
}

void IMUSetup()
{
  Serial1.begin(115200);
  Serial1.write(0xFF);
  Serial1.write(0xAA);
  Serial1.write(0x52); //0xFF 0xAA 0x52 is command for initializing angle in z Direction =0
}

void PIDSetup()
{
  Input = yaw_angle;
  setpoint = 0;
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(sampletime);
  myPID.SetOutputLimits(-25500, 25500);
  DDRA = B00001111;
  PORTA = B00000000;
  MotorStopping();
}

void setup()
{
  PS4Setup();
  IMUSetup();
  PIDSetup();
  DDRA = B00001111;
  pinMode(wheel1,OUTPUT);
  pinMode(wheel2,OUTPUT);
  pinMode(wheel3,OUTPUT);
  pinMode(wheel4,OUTPUT);

  MotorStopping();
}

void loop() {
  Usb.Task();
  motor_Speed = 120;    //for dpad movement
  calYawAngle();
  Input = yaw_angle;
  myPID.Compute();  
  /*#ifdef PIDTUNING
   Serial.println(yaw_angle);
   Serial.print(",");
   Serial.print(Output>>7);   //output*128
   Serial.print(",");
   Serial.println();
  #endif*/
  
  if (PS4.connected()) {
    if (PS4.getAnalogHat(LeftHatX) > 137 || PS4.getAnalogHat(LeftHatX) < 117 || PS4.getAnalogHat(LeftHatY) > 137 || PS4.getAnalogHat(LeftHatY) < 117 ){
      joyStickMovement();
    }
    else
      dpadmovement(motor_Speed);
  }
}