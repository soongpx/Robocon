//https://www.oracle.com/technical-resources/articles/it-infrastructure/mixing-c-and-cplusplus.html

#define MotorStopping SetPWM(0)

extern "C" void SetPWM(int PWM_Value);

#ifdef __cplusplus
extern "C"
#endif 
{
  void MoveFront(int pwm);
  extern "C" void MoveBack(int  pwm);
  extern "C" void Leftward(int pwm);
  extern "C" void MoveRight(int pwm);
}

#ifdef __cplusplus
extern "C"
#endif
uint8_t Basic_Function_Counter;

#ifdef __cplusplus
extern "C"
#endif
{
  void SerialPrintStringln(String Text);
  void SerialPrintString(String Text);
}
