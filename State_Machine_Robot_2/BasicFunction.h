//https://www.oracle.com/technical-resources/articles/it-infrastructure/mixing-c-and-cplusplus.html

#define MotorStopping SetPWM(0)

#ifdef __cplusplus
extern "C"
#endif  
void SetPWM(int PWM_Value);

#ifdef __cplusplus
extern "C"
#endif 
void MoveFront(int pwm);

#ifdef __cplusplus
extern "C"
#endif 
void MoveBack(int  pwm);

#ifdef __cplusplus
extern "C"
#endif 
void Leftward(int pwm);

#ifdef __cplusplus
extern "C"
#endif 
void MoveRight(int pwm);

#ifdef __cplusplus
extern "C"
#endif
uint8_t Basic_Function_Counter;

#ifdef __cplusplus
extern "C"
#endif
void SerialPrintStringln(char* Text);

#ifdef __cplusplus
extern "C"
#endif
void SerialPrintString(char* Text);
