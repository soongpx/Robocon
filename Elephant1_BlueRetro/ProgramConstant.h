//Mechanism
#define lift_pwm 3
#define lift_dir 23

#define pneumatic 24

#define conveyor_pwm 2
#define conveyor_dir 22

//PS4
//13,12,11,10 (Uno)
//52,50,51,53 (Mega)
#define SPI_CLK 52
#define SPI_MISO 50
#define SPI_MOSI 51
#define SlaveSelect 53
#define SlaveAck 2
#define BufferSize 9



enum
{
  USB_Detect_Hold,
  WaitStart,
  CheckActionToDo,
  CircleMove,
  RectangleMove,
  CrossMove,
  Move_Forward,
  Move_Backward,
  Move_Left,
  Move_Right,
  Rotate_Left,
  Rotate_Right
} state;
