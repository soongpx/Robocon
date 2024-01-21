//Locomotion
#define wheel1 2 //upper left
#define wheel2 3 //upper right
#define wheel3 4 //upper left
#define wheel4 5 //upper right
#define dir_1 0 //upper left direction
#define dir_2 1 //upper right direction
#define dir_3 2 //upper left direction
#define dir_4 3 //upper right direction

//Mechanism
#define tilt_pwm 44
#define tilt_dir 30
#define spin_pwm 13
#define spin_dir 29
#define extend_pwm 44
#define extend_dir 30
#define sampletime 0 //sample time for PID in ms

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
  TriangleMove,
  Move_Forward,
  Move_Backward,
  Move_Left,
  Move_Right,
  Rotate_Left,
  Rotate_Right
} state;
