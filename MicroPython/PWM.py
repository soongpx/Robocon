# from machine import Pin, PWM
# import time
# 
# # initialize the PWM object
# pwm = PWM(Pin(16))
# 
# # set the frequency of the PWM signal to 1 kHz
# pwm.freq(1000)
# 
# # gradually increase the brightness of the LED
# for duty in range(0, 65535, 100):
#     pwm.duty_u16(duty)
#     time.sleep(0.1)
# 
# # gradually decrease the brightness of the LED
# for duty in range(65535, 0, -100):
#     pwm.duty_u16(duty)
#     time.sleep(0.1)
# 
# # stop the PWM signal
# pwm.deinit()

# import machine
# import utime
# 
# # Set the GPIO pin connected to the servo as pin 16
# servo_pin = machine.Pin(16)
# 
# # Set up the PWM output on the servo pin with a frequency of 50 Hz
# pwm = machine.PWM(servo_pin)
# pwm.freq(50)
# 
# # Set the duty cycle for the servo to its middle position (1500 microseconds)
# pwm.duty_ns(1500000)
# 
# # Function to move the servo to a given angle (0-180 degrees)
# def set_servo_angle(angle):
#     # Calculate the pulse width for the given angle (500-2500 microseconds)
#     pulse_width_us = 500 + int(angle / 180 * 2000)
#     # Convert the pulse width to a duty cycle value (0-65535)
#     duty_cycle = int(pulse_width_us * 65535 / 20000)
#     # Set the duty cycle for the PWM output
#     pwm.duty_u16(duty_cycle)
# 
# # Move the servo to its minimum angle (0 degrees)
# set_servo_angle(0)
# utime.sleep(1)
# 
# # Move the servo to its maximum angle (180 degrees)
# set_servo_angle(180)
# utime.sleep(1)
# 
# # Move the servo back to its middle position (90 degrees)
# set_servo_angle(90)
# 
# # Stop the PWM output
# pwm.deinit()

from machine import Pin, PWM
import time
import utime
 
 
class Servo:
    """ A simple class for controlling a 9g servo with the Raspberry Pi Pico.
 
    Attributes:
 
        minVal: An integer denoting the minimum duty value for the servo motor.
 
        maxVal: An integer denoting the maximum duty value for the servo motor.
 
    """
 
    def __init__(self, pin: int or Pin or PWM, minVal=2500, maxVal=7500):
        """ Creates a new Servo Object.
 
        args:
 
            pin (int or machine.Pin or machine.PWM): Either an integer denoting the number of the GPIO pin or an already constructed Pin or PWM object that is connected to the servo.
 
            minVal (int): Optional, denotes the minimum duty value to be used for this servo.
 
            maxVal (int): Optional, denotes the maximum duty value to be used for this servo.
 
        """
 
        if isinstance(pin, int):
            pin = Pin(pin, Pin.OUT)
        if isinstance(pin, Pin):
            self.__pwm = PWM(pin)
        if isinstance(pin, PWM):
            self.__pwm = pin
        self.__pwm.freq(50)
        self.minVal = minVal
        self.maxVal = maxVal
 
    def deinit(self):
        """ Deinitializes the underlying PWM object.
 
        """
        self.__pwm.deinit()
 
    def goto(self, value: int):
        """ Moves the servo to the specified position.
 
        args:
 
            value (int): The position to move to, represented by a value from 0 to 1024 (inclusive).
 
        """
        if value < 0:
            value = 0
        if value > 1024:
            value = 1024
        delta = self.maxVal-self.minVal
        target = int(self.minVal + ((value / 1024) * delta))
        self.__pwm.duty_u16(target)
 
    def middle(self):
        """ Moves the servo to the middle.
        """
        self.goto(512)
 
    def free(self):
        """ Allows the servo to be moved freely.
        """
        self.__pwm.duty_u16(0)
        
# s1 = Servo(16)       # Servo pin is connected to GP0
#  
# def servo_Map(x, in_min, in_max, out_min, out_max):
#     return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
#  
# def servo_Angle(angle):
#     if angle < 0:
#         angle = 0
#     if angle > 180:
#         angle = 180
#     s1.goto(round(servo_Map(angle,0,180,0,1024))) # Convert range value to angle value
#     
# if __name__ == '__main__':
#     while True:
#         print("Turn left ...")
#         for i in range(0,180,10):
#             servo_Angle(i)
#             utime.sleep(0.05)
#         print("Turn right ...")
#         for i in range(180,0,-10):
#             servo_Angle(i)
#             utime.sleep(0.05)

servo = machine.PWM(machine.Pin(16))

servo.freq(50)

in_min = 0
in_max = 65535
out_min = 1000
out_max = 9000

while 1:
    for value in range(0,65535,10):
        Servo = (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
        servo.duty_u16(int(Servo))
    
    
