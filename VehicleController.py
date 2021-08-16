#!/usr/bin/python
# -*- coding: utf-8 -*-


import time
import math
import smbus
import RPi.GPIO as GPIO

Dir = [
    'forward',
    'backward',
]

class PCA9685:

  # Registers/etc.
  __SUBADR1            = 0x02
  __SUBADR2            = 0x03
  __SUBADR3            = 0x04
  __MODE1              = 0x00
  __PRESCALE           = 0xFE
  __LED0_ON_L          = 0x06
  __LED0_ON_H          = 0x07
  __LED0_OFF_L         = 0x08
  __LED0_OFF_H         = 0x09
  __ALLLED_ON_L        = 0xFA
  __ALLLED_ON_H        = 0xFB
  __ALLLED_OFF_L       = 0xFC
  __ALLLED_OFF_H       = 0xFD

  def __init__(self, address, debug=False):
    self.bus = smbus.SMBus(1)
    self.address = address
    self.debug = debug
    if (self.debug):
      print("Reseting PCA9685")
    self.write(self.__MODE1, 0x00)

  def write(self, reg, value):
    "Writes an 8-bit value to the specified register/address"
    self.bus.write_byte_data(self.address, reg, value)
    if (self.debug):
      print("I2C: Write 0x%02X to register 0x%02X" % (value, reg))

  def read(self, reg):
    "Read an unsigned byte from the I2C device"
    result = self.bus.read_byte_data(self.address, reg)
    if (self.debug):
      print("I2C: Device 0x%02X returned 0x%02X from reg 0x%02X" % (self.address, result & 0xFF, reg))
    return result

  def setPWMFreq(self, freq):
    "Sets the PWM frequency"
    prescaleval = 25000000.0    # 25MHz
    prescaleval /= 4096.0       # 12-bit
    prescaleval /= float(freq)
    prescaleval -= 1.0
    if (self.debug):
      print("Setting PWM frequency to %d Hz" % freq)
      print("Estimated pre-scale: %d" % prescaleval)
    prescale = math.floor(prescaleval + 0.5)
    if (self.debug):
      print("Final pre-scale: %d" % prescale)

    oldmode = self.read(self.__MODE1)
    newmode = (oldmode & 0x7F) | 0x10        # sleep
    self.write(self.__MODE1, newmode)        # go to sleep
    self.write(self.__PRESCALE, int(math.floor(prescale)))
    self.write(self.__MODE1, oldmode)
    time.sleep(0.005)
    self.write(self.__MODE1, oldmode | 0x80)

  def setPWM(self, channel, on, off):
    "Sets a single PWM channel"
    self.write(self.__LED0_ON_L + 4*channel, on & 0xFF)
    self.write(self.__LED0_ON_H + 4*channel, on >> 8)
    self.write(self.__LED0_OFF_L + 4*channel, off & 0xFF)
    self.write(self.__LED0_OFF_H + 4*channel, off >> 8)
    if (self.debug):
      print("channel: %d  LED_ON: %d LED_OFF: %d" % (channel,on,off))

  def setDutycycle(self, channel, pulse):
    self.setPWM(channel, 0, int(pulse * (4096 / 100)))

  def setLevel(self, channel, value):
    if (value == 1):
      self.setPWM(channel, 0, 4095)
    else:
      self.setPWM(channel, 0, 0)
  



# 控制机器人库
class VehicleController():
    def __init__(self):
        self.PWMA = 0
        self.AIN1 = 2
        self.AIN2 = 1

        self.PWMB = 5
        self.BIN1 = 3
        self.BIN2 = 4

        self.PWMC = 6
        self.CIN2 = 7
        self.CIN1 = 8

        self.PWMD = 11
        self.DIN1 = 25
        self.DIN2 = 24
        self.pwm = PCA9685(0x40, debug=False)
        self.pwm.setPWMFreq(50)
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.DIN1,GPIO.OUT)
        GPIO.setup(self.DIN2,GPIO.OUT)

    def MotorRun(self, motor, index, speed):
        if speed > 100:
            return
        if(motor == 0):
            self.pwm.setDutycycle(self.PWMA, speed)
            if(index == Dir[0]):
                self.pwm.setLevel(self.AIN1, 0)
                self.pwm.setLevel(self.AIN2, 1)
            else:
                self.pwm.setLevel(self.AIN1, 1)
                self.pwm.setLevel(self.AIN2, 0)
        elif(motor == 1):
            self.pwm.setDutycycle(self.PWMB, speed)
            if(index == Dir[0]):
                self.pwm.setLevel(self.BIN1, 1)
                self.pwm.setLevel(self.BIN2, 0)
            else:
                self.pwm.setLevel(self.BIN1, 0)
                self.pwm.setLevel(self.BIN2, 1)
        elif(motor == 2):
            self.pwm.setDutycycle(self.PWMC,speed)
            if(index == Dir[0]):
                self.pwm.setLevel(self.CIN1,1)
                self.pwm.setLevel(self.CIN2,0)
            else:
                self.pwm.setLevel(self.CIN1,0)
                self.pwm.setLevel(self.CIN2,1)
        elif(motor == 3):
            self.pwm.setDutycycle(self.PWMD,speed)
            if (index == Dir[0]):
                GPIO.output(self.DIN1,0)
                GPIO.output(self.DIN2,1)
            else:
                GPIO.output(self.DIN1,1)
                GPIO.output(self.DIN2,0)

    def MotorStop(self, motor):
        if (motor == 0):
            self.pwm.setDutycycle(self.PWMA, 0)
        elif(motor == 1):
            self.pwm.setDutycycle(self.PWMB, 0)
        elif(motor == 2):
            self.pwm.setDutycycle(self.PWMC, 0)
        elif(motor == 3):
            self.pwm.setDutycycle(self.PWMD, 0)
    # 前进
    def t_up(self,speed,t_time):
        self.MotorRun(0,'forward',speed)
        self.MotorRun(1,'forward',speed)
        self.MotorRun(2,'forward',speed)
        self.MotorRun(3,'forward',speed)
        time.sleep(t_time)
    #后退
    def t_down(self,speed,t_time):
        self.MotorRun(0,'backward',speed)
        self.MotorRun(1,'backward',speed)
        self.MotorRun(2,'backward',speed)
        self.MotorRun(3,'backward',speed)
        time.sleep(t_time)

    # 左移
    def moveLeft(self,speed,t_time):
        self.MotorRun(0,'backward',speed)
        self.MotorRun(1,'forward',speed)
        self.MotorRun(2,'forward',speed)
        self.MotorRun(3,'backward',speed)
        time.sleep(t_time)

    #右移
    def moveRight(self,speed,t_time):
        self.MotorRun(0,'forward',speed)
        self.MotorRun(1,'backward',speed)
        self.MotorRun(2,'backward',speed)
        self.MotorRun(3,'forward',speed)
        time.sleep(t_time)

    # 左转
    def turnLeft(self,speed,t_time):
        self.MotorRun(0,'backward',speed)
        self.MotorRun(1,'forward',speed)
        self.MotorRun(2,'backward',speed)
        self.MotorRun(3,'forward',speed)
        time.sleep(t_time)
    
    # 右转
    def turnRight(self,speed,t_time):
        self.MotorRun(0,'forward',speed)
        self.MotorRun(1,'backward',speed)
        self.MotorRun(2,'forward',speed)
        self.MotorRun(3,'backward',speed)
        time.sleep(t_time)
    
    # 前左斜
    def forward_Left(self,speed,t_time):
        self.MotorStop(0)
        self.MotorRun(1,'forward',speed)
        self.MotorRun(2,'forward',speed)
        self.MotorStop(0)
        time.sleep(t_time)

    # 前右斜
    def forward_Right(self,speed,t_time):
        self.MotorRun(0,'forward',speed)
        self.MotorStop(1)
        self.MotorStop(2)
        self.MotorRun(3,'forward',speed)
        time.sleep(t_time)

    # 后左斜
    def backward_Left(self,speed,t_time):
        self.MotorRun(0,'backward',speed)
        self.MotorStop(1)
        self.MotorStop(2)
        self.MotorRun(3,'backward',speed)
        time.sleep(t_time)
    
    # 后右斜
    def backward_Right(self,speed,t_time):
        self.MotorStop(0)
        self.MotorRun(1,'backward',speed)
        self.MotorRun(2,'backward',speed)
        self.MotorStop(3)
        time.sleep(t_time)


    # 停止
    def t_stop(self,t_time):
        self.MotorStop(0)
        self.MotorStop(1)
        self.MotorStop(2)
        self.MotorStop(3)
        time.sleep(t_time)

        # 辅助功能，使设置舵机脉冲宽度更简单。
    def set_servo_pulse(self,channel,pulse):
        pulse_length = 1000000    # 1,000,000 us per second
        pulse_length //= 60       # 60 Hz
        print('{0}us per period'.format(pulse_length))
        pulse_length //= 4096     # 12 bits of resolution
        print('{0}us per bit'.format(pulse_length))
        pulse *= 1000
        pulse //= pulse_length
        self.pwm.setPWM(channel, 0, pulse)

    # 设置舵机角度函数  
    def set_servo_angle(self,channel,angle):
        angle=4096*((angle*11)+500)/20000
        self.pwm.setPWM(channel,0,int(angle))


from enum import Enum
from typing import Tuple

# idle -> accelerate -> idle
# idle -> decelerate -> idle
# accelerate -> decelerate -> accelerate
# idle -> reverse -> accelerate -> idle (only if speed is Zero)
class EngineStatus(Enum):
    IDLE=0,
    ACCELERATE=1,
    DECELERATE=2,
    REVERSE=3,

class VehicleModel():
    speed_range = (-99, 99)
    speed = 0
    steeringwheel_angle_range = (-540, 540)  # angle > 0 means turn left, counterclockwise
    steeringwheel_angle = 0
    steeringwheel_ratio = 0. #[-1, 1]
    worker = None
    engine_status = EngineStatus.IDLE

    def __init__(self, vc:VehicleController) -> None:
        self.vc = vc
        self.start_loop()

    def start_loop(self):
        self.worker = Worker(vm=self)
        global loop_flag
        loop_flag = True
        self.worker.start()

        print('start vehicle-side worker thread')

    def stop_loop(self):
        global loop_flag
        loop_flag = False

        print('stop vehicle-side worker thread')

    """
    Speed Acceleration and Deceleration Procedure Simulation Functions
    """
    def set_speed(self, _speed):
        self.speed = _speed

        self.steeringwheel_ratio = self.steering_wheel_angle_mapping_fn(self.steeringwheel_angle)
        spd0, spd1, spd2, spd3 = self.steering_wheel_ratio_take_effect(self.steeringwheel_ratio, self.speed, self.engine_status)

        print(f'[{self.engine_status}] tspeed:[{self.speed}] | real_speed:<{(spd0, spd1, spd2, spd3)}> | sw:{self.steeringwheel_angle}->{self.steeringwheel_ratio}')
        """
        if _speed < 0:
            # backward
            self.vc.t_down(-_speed, 0) 
        else:
            # forward
            self.vc.t_up(_speed, 0)
        """
        self.vc.MotorRun(0, 'forward' if spd0 > 0 else 'backward', abs(spd0)) # front left
        self.vc.MotorRun(1, 'forward' if spd1 > 0 else 'backward', abs(spd1)) # front right
        self.vc.MotorRun(2, 'forward' if spd2 > 0 else 'backward', abs(spd2)) # back left
        self.vc.MotorRun(3, 'forward' if spd3 > 0 else 'backward', abs(spd3)) # back right
    
    def accelerate(self, max_speed:int=99):
        self.engine_status = EngineStatus.ACCELERATE
        self.speed_range = (self.speed_range[0], max_speed)
    
    def decelerate(self):
        self.engine_status = EngineStatus.DECELERATE
    
    def reverse(self, max_speed:int=-99):
        self.engine_status = EngineStatus.REVERSE
        self.speed_range = (max_speed, self.speed_range[1])

    def idle(self):
        self.engine_status = EngineStatus.IDLE

    """
    Steering Wheel Related Functions
    """    
    def steering_wheel_angle_mapping_fn(self, sw_angle: float):
        def mapping_fn(sw_angle_normalized: float) -> float:
            x = sw_angle_normalized
            y = x**3
            return y
        def normalize_sw_angle(sw_angle: float) -> float:
            return sw_angle / (self.steeringwheel_angle_range[1] if sw_angle >= 0 else abs(self.steeringwheel_angle_range[0]))

        # normalize the original angle
        normalized_angle = normalize_sw_angle(sw_angle)
        # covert it to a magical ratio which will put some effects on four wheels
        angle_ratio = mapping_fn(normalized_angle)
        return angle_ratio

    def steering_wheel_ratio_take_effect(self, angle_ratio:float, target_speed: int, target_engine_status: EngineStatus) -> Tuple[int, int, int, int]:
        # print(f'current ratio:{angle_ratio} | target_speed:{target_speed} | target_engine_status:{target_engine_status}')
        """
        clbrobot.MotorRun(0, 'forward' if spd0 > 0 else 'backward', abs(spd0))
        clbrobot.MotorRun(1, 'forward' if spd1 > 0 else 'backward', abs(spd1))
        clbrobot.MotorRun(2, 'forward' if spd2 > 0 else 'backward', abs(spd2))
        clbrobot.MotorRun(3, 'forward' if spd3 > 0 else 'backward', abs(spd3))
        """
        spd0 = target_speed if angle_ratio < 0 else int(target_speed* (1 - angle_ratio))
        spd1 = target_speed if angle_ratio > 0 else int(target_speed* (1 + angle_ratio))
        spd2 = target_speed if angle_ratio < 0 else int(target_speed* (1 - angle_ratio))
        spd3 = target_speed if angle_ratio > 0 else int(target_speed* (1 + angle_ratio))
        return spd0,spd1,spd2,spd3

import threading

loop_flag = True
class Worker(threading.Thread):
    hz = 60 # frequence
    prev_ts = 0
    
    acceleration = 33 # top speed 100, current acceleration is 30 units/sec
    brake_deceleration = 50
    reverse_acceleration = 25
    idle_deceleration = 20
    speed = int(0)
    # speed_range = None
    steeringwheel_angle = 0
    steeringwheel_angle_range = None

    def __init__(self, vm: VehicleModel) -> None:
        super().__init__()
        self.prev_ts = time.time()
        self.t_span = 1 / self.hz
        self.vm = vm
        # self.speed_range = vm.speed_range
        self.steeringwheel_angle_range = vm.steeringwheel_angle_range

    def run(self) -> None:
        accu_ts = 0.

        while loop_flag:
            curr_ts = time.time()
            span_ts = curr_ts - self.prev_ts
            accu_ts += span_ts
            self.prev_ts = curr_ts

            if accu_ts >= self.t_span:
                if self.vm.engine_status == EngineStatus.ACCELERATE:
                    target_speed = int(self.speed+accu_ts*self.acceleration)
                    max_speed = self.vm.speed_range[1]
                    self.speed = max_speed if target_speed > max_speed else target_speed
                elif self.vm.engine_status == EngineStatus.DECELERATE:
                    target_speed = int(self.speed-accu_ts*self.brake_deceleration)
                    self.speed = 0 if target_speed < 0 else target_speed
                elif self.vm.engine_status == EngineStatus.REVERSE:
                    target_speed = int(self.speed-accu_ts*self.reverse_acceleration)
                    max_speed = self.vm.speed_range[0]
                    self.speed = max_speed if target_speed < max_speed else target_speed
                else: # self.vm.engine_status == EngineStatus.IDLE:
                    if self.speed >= 0:
                        # moving forward
                        target_speed = int(self.speed-accu_ts*self.idle_deceleration)
                        self.speed = 0 if target_speed < 0 else target_speed
                    else:
                        # speed below zero
                        # moving backward
                        target_speed = int(self.speed+accu_ts*self.idle_deceleration)
                        self.speed = 0 if target_speed > 0 else target_speed

                # transfer the new speed ...
                # print(f'current: {time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())} ==> {self.speed} {self.vm.speed_range} ==> {self.vm.engine_status}')
                self.vm.set_speed(self.speed)

                # print(f'current timestamp: {curr_ts} ==> {accu_ts} >= {self.t_span}')
                accu_ts = 0.