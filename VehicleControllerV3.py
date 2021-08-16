#!/usr/bin/python
# -*- coding: utf-8 -*-


import time
import math

from matplotlib.pyplot import flag
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


    """
    ================================
    Motor Related Functions
    """
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
    # 后退
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
    # 右移
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


    """
    ================================
    Camera related functions
    """
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

class ClutchStatus(Enum):
    N=0,
    F1=1,
    F2=2,
    F3=3,
    F4=4,
    F5=5,
    F6=6,
    R=-1,

class VehicleModel():
    speed_range = (-99, 99)
    speed = 0
    steeringwheel_angle_range = (-450, 450)  # angle > 0 means turn left, counterclockwise
    steeringwheel_angle = 0
    steeringwheel_ratio = 0. #[-1, 1]
    worker = None
    engine_status = EngineStatus.IDLE

    clutch_status=ClutchStatus.N
    throttle_value=0
    brake_value=0

    def __init__(self, vc:VehicleController) -> None:
        self.vc = vc
        self.start_loop()
        self.start_video_loop()

    def start_loop(self):
        self.worker = Worker(vm=self)
        global loop_flag
        loop_flag = True
        self.worker.start()

        print('[VehicleModel] start vehicle-side worker SPEED SIMULATION thread')

    def stop_loop(self):
        global loop_flag
        loop_flag = False

        print('[VehicleModel] stop vehicle-side worker thread')
    
    def start_video_loop(self):
        self.worker_videocapture = WorkerVideoCapture(vm=self)
        global loop_videocapture_flag
        loop_videocapture_flag = True
        self.worker_videocapture.start()

        print('[VehicleModel] start vehicle-side worker VIDEO CAPTURE thread')

    def stop_video_loop(self):
        global loop_videocapture_flag
        loop_videocapture_flag = False

        print('[VehicleModel] stop vehicle-side worker videocapture thread')


    """
    Speed Acceleration and Deceleration Procedure Simulation Functions
    """
    def set_speed(self, _speed):
        # self.speed = _speed
        self.speed = _speed if abs(_speed) >= 5 else 0

        self.steeringwheel_ratio = self.steering_wheel_angle_mapping_fn(self.steeringwheel_angle)
        spd0, spd1, spd2, spd3 = self.steering_wheel_ratio_take_effect(self.steeringwheel_ratio, self.speed, self.engine_status)

        print(f'[VehicleModel] [{self.engine_status}] tspeed:[{self.speed}] | real_speed:<{(spd0, spd1, spd2, spd3)}> | sw:{self.steeringwheel_angle}->{self.steeringwheel_ratio}')
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
    
    @DeprecationWarning
    def accelerate(self, max_speed:int=99):
        self.engine_status = EngineStatus.ACCELERATE
        self.speed_range = (self.speed_range[0], max_speed)
    
    @DeprecationWarning
    def decelerate(self):
        self.engine_status = EngineStatus.DECELERATE
    
    @DeprecationWarning
    def reverse(self, max_speed:int=-99):
        self.engine_status = EngineStatus.REVERSE
        self.speed_range = (max_speed, self.speed_range[1])

    @DeprecationWarning
    def idle(self):
        self.engine_status = EngineStatus.IDLE

    def set_sw_angle(self, angle: int):
        self.steeringwheel_angle = angle

    def set_clutch_status(self, c: ClutchStatus):
        self.clutch_status = c

    def set_throttle_value(self, t: float):
        self.throttle_value = t

    def set_brake_value(self, b: float):
        self.brake_value = b

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
        # spd0 = target_speed if angle_ratio < 0 else int(target_speed* (1 - angle_ratio))
        # spd1 = target_speed if angle_ratio > 0 else int(target_speed* (1 + angle_ratio))
        # spd2 = target_speed if angle_ratio < 0 else int(target_speed* (1 - angle_ratio))
        # spd3 = target_speed if angle_ratio > 0 else int(target_speed* (1 + angle_ratio))
        # angle_ratio < 0 turning left, else turning right
        spd1 = target_speed if angle_ratio < 0 else round(target_speed* (1 - angle_ratio))
        spd0 = target_speed if angle_ratio > 0 else round(target_speed* (1 + angle_ratio))
        spd3 = target_speed if angle_ratio < 0 else round(target_speed* (1 - angle_ratio))
        spd2 = target_speed if angle_ratio > 0 else round(target_speed* (1 + angle_ratio))
        return spd0,spd1,spd2,spd3

    """
    Camera Servo Related Functions
    """
    def reset_camera_servo(self):
        # # Configure min and max servo pulse lengths
        # servo_min = 150  # Min pulse length out of 4096
        # servo_max = 600  # Max pulse length out of 4096

        # 频率设置为50hz，适用于舵机系统。
        self.vc.set_servo_angle(15, 90)  # 底座舵机 90 
        self.vc.set_servo_angle(14, 145)  # 顶部舵机 145

    def rotate_camera_lower_servo(self, angle):
        # 频率设置为50hz，适用于舵机系统。
        self.vc.set_servo_angle(15, angle)  # 底座舵机 90 

    def rotate_camera_upper_servo(self, angle):
        # 频率设置为50hz，适用于舵机系统。
        self.vc.set_servo_angle(14, angle)  # 顶部舵机 145

import threading

loop_flag = True
class Worker(threading.Thread):
    hz = 60 # frequence
    prev_ts = 0
    
    # acceleration = 33 # top speed 100, current acceleration is 30 units/sec
    # brake_deceleration = 50
    # reverse_acceleration = 25
    # idle_deceleration = 20
    speed = int(0)  # unsigned, a scalar, 
    MAX_SPEED = 99
    # speed_range = None
    # steeringwheel_angle = 0
    # steeringwheel_angle_range = None

    def __init__(self, vm: VehicleModel) -> None:
        super().__init__()
        self.prev_ts = time.time()
        self.t_span = 1 / self.hz
        self.vm = vm
        # self.speed_range = vm.speed_range
        # self.steeringwheel_angle_range = vm.steeringwheel_angle_range

    def run(self) -> None:
        accu_ts = 0.

        while loop_flag:
            curr_ts = time.time()
            span_ts = curr_ts - self.prev_ts
            accu_ts += span_ts
            self.prev_ts = curr_ts

            if accu_ts >= self.t_span:

                """
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
                """

                prev_speed = self.speed
                vehicle_prev_speed = self.vm.speed

                target_speed = self._fn_drag_acceleration_ratio() * accu_ts + prev_speed
                # we can move forward, speed range is [0, self.MAX_SPEED]
                # target_speed = clamp()
                target_speed = max(0, min(self.MAX_SPEED, target_speed))
                target_speed = round(target_speed)
                self.speed = target_speed


                if ClutchStatus.F1 == self._get_clutch_status() and vehicle_prev_speed >= 0:
                    # transfer the new speed ...
                    # print(f'current: {time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())} ==> {self.speed} {self.vm.speed_range} ==> {self.vm.engine_status}')
                    self.vm.set_speed(target_speed)
                elif ClutchStatus.R == self._get_clutch_status() and vehicle_prev_speed <= 0:
                    # transfer the new speed ...
                    # print(f'current: {time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())} ==> {self.speed} {self.vm.speed_range} ==> {self.vm.engine_status}')
                    self.vm.set_speed(-target_speed)
                else:  # 包括了N档位和方向相反档位
                    _direction = 1 if vehicle_prev_speed >=0 else -1
                    self.vm.set_speed(target_speed * _direction)

                # print(f'current timestamp: {curr_ts} ==> {accu_ts} >= {self.t_span}')
                accu_ts = 0.

    def _get_clutch_status(self) -> ClutchStatus:
        return self.vm.clutch_status

    def _get_throttle_value(self) -> float:
        return self.vm.throttle_value

    def _get_brake_value(self) -> float:
        return self.vm.brake_value

    def _fn_throttle_force(self) -> float:
        k_t = 100
        f_t = k_t * self._get_throttle_value()
        return f_t  # [0. => k_t]

    def _fn_brake_force(self) -> float:
        k_b = 300
        f_b = k_b * self._get_brake_value()
        return f_b  # [0. => k_t]

    def _fn_air_friction_force(self) -> float:
        prev_speed_uniform = abs(self.speed / self.MAX_SPEED)  # [0. => 1l]
        k_f = 100 
        # b_f = 20
        f_f = k_f * prev_speed_uniform #+ b_f
        return f_f  # [0 => k_f]

    def _fn_drag_acceleration_ratio(self) -> float:
        """
        return an acceleration_ratio which is related to the span of time needed to reach the top speed.
        right now, the maximum acc_ratio is 50, meaning that we may take approximate 2 seconds to reach top speed.
        """
        propel_force = self._fn_throttle_force() - self._fn_air_friction_force() - self._fn_brake_force()
        propel_force = round(propel_force)  # [-(100fric + 300brak), 100]  :  [-400, 100]
        m = 1 * ( 100 * 2 ) / self.MAX_SPEED  # 达到顶速需要2s多一些时间，第二个100是顶速100，也可以是99，第一个100是propelforce的加速时顶力
        # F=ma

        # a = F/m
        acceleration_ratio = propel_force / m

        return acceleration_ratio


import cv2
loop_videocapture_flag = True
class WorkerVideoCapture(threading.Thread):
    def __init__(self, vm: VehicleModel) -> None:
        super().__init__()
        self.vm = vm
        self.vc = cv2.VideoCapture(0)

    
    def run(self) -> None:
        self.vm.reset_camera_servo()
        while loop_videocapture_flag:
            # 读取一帧，read()方法是其他两个类方法的结合，具体文档
            # ret为bool类型，指示是否成功读取这一帧
            ret, frame = self.vc.read()
            # 就是个处理一帧的例子，这里转为灰度图
            # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # 不断显示一帧，就成视频了
            # 这里没有提前创建窗口，所以默认创建的窗口不可调整大小
            # 可提前使用cv.WINDOW_NORMAL标签创建个窗口
            # cv2.imshow('frame',gray)
            cv2.imshow('frame',frame)
            # 若没有按下q键，则每1毫秒显示一帧
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        # 所有操作结束后不要忘记释放
        self.vc.release()
        cv2.destroyAllWindows()

        self.vm.rotate_camera_lower_servo(30)
        self.vm.rotate_camera_upper_servo(45)
