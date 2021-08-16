#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
* @par Copyright (C): 2010-2020, hunan CLB Tech
* @file      Basic_movement
* @version    V2.1
* @details
* @par History

@author: ViolinSolo
"""
# from LOBOROBOT import LOBOROBOT  # 载入机器人库
import RPi.GPIO as GPIO
import os
# import sys
import traceback
import threading
import queue
from enum import Enum

exitFlag = False
queueLock = threading.Lock()

@DeprecationWarning
def pre_process_param(speed, timespan):
    if speed is None:
        speed = 0
    if timespan is None:
        timespan = 0
    
    speed = 100 if speed > 100 else speed
    speed = 0 if speed < 0 else speed

    timespan = 0 if timespan < 0 else timespan

    return speed, timespan

class OrderType(Enum):
    FORWARD=1,
    BACKWARD=2,
    LEFTWARD=3,
    RIGHTWARD=4,
    STOP=5,
    TERMINATE=6,

class Order():
    def __init__(self, otype: OrderType, timespan: float, speed: int):
        self.otype = otype
        self.tspan = timespan
        self.speed = speed

def print_status(current_task: Order, remain_tasks: list):
    return
    os.system('clear')
    if current_task is None:
        print('='*30)
        print(f'= All task done.')
        print('='*30)
        return
    
    print('='*30)
    print(f'= Current Task: [{current_task.otype}] => time: {current_task.tspan}s | speed: {current_task.speed}')
    print('='*30)
    for i, t in enumerate(remain_tasks, 0):
        print(f'= Remain Task [{i}]: [{t.otype}] => time: {t.tspan}s | speed: {t.speed}')
        if i == len(remain_tasks)-1:
            print('='*30)
    
class Worker(threading.Thread):
    def __init__(self, q):
        threading.Thread.__init__(self)
        self.q = q
    
    def run(self):
        print('Worker Thread Open.')
        while not exitFlag:
            queueLock.acquire()
            if not self.q.empty():
                task = self.q.get()

                # fetch and process task 
                print_status(task, self.q)
                self.process_task(task)
                queueLock.release()
            else:
                print_status(None, self.q)
                queueLock.release()

        print('Worker Thread Exit.')

    def process_task(self, task: Order):
        otype = task.otype
        tspan = task.tspan
        speed = task.speed

        if otype == OrderType.FORWARD:
            try:
                # speed, timespan = pre_process_param(speed, timespan)
                clbrobot.t_up(speed, tspan) # 机器人前进
                clbrobot.t_stop(0) # 机器人停止
                return 'Forward Success'
            except:
                clbrobot.t_stop(0) # 机器人停止
                # GPIO.cleanup()
                return 'Stop Error:\n'+traceback.format_exc()
        elif otype == OrderType.BACKWARD:
            try:
                # speed, timespan = pre_process_param(speed, timespan)
                clbrobot.t_down(speed, tspan) # 机器人后退
                clbrobot.t_stop(0) # 机器人停止
                return 'Backward Success'
            except:
                clbrobot.t_stop(0) # 机器人停止
                #GPIO.cleanup()
                return 'Stop Error:\n'+traceback.format_exc()
        elif otype == OrderType.LEFTWARD:
            try:
                # speed, timespan = pre_process_param(speed, timespan)
                clbrobot.turnLeft(speed, tspan) # 机器人左转
                clbrobot.t_stop(0) # 机器人停止
                return 'Leftward Success'
            except:
                clbrobot.t_stop(0) # 机器人停止
                GPIO.cleanup()
                return 'Stop Error:\n'+traceback.format_exc()
        elif otype == OrderType.RIGHTWARD:
            try:
                # speed, timespan = pre_process_param(speed, timespan)
                clbrobot.turnRight(speed, tspan) # 机器人右移
                clbrobot.t_stop(0) # 机器人停止
                return 'Rightward Success'
            except:
                clbrobot.t_stop(0) # 机器人停止
                #GPIO.cleanup()
                return 'Stop Error:\n'+traceback.format_exc()
        elif otype == OrderType.STOP:
            try:
                clbrobot.t_stop(0) # 机器人停止
                #GPIO.cleanup()
                return 'Stop Success'
            except:
                clbrobot.t_stop(0) # 机器人停止
                #GPIO.cleanup()
                return 'Stop Error:\n'+traceback.format_exc()
        elif otype == OrderType.TERMINATE:
            try:
                clbrobot.t_stop(0) # 机器人停止
                GPIO.cleanup()
                return 'Terminate Success'
            except:
                clbrobot.t_stop(0) # 机器人停止
                GPIO.cleanup()
                return 'Terminate Error:\n'+traceback.format_exc()




if __name__ == '__main__':
    
    from flask import Flask
    from flask import request

    app = Flask(__name__)

    from VehicleControllerV3 import VehicleController, VehicleModel, ClutchStatus
    from VehicleStatus import VehicleStatus

    vc = VehicleController()
    vm = VehicleModel(vc)
    vs = VehicleStatus(vm)
    vs.moniter()

    taskQueue = queue.Queue(20)
    workerThread = Worker(taskQueue)
    workerThread.start()

    @app.route("/")
    def hello_world():
        return "<p>Hello, World!</p>"

    @app.route('/forward/<int:speed>/<float:timespan>')
    def forward(speed: int, timespan: float):
        try:
            speed, timespan = pre_process_param(speed, timespan)
            # clbrobot.t_up(speed, timespan) # 机器人前进
            taskQueue.put(Order(OrderType.FORWARD, timespan, speed))
            return 'Forward Success'
        except:
            clbrobot.t_stop(0) # 机器人停止
            # GPIO.cleanup()
            return 'Stop Error:\n'+traceback.format_exc()

    @app.route('/backward/<int:speed>/<float:timespan>')
    def backward(speed: int, timespan: float):
        try:
            speed, timespan = pre_process_param(speed, timespan)
            # clbrobot.t_down(speed, timespan) # 机器人后退
            taskQueue.put(Order(OrderType.BACKWARD, timespan, speed))
            return 'Backward Success'
        except:
            clbrobot.t_stop(0) # 机器人停止
            #GPIO.cleanup()
            return 'Stop Error:\n'+traceback.format_exc()

    @app.route('/leftward/<int:speed>/<float:timespan>')
    def leftward(speed: int, timespan: float):
        try:
            speed, timespan = pre_process_param(speed, timespan)
            # clbrobot.turnLeft(speed, timespan) # 机器人左转
            taskQueue.put(Order(OrderType.LEFTWARD, timespan, speed))
            return 'Leftward Success'
        except:
            clbrobot.t_stop(0) # 机器人停止
            GPIO.cleanup()
            return 'Stop Error:\n'+traceback.format_exc()

    @app.route('/rightward/<int:speed>/<float:timespan>')
    def rightward(speed: int, timespan: float):
        try:
            speed, timespan = pre_process_param(speed, timespan)
            # clbrobot.moveRight(speed, timespan) # 机器人右移
            taskQueue.put(Order(OrderType.RIGHTWARD, timespan, speed))
            return 'Rightward Success'
        except:
            clbrobot.t_stop(0) # 机器人停止
            #GPIO.cleanup()
            return 'Stop Error:\n'+traceback.format_exc()

    @app.route('/control/<int:spd0>/<int:spd1>/<int:spd2>/<int:spd3>')
    def native_control(spd0, spd1, spd2, spd3):
        clbrobot.MotorRun(0, 'forward' if spd0 > 0 else 'backward', abs(spd0)) # front left
        clbrobot.MotorRun(1, 'forward' if spd1 > 0 else 'backward', abs(spd1)) # front right
        clbrobot.MotorRun(2, 'forward' if spd2 > 0 else 'backward', abs(spd2)) # back left
        clbrobot.MotorRun(3, 'forward' if spd3 > 0 else 'backward', abs(spd3)) # back right
        return 'success'

    @app.route('/stop')
    def stop():
        try:
            # clbrobot.t_stop(0) # 机器人停止
            #GPIO.cleanup()
            taskQueue.put(Order(OrderType.STOP, 0, 0))
            return 'Stop Success'
        except:
            clbrobot.t_stop(0) # 机器人停止
            #GPIO.cleanup()
            return 'Stop Error:\n'+traceback.format_exc()

    @app.route('/terminate')
    def terminate():
        try:
            # clbrobot.t_stop(0) # 机器人停止
            #GPIO.cleanup()
            taskQueue.put(Order(OrderType.TERMINATE, 0, 0))
            return 'Terminate Success'
        except:
            clbrobot.t_stop(0) # 机器人停止
            #GPIO.cleanup()
            return 'Stop Error:\n'+traceback.format_exc()

    @app.route('/zz')
    def zz():
        exitFlag = True

    @app.route('/aa')
    def aa():
        exitFlag = False
        newWorkerThread = Worker(taskQueue)
        newWorkerThread.start()

    @app.route('/vm/acc/<int:max_speed>')
    def vm_acc(max_speed):
        max_speed = max_speed if max_speed < 99 else 99
        vm.accelerate(max_speed)
        return '/vm/acc'

    @app.route('/vm/dec')
    def vm_dec():
        vm.decelerate()
        return '/vm/dec'

    @app.route('/vm/rev/<int:max_speed>')
    def vm_rev(max_speed):
        max_speed = max_speed if max_speed < 99 else 99
        vm.reverse(-max_speed)
        return '/vm/rev'

    @app.route('/vm/idl')
    def vm_idl():
        vm.idle()
        return '/vm/idl'

    # /vm/sw?angle=-30
    @app.route('/vm/sw')
    def vm_angle():
        angle = request.args.get('angle', type=int)
        # vm.steeringwheel_angle = angle
        vm.set_sw_angle(angle)
        return f'current angle : {angle}°'

    # /vm/camera?angle=-30&servo=upper
    @app.route('/vm/camera')
    def vm_camera_servo_angle():
        angle = request.args.get('angle', type=int)
        servo = request.args.get('servo', type=str)
        if servo == 'upper':
            vm.rotate_camera_upper_servo(angle)
        elif servo == 'lower':
            vm.rotate_camera_lower_servo(angle)
        else:
            print("-"*32)
            print(f"[ERROR] servo:<{servo}> | angle:<{angle}>")
            print("-"*32)
            
        return f'change to: servo:{servo} | angle:{angle}'

    
    # =====================
    # control with things.
    # =====================
    @app.route('/lg/brake')
    def vm_set_brake_value():
        brake_value = request.args.get('value', type=float)
        brake_value = brake_value if brake_value else 0.
        
        vm.set_brake_value(brake_value)
        return f'change brake_value to: {brake_value}'

    @app.route('/lg/throttle')
    def vm_set_throttle_value():
        throttle_value = request.args.get('value', type=float)
        throttle_value = throttle_value if throttle_value else 0.
        
        vm.set_throttle_value(throttle_value)
        return f'change throttle_value to: {throttle_value}'

    @app.route('/lg/clutch')
    def vm_set_clutch_status():
        clutch_status_val = request.args.get('value', type=str)
        
        clutch_status = ClutchStatus.N
        if clutch_status_val == 'F1':
            clutch_status = ClutchStatus.F1
        elif clutch_status_val == 'R':
            clutch_status = ClutchStatus.R
        
        vm.set_clutch_status(clutch_status)
        return f'change clutch_status to: {clutch_status}'

    @app.route('/lg/sw')
    def vm_set_steeringwheel_angle():
        steeringwheel_angle = request.args.get('value', type=int)
        steeringwheel_angle = min(vm.steeringwheel_angle_range[1], 
                                max(vm.steeringwheel_angle_range[0], 
                                steeringwheel_angle))
        
        vm.set_sw_angle(steeringwheel_angle)
        return f'change sw_angle to: {steeringwheel_angle}°'

    app.run(host='0.0.0.0', port=5000)