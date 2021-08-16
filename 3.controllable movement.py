#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
* @par Copyright (C): 2010-2020, hunan CLB Tech
* @file		 Basic_movement
* @version	  V2.0
* @details
* @par History

@author: zhulin
"""
from LOBOROBOT import LOBOROBOT  # 载入机器人库
import RPi.GPIO as GPIO
import os
import sys

# reload(sys)
# sys.setdefaultencoding('utf8')


def pre_process_param(speed, timespan):
	if speed is None:
		speed = 0
	if timespan is None:
		timespan = 0
	
	speed = 100 if speed > 100 else speed
	speed = 0 if speed < 0 else speed

	timespan = 0 if timespan < 0 else timespan

	return speed, timespan





if __name__ == "__main__":
	'''
	try:
		while True:
			clbrobot.t_up(50,3)	  # 机器人前进
			clbrobot.t_stop(1) # 机器人停止
			clbrobot.t_down(50,3)	# 机器人后退
			clbrobot.t_stop(1) # 机器人停止
			clbrobot.turnLeft(50,3)  # 机器人左转
			clbrobot.t_stop(1) # 机器人停止
			clbrobot.turnRight(50,3) # 机器人右转
			clbrobot.t_stop(1) # 机器人停止
			clbrobot.moveLeft(50,3)  # 机器人左移
			clbrobot.t_stop(1) # 机器人停止
			clbrobot.moveRight(50,3) # 机器人右移
			clbrobot.t_stop(1) # 机器人停止
			clbrobot.forward_Left(50,3) # 机器人前左斜
			clbrobot.t_stop(1) # 机器人停止
			clbrobot.backward_Right(50,3) # 机器人后右斜
			clbrobot.t_stop(1) # 机器人停止
			clbrobot.forward_Right(50,3)  # 机器人前右斜
			clbrobot.t_stop(1) # 机器人停止
			clbrobot.backward_Left(50,3)  # 机器人后左斜
			clbrobot.t_stop(5)	   # 机器人停止			
	except KeyboardInterrupt:
		clbrobot.t_stop(0) # 机器人停止
		GPIO.cleanup()
	'''

	from flask import Flask

	app = Flask(__name__)

	clbrobot = LOBOROBOT() # 实例化机器人对象


	@app.route("/")
	def hello_world():
		return "<p>Hello, World!</p>"

	@app.route('/forward/<int:speed>/<float:timespan>')
	def forward(speed: int, timespan: float):
		try:
			speed, timespan = pre_process_param(speed, timespan)
			clbrobot.t_up(speed, timespan) # 机器人前进
		except:
			clbrobot.t_stop(0) # 机器人停止
			GPIO.cleanup()

	@app.route('/backward/<int:speed>/<float:timespan>')
	def backward(speed: int, timespan: float):
		try:
			speed, timespan = pre_process_param(speed, timespan)
			clbrobot.t_down(speed, timespan) # 机器人后退
		except:
			clbrobot.t_stop(0) # 机器人停止
			GPIO.cleanup()

	@app.route('/leftward/<int:speed>/<float:timespan>')
	def leftward(speed: int, timespan: float):
		try:
			speed, timespan = pre_process_param(speed, timespan)
			clbrobot.turnLeft(speed, timespan) # 机器人左转
		except:
			clbrobot.t_stop(0) # 机器人停止
			GPIO.cleanup()

	@app.route('/rightward/<int:speed>/<float:timespan>')
	def rightward(speed: int, timespan: float):
		try:
			speed, timespan = pre_process_param(speed, timespan)
			clbrobot.moveRight(speed, timespan) # 机器人右移
		except:
			clbrobot.t_stop(0) # 机器人停止
			GPIO.cleanup()

	@app.route('/stop')
	def stop(speed: int, timespan: float):
		try:
			clbrobot.t_stop(0) # 机器人停止
			GPIO.cleanup()
		except:
			clbrobot.t_stop(0) # 机器人停止
			GPIO.cleanup()
