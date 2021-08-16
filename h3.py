from VehicleController import VehicleController, VehicleModel

vc = VehicleController()
vm = VehicleModel(vc)

import matplotlib.pyplot as plt
import time

titl = 'Realtime Speed'
xlab = 'Time'
ylab = 'Speed (units)'
ax = [0]
ay = [0]
max_len = 40
curr_ts = 0

print("init plt")
plt.ion() # 开启一个画图的窗口 #开启interactive mode 成功的关键函数
plt.title(titl, fontsize='large',fontweight='bold') #设置字体大小与格式
plt.xlabel(xlab)
plt.ylabel(ylab)

print("begin plt")
while True:
    _speed = vm.speed
    curr_ts += 0.01

    ay.append(_speed)
    ax.append(curr_ts)

    if len(ax) > max_len:
        ax = ax[-max_len:]
        ay = ay[-max_len:]

    plt.clf()  # 清除之前画的图
    plt.title(titl, fontsize='large', fontweight='bold')  # 设置字体大小与格式
    plt.xlabel(xlab)
    plt.ylabel(ylab)
    plt.grid(linestyle='-.') #显示网格

    plt.plot(ax, ay)  # 画出当前 ax 列表和 ay 列表中的值的图形
    plt.pause(0.01)  # 暂停0.01秒

"""
# k = 0          #x轴初始值
# len = len(data)

for i in data:
    num = umbers = list(map(float, i))#进行元组数据的转化
    num_data = num[0]

    plt.clf()  # 清除之前画的图
    plt.title(titl, fontsize='large', fontweight='bold')  # 设置字体大小与格式
    plt.xlabel(xlab)
    plt.ylabel(ylab)
    plt.grid(linestyle='-.') #显示网格

    ax.append(k)
    ay.append(num_data)
    k = k + 1
    plt.plot(ax,ay)        # 画出当前 ax 列表和 ay 列表中的值的图形
    plt.pause(0.01)         # 暂停0.1秒
"""

