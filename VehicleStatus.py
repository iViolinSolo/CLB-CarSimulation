import threading
import matplotlib.pyplot as plt
from VehicleController import VehicleModel
import time

class Worker(threading.Thread):
    keep_moniter = True

    def __init__(self, vm: VehicleModel) -> None:
        super().__init__()
        self.vm = vm
        
    def run(self) -> None:
        titl = 'Realtime Speed'
        xlab = 'Time'
        ylab = 'Speed (units)'
        ax = [0]
        ay = [0]
        max_len = 20
        curr_ts = 0

        print("init plt")
        plt.ion() # 开启一个画图的窗口 #开启interactive mode 成功的关键函数
        plt.title(f'{titl} [{self.vm.engine_status}]', fontsize='large',fontweight='bold') #设置字体大小与格式
        plt.xlabel(xlab)
        plt.ylabel(ylab)

        print("begin plt")
        while self.keep_moniter:
            _speed = self.vm.speed
            curr_ts += 0.1

            ay.append(_speed)
            ax.append(curr_ts)

            if len(ax) > max_len:
                ax = ax[-max_len:]
                ay = ay[-max_len:]

            plt.clf()  # 清除之前画的图
            plt.title(f'{titl}: {self.vm.speed} | [{self.vm.engine_status}]', fontsize='large', fontweight='bold')  # 设置字体大小与格式
            plt.xlabel(xlab)
            plt.ylabel(ylab)
            plt.grid(linestyle='-.') #显示网格

            plt.plot(ax, ay)  # 画出当前 ax 列表和 ay 列表中的值的图形
            plt.pause(0.01)  # 暂停0.1秒  ### 0.1 == 1s I don't know why this happens.

class VehicleStatus:
    wk = None
    def __init__(self, vm: VehicleModel) -> None:
        self.vm = vm
        self.wk = Worker(self.vm)

    def moniter(self):
        self.wk.start()
 
    def stop_moniter(self):
        self.wk.keep_moniter = False
