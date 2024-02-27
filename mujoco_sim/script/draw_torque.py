#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import sys

# 初始化数据列表
x_data = []
y_data = []

def callback(data):
    index = int(sys.argv[1]) - 1  # 获取命令行参数并转换为索引
    global x_data, y_data

    # 取前12个数
    real_torque = data.data[index]

    # 添加x轴数据（时间）
    x_data.append(rospy.Time.now().to_sec())

    # 更新y轴数据
    y_data.append(real_torque)

    # 绘制指定索引的折线图
    plt.cla()  # 清除之前的图形
    
    if index >= 0 and index < 12:
        # 获取指定索引的数据
        plt.plot(x_data, y_data)
        plt.xlabel('Time')
        plt.ylabel('Torque')
        plt.title(f'Torque {index+1}')
        plt.draw()
        plt.pause(0.001)

def listener():
    rospy.init_node('torque_listener', anonymous=True)
    rospy.Subscriber('/realTorque', Float32MultiArray, callback)
    plt.ion()  # 打开交互模式
    plt.show()  # 显示图形
    rospy.spin()

if __name__ == '__main__':
    if len(sys.argv) != 2 or not sys.argv[1].isdigit():
        print('请提供一个有效的整数参数作为索引。')
        sys.exit(1)

    listener()