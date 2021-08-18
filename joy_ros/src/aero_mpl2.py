#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
from matplotlib import gridspec
import matplotlib
import numpy as np

imu_x,imu_y,imu_z=[],[],[]
motor1,motor2,motor3,motor4=[],[],[],[]
ref1,ref2,ref3=[],[],[]

plt.ion()
fig = plt.figure()
gs = gridspec.GridSpec(12, 2)
ax11 = fig.add_subplot(gs[0:4,0])
plt.setp(ax11.get_xticklabels(), visible=False)
ax12 = fig.add_subplot(gs[4:8,0])
plt.setp(ax12.get_xticklabels(), visible=False)
ax13 = fig.add_subplot(gs[8:12,0])

ax2 = fig.add_subplot(gs[0:3,1])
plt.setp(ax2.get_xticklabels(), visible=False)
ax3 = fig.add_subplot(gs[3:6,1])
plt.setp(ax3.get_xticklabels(), visible=False)
ax4 = fig.add_subplot(gs[6:9,1])
plt.setp(ax4.get_xticklabels(), visible=False)
ax5 = fig.add_subplot(gs[9:12,1])

matplotlib.rcParams.update({'font.size': 13})

line1, = ax11.plot([], [],c='m',linewidth=2, label='imu_roll')
line12, = ax11.plot([], [],c='k',linewidth=2, label='ref_roll')
line2, = ax12.plot([], [],c='b',linewidth=2, label='imu_pitch')
line22, = ax12.plot([], [],c='k',linewidth=2, label='ref_pitch')
line3, = ax13.plot([], [], c='g',linewidth=2, label='imu_yaw')
line32, = ax13.plot([], [],c='k',linewidth=2, label='ref_yaw')

ax11.legend(loc='upper left')
ax12.legend(loc='upper left')
ax13.legend(loc='upper left')

ax13.set_xlabel('Time step')

ax11.set_ylim(-50,50)
ax11.set_xlim(-201,0)
ax12.set_ylim(-50,50)
ax12.set_xlim(-201,0)
ax13.set_ylim(-180,180)
ax13.set_xlim(-201,0)


ax2.set_ylabel('Motor1 PWM')
ax2.set_xlim(-201,0)
ax2.set_ylim(0,1000)
line4, = ax2.plot([], [],c='b',linewidth=2)
ax3.set_ylabel('Motor2 PWM')
ax3.set_xlim(-201,0)
ax3.set_ylim(0,1000)
line5, = ax3.plot([], [],c='g',linewidth=2)
ax4.set_ylabel('Motor3 PWM')
ax4.set_xlim(-201,0)
ax4.set_ylim(0,1000)
line6, = ax4.plot([], [],c='r',linewidth=2)
ax5.set_ylabel('Motor4 PWM')
ax5.set_xlim(-201,0)
ax5.set_ylim(0,1000)
line7, = ax5.plot([], [],c='k',linewidth=2)
ax5.set_xlabel('Time Step')



def update_plot(ix,iy,iz,mot1,mot2,mot3,mot4,r1,r2,r3):
    xr = np.arange(-200., 0, 1.0)

    line1.set_ydata(ix)
    line12.set_ydata(r1)
    line2.set_ydata(iy)
    line22.set_ydata(r2)
    line3.set_ydata(iz)
    line32.set_ydata(r3)

    line4.set_ydata(mot1)
    line5.set_ydata(mot2)
    line6.set_ydata(mot3)
    line7.set_ydata(mot4)

    line1.set_xdata(xr)
    line12.set_xdata(xr)
    line2.set_xdata(xr)
    line22.set_xdata(xr)
    line3.set_xdata(xr)
    line32.set_xdata(xr)
    line4.set_xdata(xr)
    line5.set_xdata(xr)
    line6.set_xdata(xr)
    line7.set_xdata(xr)
    ax11.autoscale_view(False, True, False)
    ax12.autoscale_view(False, True, False)
    ax13.autoscale_view(False, True, False)
    plt.draw()
    plt.pause(0.01)



def imu_msg(msg1):
    global imu_x,imu_y,imu_z
    if len(imu_x)==200:
        imu_x.pop(0)
        imu_y.pop(0)
        imu_z.pop(0)
        imu_x.append(msg1.data[0])
        imu_y.append(msg1.data[1])
        if msg1.data[2]>180:
            imu_z.append(msg1.data[2]-360.)
        else:
            imu_z.append(msg1.data[2])
    else:
        imu_x.append(msg1.data[0])
        imu_y.append(msg1.data[1])
        if msg1.data[2]>180:
            imu_z.append(msg1.data[2]-360.)
        else:
            imu_z.append(msg1.data[2])


def motor_msg(msg2):
    global motor1, motor2, motor3, motor4
    if len(motor1) == 200:
        motor1.pop(0)
        motor2.pop(0)
        motor3.pop(0)
        motor4.pop(0)

        motor1.append(msg2.data[0])
        motor2.append(msg2.data[1])
        motor3.append(msg2.data[2])
        motor4.append(msg2.data[3])
    else:
        motor1.append(msg2.data[0])
        motor2.append(msg2.data[1])
        motor3.append(msg2.data[2])
        motor4.append(msg2.data[3])

def ref_msg(msg3):
    global ref1,ref2,ref3
    if len(ref1)== 200:
        ref1.pop(0)
        ref2.pop(0)
        ref3.pop(0)
        ref1.append(msg3.data[0])
        ref2.append(msg3.data[1])
        ref3.append(msg3.data[2])
    else:
        ref1.append(msg3.data[0])
        ref2.append(msg3.data[1])
        ref3.append(msg3.data[2])


if __name__ == '__main__':
    rospy.init_node('gui_aerobeam', anonymous=True)
    rate = rospy.Rate(100)
    rospy.Subscriber("/imu", Float32MultiArray, imu_msg)
    rospy.Subscriber("/motors", Float32MultiArray, motor_msg)
    rospy.Subscriber("/refs", Float32MultiArray, ref_msg)
    while not rospy.is_shutdown():
        if len(imu_x)==200:
            update_plot(imu_x,imu_y,imu_z, motor1,motor2,motor3,motor4,ref1,ref2,ref3)
        rate.sleep()
