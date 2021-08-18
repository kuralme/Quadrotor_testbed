#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
import struct
import serial
from ctypes import *

port_name = rospy.get_param("/joy_py/port")
baud = rospy.get_param("/joy_py/baud")

ser=serial.Serial(port_name, baud, timeout=1)
pub1 = rospy.Publisher('/imu', Float32MultiArray, queue_size=10)
pub2 = rospy.Publisher('/motors', Float32MultiArray, queue_size=10)
pub3 = rospy.Publisher('/refs', Float32MultiArray, queue_size=10)

throttle_ref=1400.0
yaw_ref=1500.0
flag1=True
flag2=True
flag3=True
flag4=True
state=1100.0
timer=0.
kats=0.1
warn_count = 0
data_buf=[1100.,1500.,1500.,1400.,1500,0.1]
ref=[0,0,0]

def convert(s):
    i = int(s, 16)                   # convert from hex to a Python int
    cp = pointer(c_int(i))           # make this into a c integer
    fp = cast(cp, POINTER(c_float))  # cast the int pointer to a float pointer
    return fp.contents.value

def joy_call(msg):
    global throttle_ref,yaw_ref,flag1,flag2,flag3,flag4,state,kats,data_buf
    start=msg.buttons[7]
    emergency=msg.buttons[1]
    if start==1.0:
        state=1500.0
    if emergency==1.0:
        state=1000.0

    roll=1500-msg.axes[4]*500.0
    pitch=1500-msg.axes[3]*500.0

    if (msg.axes[7]==1.0 or msg.axes[7]==-1.0) and flag1:
        throttle_ref+=50*msg.axes[7]
        if min(throttle_ref,1700)==1700:
            throttle_ref=1700.0
        elif max(throttle_ref,1200)==1200:
            throttle_ref=1200.0
        flag1=False
    elif msg.axes[7]==0.0:
        flag1=True

    throttle=throttle_ref+msg.axes[1]*500.0
    if max(throttle,1000)==1000:
        throttle=1000.0
    elif min(throttle,2000)==2000:
        throttle=2000.0

    if (msg.axes[6]==1.0 or msg.axes[6]==-1.0) and flag2:
        yaw_ref-=50*msg.axes[6]
        if min(yaw_ref,1700)==1700:
            yaw_ref=1700
        elif max(yaw_ref,1300)==1300:
            yaw_ref=1300
        flag2=False
    elif msg.axes[6]==0.0:
        flag2=True

    yaw=yaw_ref-msg.axes[0]*500.0
    if max(yaw,1000)==1000:
        yaw=1000.0
    elif min(yaw,2000)==2000:
        yaw=2000.0
    
    if msg.buttons[5]==1.0 and flag3:
	kats+=0.05
	flag3=False
	if min(kats,20.)==20.:
            kats=20.
    elif msg.buttons[5]==0.0:
	flag3=True
	
    if msg.buttons[4]==1.0 and flag4:
	kats-=0.05
	flag4=False
	if max(kats,0.)==0.:
            kats=0.
    elif msg.buttons[4]==0.0:
	flag4=True
    data_buf=[state,roll,pitch,throttle,yaw,kats]



def send_commands(data):
    print data
    messages = [struct.pack('<f', data[i]).encode('hex') for i in range(len(data))]
    val = int('00', 16)
    splitted_data = list(map(''.join, zip(*[iter("".join(messages))] * 2)))
    new_txt = ""
    for i in splitted_data:
        val = val ^ int(i, 16)
        new_txt += str(chr(int(i, 16)))
    new_txt += str(chr(val))
    new_txt = str(chr(int('A5', 16))) + str(chr(len(splitted_data))) + \
              new_txt + str(chr(int('5A', 16)))+str(chr(int('0A', 16)))
    ser.write(new_txt)


def serial_callback():
    global warn_count
    incoming_data=ser.readline()
    if len(incoming_data)>3:
        a=[ord(i) for i in incoming_data[:-1]]
        values=[format(i,'02X') for i in a][2:a[1]+2]
        val_data=int('00',16)
        for dt in values:
            val_data^=int(dt,16)
        if val_data==a[-2] and a[0]==0xa5 and a[-1]==0x5a:

            angle_data=Float32MultiArray()
            angle_data.data=[convert(''.join(reversed(values[0:4]))),
                                 convert(''.join(reversed(values[4:8]))),
                                 convert(''.join(reversed(values[8:12])))]
	  

            motor_data=Float32MultiArray()
            motor_data.data=[convert(''.join(reversed(values[12:16]))),
                                 convert(''.join(reversed(values[16:20]))),
                                 convert(''.join(reversed(values[20:24]))),
                         convert(''.join(reversed(values[24:28])))]
	    
	    ref_data=Float32MultiArray()
	    ref_data.data=[(data_buf[1]-1500.)/20.,(1500.-data_buf[2])/20.,(data_buf[4]-1500)/20.]
            pub1.publish(angle_data)
            pub2.publish(motor_data)
	    pub3.publish(ref_data)
           
        else:
            warn_count=warn_count+1
            print("Serial port did not understand the message "+str(warn_count)+ " times")

    else:
        warn_count = warn_count + 1
        print("Serial port did not understand the message " + str(warn_count) + " times")



if __name__ == '__main__':
    rospy.init_node('joy_aerobeam', anonymous=True)
    rate = rospy.Rate(20)
    rospy.Subscriber("/joy",Joy,joy_call)
    while not rospy.is_shutdown():
        serial_callback()
        timer+=1.
	if len(data_buf)==6:    
	   data_buf.insert(0,timer)
	   send_commands(data_buf)
        rate.sleep()
