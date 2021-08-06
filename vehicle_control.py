import math
import glob
import serial
import struct
import time
import sys

import rospy
from autoware_msgs.msg import NDTStat
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32

interfaces = {}
target_velocity = 0.0
current_velocity = 0.0
steering_angle = 0.0
should_reverse = False
current_throttle = 0.0

def identify_serial(dev_name, baud_rate=9600):
    device_id_map = {
        'S': 'steering',
        'T': 'throttle',
        'B': 'brakes',
    }

    print("Trying device %s" % dev_name)
    ser = serial.Serial(dev_name, baudrate=baud_rate, timeout=0.5)
    for i in range(5):
        ser.write(struct.pack('>B', 0xFF))
        dev_id = ser.read(1)
        if dev_id:
            break
        else:
            time.sleep(0.5)
    if dev_id in device_id_map.keys():
        dev_type = device_id_map[dev_id]
        print("Device %s identified as %s" % (dev_name, dev_type))
        interfaces[dev_type] = ser
    else:
        print("Device %s failed to identify (%s)" % (dev_name, dev_id))
        ser.close()

def get_twist(msg):
    global steering_angle, should_reverse
    steering_angle = -msg.twist.angular.z * 180.0 / math.pi
    try:
        steering_angle_sign = steering_angle / abs(steering_angle)
    except ZeroDivisionError:
        steering_angle_sign = 1.0
    steering_angle = steering_angle_sign * ((steering_angle / 10.0) ** 2) * 10.0
    steering_angle = steering_angle + 90.0
    if steering_angle > 135.0:
        steering_angle = 135.0
    elif steering_angle < 45.0:
        steering_angle = 45.0
    should_reverse = msg.twist.linear.x < 0

def get_target_velocity(msg):
    global target_velocity
    target_velocity = msg.data

def get_current_velocity(msg):
    global current_velocity
    current_velocity = msg.velocity

def update_throttle():
    global current_throttle

    vel_delta = target_velocity - current_velocity
    #should_brake = vel_delta < 0

    #Throttle change should be proportional to diff between target and current velocity
    if vel_delta >= 0:
        change_speed = 2.0
    else:
        change_speed = 10.0
    throttle_change = vel_delta * change_speed / freq
    current_throttle += throttle_change
    if current_throttle > 50.0:
        current_throttle = 50.0
    elif current_throttle < 0.0:
        current_throttle = 0.0

def update_vehicle_state():
    #Steering
    print("Steering angle: %s - reverse: %s" % (steering_angle, should_reverse))
    interfaces['steering'].write(struct.pack('>B', steering_angle))

    #Throttle
    print("Velocity target: %s - current: %s - throttle: %s" % (target_velocity, current_velocity, current_throttle))
    interfaces['throttle'].write(struct.pack('>B', current_throttle))

    #Brakes
    brakes_val = 0
    print("Brakes: %s" % brakes_val)
    #interfaces['brakes'].write(struct.pack('>B', brakes_val))

    print("----------------------------")

all_devices = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
print("Serial devices available: %s" % ' '.join(all_devices))
for dev_name in all_devices:
    try:
        identify_serial(dev_name)
    except serial.SerialException as e:
        print("Error opening device %s: %s" % (dev_name, e))
needed_interfaces = ['throttle', 'steering']#, 'brakes']
exit = False
for interface in needed_interfaces:
    if not interface in interfaces.keys():
        print("Interface %s not found, exiting" % interface)
        exit = True
if exit:
    sys.exit(-1)
print("Starting")

rospy.init_node('vehicle_control')
rospy.Subscriber('twist_raw', TwistStamped, get_twist)
rospy.Subscriber('target_velocity', Float32, get_target_velocity)
rospy.Subscriber('ndt_stat', NDTStat, get_current_velocity)

freq = 10
r = rospy.Rate(freq)


while not rospy.is_shutdown():
    update_throttle()
    update_vehicle_state()
    try:
        r.sleep()
    except rospy.ROSTimeMovedBackwardsException:
        pass
