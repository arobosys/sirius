#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from malish.msg import NewTwist, Diode, Obstacle, Lift, JoyCMD
from sensor_msgs.msg import Joy
from math import atan2,pi,sqrt
from geometry_msgs.msg import Twist

vel_msg = Twist()
vel_msg.linear.x = 0.0
vel_msg.linear.y = 0.0
vel_msg.linear.z = 0.0
vel_msg.angular.x = 0.0
vel_msg.angular.y = 0.0
vel_msg.angular.z = 0.0

vel_joy =  vel_msg
t1 = NewTwist()
t1.linear_vel = 0;
t1.orient = 0.0;
t1.angle_vel = 0.0;

t2 = NewTwist()
t2.linear_vel = 0;
t2.orient = 0.0;
t2.angle_vel = 0.0;

t3 = NewTwist()
t3.linear_vel = 0;
t3.orient = 0.0;
t3.angle_vel = 0.0;

flag_no_cmd = False;

lift = False;
alert = False;
gogogo_prev = False;
restart_prev = False;

class Smoother(object):
    """docstring"""
    
    def __init__(self, data, time):
        """Constructor"""
        self.data = data
        self.time = time

    def smooth(self, data, time, tau):
        dt = time - self.time
        delta = data - self.data

        if abs(delta) > tau * dt:
            self.data = self.data + tau * dt * delta/abs(delta)
        else:
            self.data = data

        self.time = time

        return self.data

def callback(data):
    global pub, t1, flag_no_cmd, pub_lift, gogogo_prev, restart_prev

    #Dead zone constant
    thresh = 0.15

    #Constants
    norm_lin_vel = 1000
    norm_orient = pi/2.0
    norm_angle_vel = pi/8.0

    #Joystick Buttons
    lr = data.axes[0]
    ud = data.axes[1]
    throttle = data.buttons[0]
    lifter = data.buttons[4]
    unlifter = data.buttons[5]
    gogogo = data.buttons[1]
    restart = data.buttons[2]
    joy_cmd = JoyCMD()

    if (gogogo==1):
        joy_cmd.gogogo = True
    else:
        joy_cmd.gogogo = False

    if (restart==1):
        joy_cmd.restart = True
    else:
        joy_cmd.restart = False
    
    #pub.publish(joy cmd)
    if (gogogo!=gogogo_prev) or(restart!=restart_prev):
        pub_command.publish(joy_cmd)

    gogogo_prev = bool(gogogo)
    restart_prev = bool(restart) 
    if (data.buttons[3]==1):
        flag_no_cmd = True
    if (throttle==1):
        flag_no_cmd = False
    
  
    left_turn = data.axes[4]#2]
    right_turn = data.axes[5]#5]

    lift_msg = Lift()
    if (lifter==1) and (unlifter!=1):
        lift_msg.dio1 = True
        lift_msg.dio2 = False
        lift_msg.dio3 = False

    if (lifter!=1) and (unlifter==1):
        lift_msg.dio1 = False
        lift_msg.dio2 = True
        lift_msg.dio3 = True
    
    #Dead zone logick
    if lr < thresh and lr > -thresh:
        lr = 0
    if ud < thresh and ud > -thresh:
        ud = 0
    

    #Dead zone turn
    turn = left_turn - right_turn
    if turn < thresh*2 and turn > -thresh*2:
        turn = 0.0

    vel_joy.linear.x =  norm_lin_vel * ud * throttle
    vel_joy.linear.y =  norm_lin_vel * lr * throttle
    vel_joy.angular.z =  turn * norm_angle_vel * throttle
    
    t1.linear_vel = int(throttle * norm_lin_vel * sqrt(lr**2+ud**2))
    t1.orient = norm_orient + atan2(lr, -ud)  
    t1.angle_vel =  turn * norm_angle_vel * throttle
	
    #pub.publish(t1)
    pub_lift.publish(lift_msg)

def vel_callback(msg):
    global vel_msg, t2
    norm_orient = pi/2.0
    vel_msg = msg
    t2.linear_vel = int(1000*(sqrt(vel_msg.linear.x**2 + vel_msg.linear.y**2)))
    t2.orient = norm_orient + atan2(vel_msg.linear.y, -vel_msg.linear.x)
    t2.angle_vel = vel_msg.angular.z

def smooth_vel(smoother_x, smoother_y, time, tau):
    global vel_joy, t1

    vel_x = smoother_x.smooth(vel_joy.linear.x, time, tau)
    vel_y = smoother_y.smooth(vel_joy.linear.y, time, tau)
    linear_vel = int(sqrt(vel_x**2 + vel_y**2))
    orient = pi/2.0 + atan2(vel_y, -vel_x)
    return linear_vel, orient

def safety_callback(msg):
    global alert
    alert = msg.alert

def talker():
    global pub, t1, t2, flag_no_cmd, pub_lift, alert, pub_command 
    rospy.init_node('twist_joy')
    pub = rospy.Publisher('/twist/command', NewTwist, queue_size=1)
    pub_lift = rospy.Publisher('/lift', Lift , queue_size=10)
    pub_command = rospy.Publisher('/joy/command', JoyCMD , queue_size=10)

    rospy.Subscriber("/joy", Joy, callback)
    rospy.Subscriber("/cmd_vel", Twist, vel_callback)
    rospy.Subscriber("/safety", Obstacle, safety_callback)

    rate = rospy.Rate(20) # 100hz

    x_vel_smooth = Smoother(0.0, rospy.get_time())
    y_vel_smooth = Smoother(0.0, rospy.get_time())
    angle_vel_smooth = Smoother(0.0, rospy.get_time())

    while not rospy.is_shutdown():
        if (flag_no_cmd):
            if alert:
                t = t3
            else:
                t = t2
        else:
            t = t1

            t.angle_vel = angle_vel_smooth.smooth(vel_joy.angular.z, rospy.get_time(), pi/2.0)

            t.linear_vel, t.orient =  smooth_vel(x_vel_smooth, y_vel_smooth, rospy.get_time(), 800)

        pub.publish(t)
        rate.sleep()
                                            
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
