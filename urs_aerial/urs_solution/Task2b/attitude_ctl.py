#!/usr/bin/env python

__author__ = 'thaus'
from datetime import datetime
import rospy
from pid import PID
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from mav_msgs.msg import Actuators
from dynamic_reconfigure.server import Server
from urs_solution.msg import PIDController
from urs_solution.cfg import MavAttitudeCtlParamsConfig
import math

class AttitudeControl:
    '''
    Class implements MAV attitude control (roll, pitch, yaw). Two PIDs in cascade are
    used for each degree of freedom.
    Subscribes to:
        odometry           - used to extract attitude and attitude rate of the vehicle
        mot_vel_ref        - used to receive reference motor velocity from the height controller
        euler_ref          - used to set the attitude reference (useful for testing controllers)
    Publishes:
        command/motors     - reference motor velocities sent to each motor controller
        pid_roll           - publishes PID-roll data - reference value, measured value, P, I, D and total component (useful for tuning params)
        pid_roll_rate      - publishes PID-roll_rate data - reference value, measured value, P, I, D and total component (useful for tuning params)
        pid_pitch          - publishes PID-pitch data - reference value, measured value, P, I, D and total component (useful for tuning params)
        pid_pitch_rate     - publishes PID-pitch_rate data - reference value, measured value, P, I, D and total component (useful for tuning params)
        pid_yaw            - publishes PID-yaw data - reference value, measured value, P, I, D and total component (useful for tuning params)
        pid_yaw_rate       - publishes PID-yaw_rate data - reference value, measured value, P, I, D and total component (useful for tuning params)

    Dynamic reconfigure is used to set controllers param online.
    '''

    def __init__(self):
        '''
        Initialization of the class.
        '''

        self.start_flag = False             # flag indicates if the first measurement is received
        self.config_start = False           # flag indicates if the config callback is called for the first time
        self.euler_mv = Vector3()           # measured euler angles
        self.euler_sp = Vector3(0, 0, 0)    # euler angles reference values

        self.w_sp = 0                       # reference value for motor velocity - it should be the output of height controller

        self.euler_rate_mv = Vector3()      # measured angular velocities

        self.pid_roll = PID()                           # roll controller
        self.pid_roll_rate  = PID()                     # roll rate (wx) controller

        self.pid_pitch = PID()                          # pitch controller
        self.pid_pitch_rate = PID()                     # pitch rate (wy) controller

        self.pid_yaw = PID()                            # yaw controller
        self.pid_yaw_rate = PID()                       # yaw rate (wz) controller
        self.t_old = datetime.now()
        ##################################################################
        ##################################################################
        # Add your PID params here

        self.pid_roll_rate.set_kp(80)
        self.pid_roll_rate.set_ki(41)
        self.pid_roll_rate.set_kd(0)

        self.pid_roll.set_kp(5.0)
        self.pid_roll.set_ki(0.0)
        self.pid_roll.set_kd(0.0)

        self.pid_pitch_rate.set_kp(80)
        self.pid_pitch_rate.set_ki(41)
        self.pid_pitch_rate.set_kd(0)

        self.pid_pitch.set_kp(5.0)
        self.pid_pitch.set_ki(0.0)
        self.pid_pitch.set_kd(0.0)

        self.pid_yaw_rate.set_kp(80)
        self.pid_yaw_rate.set_ki(40)
        self.pid_yaw_rate.set_kd(0.0)
        
        self.pid_yaw.set_kp(0.4)
        self.pid_yaw.set_ki(0)
        self.pid_yaw.set_kd(0.0)

        ##################################################################
        ##################################################################

        self.ros_rate = rospy.Rate(100)                 # attitude control at 100 Hz

        rospy.Subscriber('odometry', Odometry, self.odometry_cb)
        rospy.Subscriber('mot_vel_ref', Float32, self.mot_vel_ref_cb)
        rospy.Subscriber('euler_ref', Vector3, self.euler_ref_cb)
        self.pub_mot = rospy.Publisher('/gazebo/command/motor_speed', Actuators, queue_size=1)
        self.pub_pid_roll = rospy.Publisher('pid_roll', PIDController, queue_size=1)
        self.pub_pid_roll_rate = rospy.Publisher('pid_roll_rate', PIDController, queue_size=1)
        self.pub_pid_pitch = rospy.Publisher('pid_pitch', PIDController, queue_size=1)
        self.pub_pid_pitch_rate = rospy.Publisher('pid_pitch_rate', PIDController, queue_size=1)
        self.pub_pid_yaw = rospy.Publisher('pid_yaw', PIDController, queue_size=1)
        self.pub_pid_yaw_rate = rospy.Publisher('pid_yaw_rate', PIDController, queue_size=1)
        self.cfg_server = Server(MavAttitudeCtlParamsConfig, self.cfg_callback)

    def run(self):
        '''
        Runs ROS node - computes PID algorithms for cascade attitude control.
        '''

        while not self.start_flag:
            print "Waiting for the first measurement."
            rospy.sleep(0.5)
        print "Starting attitude control."

        while not rospy.is_shutdown():
            self.ros_rate.sleep()

            ####################################################################
            ####################################################################
            # Add your code for cascade control for roll, pitch, yaw.
            # reference attitude values are stored in self.euler_sp
            # (self.euler_sp.x - roll, self.euler_sp.y - pitch, self.euler_sp.z - yaw)
            # Measured attitude values are stored in self.euler_mv (x,y,z - roll, pitch, yaw)
            # Measured attitude rate values are store in self.euler_rate_mv (self.euler_rate_mv.x, y, z)
            # Your result should be reference velocity value for each motor.
            # Store them in variables mot_sp1, mot_sp2, mot_sp3, mot_sp4
            u=self.pid_roll.compute(self.euler_sp.x,self.euler_mv.x)
            mot_speedr=self.pid_roll_rate.compute(u,self.euler_rate_mv.x)
            mot_sp1_r=-mot_speedr
            mot_sp2_r=mot_speedr
            mot_sp3_r=mot_speedr
            mot_sp4_r=-mot_speedr

            u=self.pid_pitch.compute(self.euler_sp.y,self.euler_mv.y)
            mot_speedp=self.pid_pitch_rate.compute(u,self.euler_rate_mv.y)
            mot_sp1_p=-mot_speedp
            mot_sp2_p=-mot_speedp
            mot_sp3_p=mot_speedp
            mot_sp4_p=mot_speedp

            u=self.pid_yaw.compute(self.euler_sp.z,self.euler_mv.z)
            mot_speedy=self.pid_yaw_rate.compute(u,self.euler_rate_mv.z)
            print("-----------")
            print("YAW >>> " , mot_speedy )
            mot_sp1_y=-mot_speedy
            mot_sp2_y=mot_speedy
            mot_sp3_y=-mot_speedy
            mot_sp4_y=mot_speedy
            


            
            mot_sp1=self.w_sp + mot_sp1_r + mot_sp1_p + mot_sp1_y
            mot_sp2=self.w_sp + mot_sp2_r + mot_sp2_p + mot_sp2_y
            mot_sp3=self.w_sp + mot_sp3_r + mot_sp3_p + mot_sp3_y
            mot_sp4=self.w_sp + mot_sp4_r + mot_sp4_p + mot_sp4_y
            Ts =  datetime.now() - self.t_old
            self.t_old = datetime.now()
            print( Ts.total_seconds())



            ####################################################################
            ####################################################################

            # Publish motor velocities
            mot_speed_msg = Actuators()
            mot_speed_msg.angular_velocities = [mot_sp1,mot_sp2,mot_sp3,mot_sp4]
            self.pub_mot.publish(mot_speed_msg)

            # Publish PID data - could be usefule for tuning
            self.pub_pid_roll.publish(self.pid_roll.create_msg())
            self.pub_pid_roll_rate.publish(self.pid_roll_rate.create_msg())
            self.pub_pid_pitch.publish(self.pid_pitch.create_msg())
            self.pub_pid_pitch_rate.publish(self.pid_pitch_rate.create_msg())
            self.pub_pid_yaw.publish(self.pid_yaw.create_msg())
            self.pub_pid_yaw_rate.publish(self.pid_yaw_rate.create_msg())

    def mot_vel_ref_cb(self, msg):
        '''
        reference motor velocity callback. (This should be published by height controller).
        :param msg: Type Float32
        '''
        self.w_sp = msg.data

    def odometry_cb(self, msg):
        '''
        Odometry callback. Used to extract roll, pitch, yaw and their rates.
        We used the following order of rotation - 1)yaw, 2) pitch, 3) roll
        :param msg: Type nav_msgs/Odometry
        '''

        if not self.start_flag:
            self.start_flag = True

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        # conversion quaternion to euler (yaw - pitch - roll)
        self.euler_mv.x = math.atan2(2 * (qw * qx + qy * qz), qw * qw - qx * qx - qy * qy + qz * qz)
        self.euler_mv.y = math.asin(2 * (qw * qy - qx * qz))
        self.euler_mv.z = math.atan2(2 * (qw * qz + qx * qy), qw * qw + qx * qx - qy * qy - qz * qz)

         # gyro measurements (p,q,r)
        p = msg.twist.twist.angular.x
        q = msg.twist.twist.angular.y
        r = msg.twist.twist.angular.z

        sx = math.sin(self.euler_mv.x)   # sin(roll)
        cx = math.cos(self.euler_mv.x)   # cos(roll)
        cy = math.cos(self.euler_mv.y)   # cos(pitch)
        ty = math.tan(self.euler_mv.y)   # cos(pitch)

        # conversion gyro measurements to roll_rate, pitch_rate, yaw_rate
        self.euler_rate_mv.x = p + sx * ty * q + cx * ty * r
        self.euler_rate_mv.y = cx * q - sx * r
        self.euler_rate_mv.z = sx / cy * q + cx / cy * r

    def euler_ref_cb(self, msg):
        '''
        Euler ref values callback.
        :param msg: Type Vector3 (x-roll, y-pitch, z-yaw)
        '''
        self.euler_sp = msg

    def cfg_callback(self, config, level):
        """ Callback for dynamically reconfigurable parameters (P,I,D gains for each controller)
        """

        if not self.config_start:
            # callback is called for the first time. Use this to set the new params to the config server
            config.roll_kp = self.pid_roll.get_kp()
            config.roll_ki = self.pid_roll.get_ki()
            config.roll_kd = self.pid_roll.get_kd()

            config.roll_r_kp = self.pid_roll_rate.get_kp()
            config.roll_r_ki = self.pid_roll_rate.get_ki()
            config.roll_r_kd = self.pid_roll_rate.get_kd()

            config.pitch_kp = self.pid_pitch.get_kp()
            config.pitch_ki = self.pid_pitch.get_ki()
            config.pitch_kd = self.pid_pitch.get_kd()

            config.pitch_r_kp = self.pid_pitch_rate.get_kp()
            config.pitch_r_ki = self.pid_pitch_rate.get_ki()
            config.pitch_r_kd = self.pid_pitch_rate.get_kd()

            config.yaw_kp = self.pid_yaw.get_kp()
            config.yaw_ki = self.pid_yaw.get_ki()
            config.yaw_kd = self.pid_yaw.get_kd()

            config.yaw_r_kp = self.pid_yaw_rate.get_kp()
            config.yaw_r_ki = self.pid_yaw_rate.get_ki()
            config.yaw_r_kd = self.pid_yaw_rate.get_kd()

            self.config_start = True
        else:
            # The following code just sets up the P,I,D gains for all controllers
            self.pid_roll.set_kp(config.roll_kp)
            self.pid_roll.set_ki(config.roll_ki)
            self.pid_roll.set_kd(config.roll_kd)

            self.pid_roll_rate.set_kp(config.roll_r_kp)
            self.pid_roll_rate.set_ki(config.roll_r_ki)
            self.pid_roll_rate.set_kd(config.roll_r_kd)

            self.pid_pitch.set_kp(config.pitch_kp)
            self.pid_pitch.set_ki(config.pitch_ki)
            self.pid_pitch.set_kd(config.pitch_kd)

            self.pid_pitch_rate.set_kp(config.pitch_r_kp)
            self.pid_pitch_rate.set_ki(config.pitch_r_ki)
            self.pid_pitch_rate.set_kd(config.pitch_r_kd)

            self.pid_yaw.set_kp(config.yaw_kp)
            self.pid_yaw.set_kp(config.yaw_kp)
            self.pid_yaw.set_ki(config.yaw_ki)
            self.pid_yaw.set_kd(config.yaw_kd)

            self.pid_yaw_rate.set_kp(config.yaw_r_kp)
            self.pid_yaw_rate.set_ki(config.yaw_r_ki)
            self.pid_yaw_rate.set_kd(config.yaw_r_kd)

        # this callback should return config data back to server
        return config

if __name__ == '__main__':

    rospy.init_node('mav_attitude_ctl')
    attitude_ctl = AttitudeControl()
    attitude_ctl.run()