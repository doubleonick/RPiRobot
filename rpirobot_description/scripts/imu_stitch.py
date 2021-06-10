#! /usr/bin/python
from __future__ import print_function

# Based heavily on the FLIR Trigger_QuickSpin.py example code

#import PySpin
import rospy
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
#import cv2
#import os
#import datetime

#from multiprocessing.pool import ThreadPool
#from collections import deque

#from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool
from std_msgs.msg import String  
#from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import Imu
#from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64    
            
class Realsense_imu:

    def __init__(self):
        self.timestamp = 0
        #self.data_dir = "/tmp/BHG_DATA"
        #self.image_folder = self.data_dir + '/FLIR/'
        #self.image_filename = "default_flir_imgname.png"
        #self.csv_filename = "default_flir.csv"
        #self.datetimeData = ""
        #self.is_recording = False
        #self.imu_mag = MagneticField()
        self.imu_data = Imu()
        #self.vel_gps = TwistStamped()
        #self.threadn = cv2.getNumberOfCPUs()
        #self.pool = ThreadPool(processes = self.threadn)
        #self.pending = deque()
        #self.threaded_mode = _threaded_mode        
        self.gyro_data  = Imu()
        self.accel_data = Imu()
        
        #rospy.Subscriber('/directory', String, self.directory_callback)
        #rospy.Subscriber("/record", Bool, self.record_callback)
        #rospy.Subscriber("/mavros/altitude", Altitude, self.alt_cb)
        #rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, self.gps_cb)
        #rospy.Subscriber("/mavros/imu/mag", MagneticField, self.mag_cb)
        #rospy.Subscriber("/mavros/imu/data", Imu, self.imu_cb)
        #rospy.Subscriber("/mavros/global_position/raw/gps_vel", TwistStamped, self.vel_cb)
        #rospy.Subscriber("/mavros/imu/temperature_imu", Temperature, self.temp_cb)
        rospy.Subscriber("/camera/gyro/sample", Imu, self.gyro_cb)
        rospy.Subscriber("/camera/accel/sample", Imu, self.accel_cb)

    '''    
    # Helper function for multi threaded 
    class DummyTask:
        def __init__(self, data):
            self.data = data
        def ready(self):
            return True
        def get(self):
            return self.data
    '''
    def gyro_cb(self, msg):
        self.gyro_data = msg

    def accel_cb(self, msg):
        self.accel_data = msg

    def imu_pub(self):
        pub = rospy.Publisher('imu_data', Imu)
        #rospy.init_node('imu_talker')
        r = rospy.Rate(10) #10hz
        orientation = Quaternion()
        orientation.x = 0.0
        orientation.y = 0.0
        orientation.z = 0.0
        orientation.w = 0.0

        msg = Imu()
        msg.orientation      = orientation
        msg.angular_velocity = self.gyro_data.angular_velocity
        msg.linear_acceleration  = self.accel_data.linear_acceleration
        #rospy.loginfo(msg)
        pub.publish(msg)

if __name__ == "__main__":
        """
        Example entry point; please see Enumeration example for more in-depth
        comments on preparing and cleaning up the system.
        :return: True if successful, False otherwise.
        :rtype: bool
        """
        rospy.init_node('imu_stitcher')
        realsense_imu = Realsense_imu()
        r = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown(): 
            realsense_imu.imu_pub()        
        
