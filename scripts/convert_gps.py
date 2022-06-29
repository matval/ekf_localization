#!/usr/bin/env python3

import rospy
from fpn_msgs.msg import FullGPS, GPSPosition, GoalMultiArray
from geometry_msgs.msg import PointStamped
import signal, sys, math

class ConvertGNSS():
    def __init__(self):
        sub_gnss_topic = rospy.get_param('~sub_gps_topic', 'full_gps')
        pub_point_topic = rospy.get_param('~pub_position_topic', 'gps_point')
        pub_position_topic = rospy.get_param('~pub_position_topic', 'gps_position')
        self.gnss_min_acc = rospy.get_param('~gnss_min_acc', 1.0)

        rospy.Subscriber(sub_gnss_topic, FullGPS, self.gps_callback)
        self.pub_point = rospy.Publisher(pub_point_topic, PointStamped, queue_size=1)
        self.pub_gps = rospy.Publisher(pub_position_topic, GPSPosition, queue_size=1)
        self.pub_init_xy_latlon = rospy.Publisher("initial_xy_latlon", GoalMultiArray, queue_size=1)

        self.init_x = -1.0
        self.init_y = -1.0
        self.init_z = -1.0
        self.first_lat = -1.0
        self.first_lon = -1.0
        self.was_invalid = False
        self.last_print_time = rospy.get_time()

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            aux = GoalMultiArray()
            aux.zero_lat = self.first_lat
            aux.zero_lon = self.first_lon
            aux.zero_x = self.init_x
            aux.zero_y = self.init_y
            aux.header.stamp = rospy.Time.now()
            self.pub_init_xy_latlon.publish(aux)

            rate.sleep()

    def gps_callback(self, gnss_msg):
        cur_time = rospy.get_time()
        setWithHighGnssAcc = False

        lat = gnss_msg.latitude
        lon = gnss_msg.longitude
        alt = gnss_msg.altitude
        horizontalAcc = gnss_msg.horizontal_accuracy

        if lat != 0.0 and lon != 0.0 and horizontalAcc < self.gnss_min_acc:
            # If first GPS measurement is good, set first lat/lon
            if (self.first_lat == -1.0 and self.first_lon == -1.0) or setWithHighGnssAcc:
                self.first_lat = lat
                self.first_lon = lon
            
            gps_x, gps_y = self.dlatlon2dxy(self.first_lat, self.first_lon, lat, lon)

            if (self.init_x == -1.0 and self.init_y == -1.0) or setWithHighGnssAcc:
                self.init_x = gps_x
                self.init_y = gps_y
                self.init_z = alt
                setWithHighGnssAcc = False

            gps_point = PointStamped()
            gps_point.header.stamp = rospy.Time.now()
            gps_point.header.frame_id = 'map'
            gps_point.point.x = gps_x + self.init_x
            gps_point.point.y = gps_y + self.init_y
            gps_point.point.z = alt - self.init_z

            self.pub_point.publish(gps_point)

            gps_position = GPSPosition()
            gps_position.header = gps_point.header
            gps_position.position = gps_point.point
            gps_position.gps_accuracy = gnss_msg.horizontal_accuracy
            self.pub_gps.publish(gps_position)
            self.was_invalid = False
        else:
            self.was_invalid = True
            if not self.was_invalid or cur_time-self.last_print_time > 10:
                rospy.logerr("INVALID GNSS MEASUREMENTS!")
                self.last_print_time = cur_time

    def dlatlon2dxy(self, lat1, lon1, lat2, lon2):
        R = 6371000.0
        rlat1 = lat1*math.pi/180.0
        rlat2 = lat2*math.pi/180.0
        rlon1 = lon1*math.pi/180.0
        rlon2 = lon2*math.pi/180.0

        dlat = rlat2 - rlat1
        dlon = rlon2 - rlon1

        dx = R*dlon*math.cos((rlat1+rlat2)/2)
        dy = R*dlat
        return dx, dy

def signal_handler(sig, frame):
    print('convert_gps_node You pressed Ctrl+C!')
    rospy.signal_shutdown('Ctrl+C')
    sys.exit(0)

if __name__ == '__main__':
    rospy.init_node('convert_gps_node', disable_signals=True)
    signal.signal(signal.SIGINT, signal_handler)

    correct = ConvertGNSS()

