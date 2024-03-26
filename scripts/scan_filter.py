#!/usr/bin/env python3
import rospy, re
from math import inf
from sensor_msgs.msg import LaserScan
from test_unknown_rendezvous.msg import cluster
    
def filter_scan(ls):
    if leader:
        pub.publish(ls)
    else:
        new = ls
        new.ranges = [inf] * len(ls.ranges)
        pub.publish(new)
    
def check_leader(cl):
    global leader
    leader = cl.leader

if __name__ == '__main__':
    robot_id = int(re.findall("[0-9]+", rospy.get_namespace())[0])-1
    rospy.init_node('scan_filter')
    leader = True
    pub = rospy.Publisher("filtered_scan", LaserScan, queue_size=10)
    rospy.Subscriber("scan", LaserScan, filter_scan)
    rospy.Subscriber("cluster", cluster, check_leader)
    rospy.spin()