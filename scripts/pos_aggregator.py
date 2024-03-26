#!/usr/bin/env python3
import rospy, rospkg, sys, numpy as np, sqlite3
from sqlite3 import Error
from test_unknown_rendezvous.msg import send_pos, array_pos

def aggregate(single_pos):
    global count
    all_pos.positions[single_pos.robot_id-1] = single_pos

def pub_update():
    r = rospy.Rate(10) #Hz
    while not rospy.is_shutdown():
        all_pos_pub.publish(all_pos)
        r.sleep()
        
if __name__ == '__main__':
    robot_number = int(sys.argv[1]) #numero di robot usati
    count = 0

    package_dir = rospkg.RosPack().get_path('test_unknown_rendezvous')
    conn = None
    try:
        conn = sqlite3.connect(package_dir+'/data/data_test.db')
    except Error as e:
        print(e)
    sql = "INSERT INTO Executions(time) VALUES(datetime('now'))"
    conn.cursor().execute(sql)
    conn.commit()

    all_pos = array_pos()
    all_pos.positions = [send_pos()]*robot_number
    rospy.init_node('pos_aggregator')
    all_pos_pub = rospy.Publisher('/all_positions', array_pos, queue_size=1)
    odom = rospy.Subscriber('/single_position', send_pos, aggregate)
    pub_update()
    
    rospy.spin()