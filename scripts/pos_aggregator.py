#!/usr/bin/env python3
import rospy, rospkg, sys, numpy as np, sqlite3
from sqlite3 import Error
from journal_rendezvous.msg import send_pos, array_pos

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

    package_dir = rospkg.RosPack().get_path('journal_rendezvous')
    conn = None
    try:
        conn = sqlite3.connect(package_dir+'/data/data_test.db')
    except Error as e:
        print(e)
    sql = "INSERT INTO Executions(time,map,robot_nr,rendezvous) VALUES(datetime('now'),?,?,?)"
    conn.cursor().execute(sql, (rospy.get_param('world'), robot_number, False))
    conn.commit()
    execution_nr = conn.cursor().execute('SELECT MAX(id) FROM Executions').fetchone()[0] 
    print(f"POS AGGREGATOR: added execution {execution_nr} to db")

    all_pos = array_pos()
    all_pos.positions = [send_pos()]*robot_number
    rospy.init_node('pos_aggregator')
    all_pos_pub = rospy.Publisher('/all_positions', array_pos, queue_size=1)
    odom = rospy.Subscriber('/single_position', send_pos, aggregate)
    pub_update()
    
    rospy.spin()