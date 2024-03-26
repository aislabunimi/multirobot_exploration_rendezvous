#!/usr/bin/env python3
import rospy, sys, rospkg, sqlite3, rosnode, time
from sqlite3 import Error
from test_unknown_rendezvous.msg import send_pos
from nav_msgs.msg import Odometry

def update_pos(odom_update):
    new_pos.robot_id = robot_id
    new_pos.position = odom_update
    new_pos.position.header.frame_id = f'robot{robot_id}/map'

def pub_pos(_):
    single_pos.publish(new_pos)
    
def log_pos(_):
    global positions_buffer
    if len(positions_buffer)<=max_len:
        positions_buffer.append(
            (
                rospy.get_time(),
                new_pos.position.pose.pose.position.x,   #non salvo anche robot_id e execution_nr, sprecherei molto spazio
                new_pos.position.pose.pose.position.y,
            )
        )
    else: #buffer pieno, aggiungo a db
        add_data()
        positions_buffer = [(       #azzero buffer
                rospy.get_time(),
                new_pos.position.pose.pose.position.x,
                new_pos.position.pose.pose.position.y,
            )]

def add_data():
    print(f'[{robot_id}] ADDED POSITION DATA ({len(positions_buffer)}) %%%%%%%%%%%%%%%%%%%%%%%%%%')
    sql = f'INSERT INTO Positions VALUES(?,{robot_id},?,?,{execution_nr})'
    conn.cursor().executemany(sql, positions_buffer)
    conn.commit()
    print(f'[{robot_id}] CONFIRM ADDED DATA POS %%%%%%%%%%%%%%%%%%%%%%%%%%')

if __name__ == '__main__':
    robot_id = int(sys.argv[1])
    rospy.init_node('positions_bd')
    single_pos = rospy.Publisher('/single_position', send_pos, queue_size=10)
    new_pos = send_pos()
    odom = rospy.Subscriber(f'/robot{robot_id}/odom', Odometry, update_pos)
    rospy.Timer(rospy.Duration(0.05), pub_pos)

    max_len = 2500 #numero massimo di posizioni salvate prima di inserirle nel db
    positions_buffer = []

    while not rosnode.rosnode_ping(f'/pos_aggregator', max_count=1, verbose=False):{} #aspetto pos_aggregator 

    db_conn = None
    package_dir = rospkg.RosPack().get_path('test_unknown_rendezvous')
    try:
        conn = sqlite3.connect(package_dir+'/data/data_test.db', check_same_thread=False)
        print(f'[{robot_id}] pos_broadcast: creata connessione db')
    except Error as e:
        print(e)

    cur = conn.cursor()
    cur.execute('SELECT MAX(id) FROM Executions')
    execution_nr = cur.fetchone()[0]

    rospy.Timer(rospy.Duration(0.15), log_pos)

    rospy.on_shutdown(add_data)
    rospy.spin()