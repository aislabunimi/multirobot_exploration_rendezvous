import sqlite3, rospy, pickle, zstd, rospkg
from sqlite3 import Error
from nav_msgs.msg import OccupancyGrid, Odometry

def updateMap(m):
    global map
    map = m

def update_pos(odom_update):
    global new_pos
    new_pos = odom_update

def add_map(_):
    print('aggiungengo mappa')
    sql = 'INSERT INTO RawMaps VALUES(?,?)'
    conn.cursor().execute(sql, (map_name, zstd.compress(pickle.dumps(map))))
    conn.commit()
    print('mappa aggiunta')

if __name__ == '__main__':
    rospy.init_node('save_map')

    map = OccupancyGrid()
    new_pos = Odometry()

    db_conn = None
    package_dir = rospkg.RosPack().get_path('test_unknown_rendezvous')
    try:
        conn = sqlite3.connect('/home/aislab/Documents/Tellaroli/Dati esperimenti tesi/data.db', check_same_thread=False)
        print(f'Cluster Controller: creata connessione db')
    except Error as e:
        print(e)
    
    #odom = rospy.Subscriber(f'/robot1/odom', Odometry, update_pos)
    rospy.Subscriber('robot1/map', OccupancyGrid, updateMap)

    map_name = rospy.get_param('map_name')

    rospy.Timer(rospy.Duration(2.5), add_map)

    rospy.spin()

    #while not rospy.is_shutdown(): {}
#
    #sql = 'INSERT INTO FinalMaps VALUES(?,?,?,?)'
#
    #conn.cursor().execute(sql, (249, 4, zstd.compress((pickle.dumps(map))), 1818))
    #conn.commit()