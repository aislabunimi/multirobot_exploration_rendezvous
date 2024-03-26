import sqlite3, rospy, pickle, zstd, rospkg
from sqlite3 import Error
from nav_msgs.msg import OccupancyGrid

def callback(_):
    pub.publish(map)

if __name__ == '__main__':
    rospy.init_node('public_map')

    db_conn = None
    package_dir = rospkg.RosPack().get_path('test_unknown_rendezvous')
    try:
        conn = sqlite3.connect(package_dir+'/data/data_test.db', check_same_thread=False)
        print(f'Cluster Controller: creata connessione db')
    except Error as e:
        print(e)
    
    map_name = rospy.get_param('map_name')

    cur = conn.cursor()
    cur.execute(f'select map from RawMaps where name="{map_name}"')
    map = pickle.loads(zstd.decompress(cur.fetchone()[0]))

    pub = rospy.Publisher('map', OccupancyGrid, queue_size=10)

    rospy.Timer(rospy.Duration(2), callback)

    rospy.spin()