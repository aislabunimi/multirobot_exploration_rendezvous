import rospy, sys, time, tf2_ros, tf2_geometry_msgs, sqlite3, rospkg, pickle, zstd
from sqlite3 import Error
from shapely.geometry import Point, Polygon
from shapely.ops import unary_union
from shapely import affinity
from shapely import set_precision
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from test_unknown_rendezvous.msg import cluster, points, point_sec
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point as RosPoint, PoseStamped
from std_msgs.msg import ColorRGBA
import matplotlib.pyplot as plt

def update_pos(now_pos):
    global actual_pos, actual_pos_xy
    actual_pos_xy = (now_pos.pose.pose.position.x, now_pos.pose.pose.position.y)
    actual_pos = now_pos

def update_area(_):
    global visited_points, visited_area, marker_id, transform, my_cluster_aux, last_area_marker
    #print(list(map(lambda x: Point(x['point']), visited_points)))
    if leader:
        if len(my_cluster_aux) != len(my_cluster): #cluster cambiato
            new_entries = [c for c in my_cluster if c not in my_cluster_aux]
            print(f"[{robot_id}] new_entries: {new_entries}")
            for ne in new_entries:
                new_points = rospy.wait_for_message(f'/robot{ne+1}/visited_points', points, 10).points
                visited_points += [[
                    {'time': np.time,
                     'point': (np.point.x,np.point.y), 
                     'origin': (np.point.x,np.point.y)} 
                    for np in new_points
                ]] #unisco punti visitati
            my_cluster_aux += new_entries
        try:
            transform = tf_buffer.lookup_transform(
                f'robot{robot_id}/map',
                f'robot{robot_id}/odom',
                rospy.Time(),
                rospy.Duration(1.0)
            )
        #print(f"[{robot_id}] transform fatta")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            {'''print(f"[{robot_id}] transform NON fatta")'''}
        pose_transformed = tf2_geometry_msgs.do_transform_pose(getPoseStamped(actual_pos), transform)
        x,y = pose_transformed.pose.position.x, pose_transformed.pose.position.y
        o_x, o_y = actual_pos_xy
        diff_x, diff_y = o_x-x, o_y-y
        for v2 in visited_points:
            for v in v2:
                v['point'] = (v['origin'][0]-diff_x, v['origin'][1]-diff_y)
        if len(flat(visited_points))>3:
            information_decay(-diff_x, -diff_y)
        visited_points[0] += [{'time': rospy.get_time(), 'point':(o_x-diff_x, o_y-diff_y) , 'origin':actual_pos_xy }]
        visited_area = unary_union(list(map(lambda x: Point(x['point']).buffer(raggio), flat(visited_points))))
        marker_area_pub.publish(getMarker(last_area_marker, [], remove=True))
        marker_area_pub.publish(getMarker(marker_id, visited_area.exterior.coords))

        add_data(
            rospy.get_time(),
            robot_id,
            pickle.dumps([vp['origin'] for vp in flat(visited_points)]),
            raggio,
            execution_nr
        )

        last_area_marker = marker_id
        marker_id += 1
    else:
        marker_area_pub.publish(getMarker(last_area_marker, [], remove=True))
        if len(flat(visited_points))>2:
            information_decay(0,0)
    vp_pub = points()
    vp_pub.points = []
    for vp in flat(visited_points):
        x,y = vp['origin']
        RP =  point_sec()
        RP.time = vp['time']
        RP.point.x = x
        RP.point.y = y
        RP.point.z = 0
        vp_pub.points += [RP]
    visited_points_pub.publish(vp_pub)

def information_decay(offset_x, offset_y):
    global visited_points, marker_id, first_frontier_marker, id_frontiers, frontiers_count
    now = rospy.get_time()
    single_blob_area = Polygon()  #blob dei singoli robot
    for i2 in range(len(visited_points)):
        i = 0
        last_i_near = (0,0) #punto successivo all'ultimo punto tolto
        to_remove = [] #lista dei punti (x,y) da rimuovere
        while i<len(visited_points[i2]):
            if (-coeff)*(now-visited_points[i2][i]['time'])+1<0: #rimuovo porzione (information decay)
                to_remove += [visited_points[i2][i]['origin']]
                if len(visited_points[i2])>1:
                    last_i_near = visited_points[i2][i+1]['origin']
                visited_points[i2].pop(i)
                i -= 1
            i += 1
        if len(to_remove)>0:
            single_blob_area = unary_union([Point(vp['origin']).buffer(raggio)  #blobarea senza l'ultimo punto
                                         for vp in visited_points[i2] 
                                         if vp['origin']!=last_i_near]+[single_blob_area])
            if frontiers_count%frontier_rate==0:
                blob_area_full = unary_union([Point(vp['origin']).buffer(raggio) for vp in visited_points[i2]])
                diff = unary_union([Point(tr).buffer(raggio) for tr in to_remove]).difference(blob_area_full)
                frontier = set_precision(blob_area_full.intersection(diff), 0.05)
                id_frontiers += [frontier]
    f_to_remove = [] #lista degli indici delle frontiere da rimuovere
    for idx,f in enumerate(id_frontiers):
        id_frontiers[idx] = f.difference(single_blob_area) #tolgo parte di frontiera dentro blob
        if f.length<frontier_min_size:  #tolgo frontiere troppo piccole
            f_to_remove += [idx]
    [id_frontiers.pop(f_to_remove[idx]-idx) for idx,_ in enumerate(f_to_remove)] #tolgo frontiere troppo piccole
    #print(f"[{robot_id}] rimossi {c} blob al tempo {now}")
    if len(id_frontiers)>0:
        MA = MarkerArray()
        MA.markers = [getMarker(idx,[],remove=True) for idx in range(first_frontier_marker,marker_id-1)]
        marker_frontiers_pub.publish(MA)
        MA.markers = [
            getMarker(marker_id+idx, extract_points(affinity.translate(l,offset_x,offset_y)), frontier=True)
            for idx,l in enumerate(id_frontiers)
            ]
        if len(MA.markers)==1 and len(MA.markers[0].points)==0:
            return
        marker_frontiers_pub.publish(MA)
        first_frontier_marker = marker_id
        marker_id += len(MA.markers)
    frontiers_count += 1 
    
def extract_points(frontier): #motivo dell'esistenza: shapely decide a caso il tipo
    points = []
    try:
        points = frontier.exterior.coords #se Polygon
    except (AttributeError, NotImplementedError):
        try:
            points = frontier.coords #se LineString
        except (AttributeError, NotImplementedError):
            points = [point for g in frontier.geoms for point in g.coords] #se non Polygon
    return points

def flat (l): #appiattisce un array di array (doppio) [[1],[2]] -> [1,2]
    return [j for i in l for j in i]

def update_cluster(update):
    global my_cluster, leader
    my_cluster = update.robots
    leader = update.leader

def getPoseStamped(covPos): #preso un Odometry restistuisce il relativo PoseStamped
    new = PoseStamped()
    new.header = covPos.header
    new.pose = covPos.pose.pose
    return new

#positions = [(x1,y1),(x2,y2)...]
def getMarker(id, positions, remove=False, frontier=False):
    M = Marker()
    M.header.frame_id = "map"
    M.header.stamp = rospy.Time.now()
    M.ns = f"robot{robot_id}"
    M.type = Marker.POINTS
    M.action = Marker.ADD if not remove else Marker.DELETE
    M.id = id
    for pos in positions:
        P = RosPoint()
        x,y = pos
        P.x = x
        P.y = y
        P.z = 0
        M.points += [P]
    #M.pose.position.x = x
    #M.pose.position.y = y
    #M.pose.position.z = 0
    #M.pose.orientation.x = 0
    #M.pose.orientation.y = 0
    #M.pose.orientation.z = 0
    M.pose.orientation.w = 1
    M.scale.x = 0.05
    M.scale.y = 0.05
    M.scale.z = 0.05
    M.color = colors[(robot_id-1)%4] if not frontier else getColorRGB((255,0,0))
    return M

def getColorRGB(rgb):
    r,g,b = rgb
    C = ColorRGBA()
    C.r = r/255
    C.g = g/255
    C.b = b/255
    C.a = 1
    return C

def plot_area(_):
    plt.plot(*visited_area.exterior.xy, "-k")
    plt.show()

def add_data(time, robot, visitedPoints, raggio, execution):
    sql = 'INSERT INTO VisitedPoints VALUES(?,?,?,?,?)'
    conn.cursor().execute(sql, (time, robot, visitedPoints, raggio, execution))
    conn.commit()

if __name__ == '__main__':
    robot_id = int(sys.argv[1])
    colors = [(0,255,0),(0,255,255),(255,20,147),(255,255,0)]
    colors = list(map(getColorRGB,colors))
    visited_points = [[]] #punti visitati
    visited_area = Polygon([])
    id_frontiers = [] #information decay frontiers
    frontiers_count = 0 #numero frontiere generate
    frontier_rate = 9
    frontier_min_size = 1.5
    leader = True
    my_cluster = [robot_id-1]
    my_cluster_aux = [robot_id-1]
    coeff = 0.003 #Bartiromo
    raggio = 1.5
    actual_pos_xy = (0,0) #Posizione attuale sempre aggiornata
    actual_pos = Odometry()
    source_header = ""
    marker_id = 0
    last_area_marker = 0
    first_frontier_marker = 0
    rospy.init_node('visited_area')

    db_conn = None
    package_dir = rospkg.RosPack().get_path('test_unknown_rendezvous')
    try:
        conn = sqlite3.connect(package_dir+'/data/data_test.db', check_same_thread=False)
        print(f'Cluster Controller: creata connessione db')
    except Error as e:
        print(e)

    cur = conn.cursor()
    cur.execute('SELECT MAX(id) FROM Executions')
    execution_nr = cur.fetchone()[0]

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    transform = tf2_ros.TransformStamped()
    rospy.wait_for_service('/gazebo/spawn_urdf_model') #Mi assicuro lo spawn del robot
    time.sleep(2)
    rospy.Subscriber('cluster', cluster, update_cluster)
    rospy.Subscriber(f'/robot{robot_id}/odom', Odometry, update_pos)
    marker_area_pub = rospy.Publisher('visited_area', Marker, queue_size=5)
    marker_frontiers_pub = rospy.Publisher('blob_frontiers', MarkerArray, queue_size=5)
    visited_points_pub = rospy.Publisher('visited_points', points, queue_size=10)
    rospy.Timer(rospy.Duration(2), update_area) #non riesce ad accedere ai campi dei valori nella callback
    #rospy.Timer(rospy.Duration(5), plot_area)
    rospy.spin()