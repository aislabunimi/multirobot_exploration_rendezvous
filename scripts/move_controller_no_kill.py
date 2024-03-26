#!/usr/bin/env python3
import rospy, re, actionlib, tf2_ros, tf2_geometry_msgs, time, random, rospkg, sqlite3, pickle, zstd
from sqlite3 import Error
from shapely import LineString, Point as sPoint, Polygon
from math import isnan
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
from test_unknown_rendezvous.msg import cluster
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Point, Twist
from nav_msgs.msg import Odometry, OccupancyGrid
        
def mb_goal(goal):
    print(f'[{robot_id}] send Goal')
    '''
    Manda il goal a move-base
    '''
    g = MoveBaseGoal()
    g.target_pose.header.frame_id = f'robot{robot_id+1}/map'
    g.target_pose.header.stamp = rospy.Time.now()
    g.target_pose.pose.position = goal
    g.target_pose.pose.orientation.w = 1
    client.send_goal(g, done_cb=reachedGoal)

def reachedGoal(status, _):
    if status==actionlib.GoalStatus.SUCCEEDED:
        print(f'[{robot_id}] GOAL RAGGIUNTO ###########################') #così si nota
        idx = findFrontierByC(old_max_frontier_centroid, other_frontiers)
        if idx!=-1:
            other_frontiers.pop(idx)
    elif status==actionlib.GoalStatus.ABORTED:
        print(f'[{robot_id}] ABORTED ###########################')
    rospy.Timer(rospy.Duration(0.05), updateGoal, oneshot=True)

def getFrontiersEL(l):
    # dato un MarkerArray ne estrae le frontiere in una lista di tuple (centroide, costo, punti della frontiera)
    return list(
        map(
            lambda x: 
                (getCentroid(x.points),
                 getCost(LineString([(p.x,p.y) for p in x.points])),
                 LineString([(p.x, p.y) for p in x.points])
                ),
            filter(
                lambda y: y.type==8,
                l.markers
            )
        )
    )

def getFrontiersB(markerArray):
    # dato un Marker ne estrae le frontiere in una lista di tuple (centroide, costo)
    return [(getCentroid(m.points),
             getCost(LineString([(p.x,p.y) for p in m.points])),
             LineString([(p.x,p.y) for p in m.points])
            ) 
            for m in markerArray.markers]

def updateELFrontiers(frontiersArray): #update Explore Lite frontiers
    global frontiers, my_cluster_aux, other_frontiers, blob_frontiers, marker_id
    if not leader or len(frontiersArray.markers)==0:
        return
    #print(f'[{robot_id}] aggiorno frontiereEL')
    frontiers = getFrontiersEL(frontiersArray)
    if len(my_cluster)!=len(my_cluster_aux) and not EXPLORE_LITE: #qualcuno si è aggiunto al cluster
        new_entries = [c for c in my_cluster if c not in my_cluster_aux]
        for ne in new_entries:
            other_frontiers += [(bo[0],bo[1]*10,bo[2]) for bo in getFrontiersEL(
                rospy.wait_for_message(
                    f'/robot{ne+1}/explore/frontiers',
                    MarkerArray
                )
            )]
        my_cluster_aux += new_entries
    
    MA = MarkerArray()
    MA.markers = [getMarker(marker_id-1, [], remove=True)]
    debug.publish(MA)
    MA.markers = [getMarker(marker_id, [(f[0].x, f[0].y) for f in (frontiers+blob_frontiers+other_frontiers)])]
    #MA.markers = [getMarker(marker_id, [(c[0],c[1]) for f in all_frontiers for c in f[2].coords])]
    debug.publish(MA)
    marker_id += 1

def updateBFrontiers(markerArray):
    global blob_frontiers
    if not leader or len(markerArray.markers)==0 or markerArray.markers[0].action==2:
        return
    blob_frontiers = [f for f in getFrontiersB(markerArray) if not isnan(f[1])]

def updateGoal(_):
    global old_max_frontier_centroid, old_max_frontier, prev_distance, last_progress, marker_id, last_try
    global execution_nr, goal_state
    print(f'[{robot_id}] updateGoal()')
    if FINISH:
        client.cancel_all_goals()
        print("RENDEZVOUS!")
        return
    if not leader:
        MA = MarkerArray()
        MA.markers = [getMarker(marker_id-1, [], remove=True)]
        debug.publish(MA)
        return
    all_frontiers = frontiers + other_frontiers + blob_frontiers

    if not visited_area.is_empty:
        i=0
        length = len(all_frontiers)
        while i < length:
            try:
                if visited_area.contains(all_frontiers[i][2]):
                    print(f'[{robot_id}] rimozione frontiera')
                    all_frontiers.pop(i) #tolgo
                    i-=1
                    length-=1
            except:
                print(f'f: {frontiers} || oth: {other_frontiers} || b: {blob_frontiers}')
            i+=1

    if len(all_frontiers)==0: #frontiere vuote
        if last_try: #ha provato a girarsi ma niente, termino exploration
            print(f'[{robot_id}] frontiere finite')
            return
        turn_around()
        updateELFrontiers(rospy.wait_for_message('explore/frontiers', MarkerArray))
        last_try = True
        time.sleep(3) #girati per 3 secondi
        rospy.Timer(rospy.Duration(0.05), updateGoal, oneshot=True)
        return
    all_frontiers.sort(key=lambda x: x[1], reverse=True)

    same_goal = old_max_frontier_centroid==all_frontiers[0][0]
    old_max_frontier_centroid = all_frontiers[0][0]
    old_max_frontier = all_frontiers[0][2]
    robotFrontierDistance = getRobotDistance(old_max_frontier)
    if not same_goal:
        add_data(
            rospy.get_time(),
            robot_id+1,
            zstd.compress(pickle.dumps(all_frontiers)),
            execution_nr
        )
    if (not same_goal) or prev_distance>robotFrontierDistance:
        print(f'[{robot_id}] if 1')
        prev_distance = robotFrontierDistance
        last_progress = rospy.Time.now()
    print(f'[{robot_id}] tempo passato: {rospy.Time.to_sec(rospy.Time.now() - last_progress)}')
    if rospy.Time.to_sec(rospy.Time.now() - last_progress)>max_time: #frontiera complicata, cambio
        print(f'[{robot_id}] if 2')
        mb_goal(all_frontiers[1][0])
        return
    if same_goal:
        if goal_state%goal_rate==0:
            mb_goal(all_frontiers[0][0])
        goal_state+=1
        return
    mb_goal(all_frontiers[0][0])
    rospy.Timer(rospy.Duration(0.05), updateGoal, oneshot=True)

def findFrontierByC(C,L): #restituisce l'indice della frontiera nella lista L con centroide C
    for i,f in enumerate(L):
        if f[0]==C:
            return i
    return -1

def getCost(frontier): # preso una pose (Point) e una frontiera (Linestring) calcola il costo
    return (potential_scale*getRobotDistance(frontier) +
            gain_scale*frontier.length)

def getRobotDistance(frontier): #distanza minima del robot dalla frontiera
    pose_tf = tf2_geometry_msgs.do_transform_pose(getPoseStamped(my_pose), transform)
    easyPose = pose_tf.pose.position
    return sPoint((easyPose.x, easyPose.y)).distance(frontier)

def getCentroid(points): #presi una lista di Point ne restituisce il centroide
    if len(points)==0:
        return []
    P = Point()
    P.x, P.y = [p2 for p2 in LineString([(p.x, p.y) for p in points]).centroid.coords][0]
    P.z = 0
    return P
    
def getPoseStamped(covPos): #preso un Odometry restistuisce il relativo PoseStamped
    new = PoseStamped()
    new.header = covPos.header
    new.pose = covPos.pose.pose
    return new
    
def updateTransform(_):
    global transform
    try:
        transform = tf_buffer.lookup_transform(
            f'robot{robot_id+1}/map',
            f'robot{robot_id+1}/odom',
            rospy.Time()
        )
    #print(f"[{robot_id}] transform fatta")
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        {'''print(f"[{robot_id}] transform NON fatta")'''}

def updateMyPose(odom):
    global my_pose
    my_pose = odom

def updateArea(marker):
    global visited_area
    visited_area = Polygon(
        [(p.x, p.y) for p in marker.points]
    )

def turn_around(): #gira il robot
    msg=Twist()
    msg.linear.x = 0
    msg.angular.z = -1.3
    cmd_vel_pub.publish(msg)

#positions = [(x1,y1),(x2,y2)...]
def getMarker(id, positions, remove=False, frontier=False):
    M = Marker()
    M.header.frame_id = "map"
    M.header.stamp = rospy.Time.now()
    M.ns = f"robot{robot_id}"
    M.type = Marker.SPHERE_LIST
    M.action = Marker.ADD if not remove else Marker.DELETE
    M.id = id
    for pos in positions:
        P = Point()
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
    M.scale.x = 0.4
    M.scale.y = 0.4
    M.scale.z = 0.4
    M.color = colors[robot_id%len(colors)]
    return M

def getColorRGB(rgb):
    r,g,b = rgb
    C = ColorRGBA()
    C.r = r/255
    C.g = g/255
    C.b = b/255
    C.a = 1
    return C

def update_cluster(update):
    global my_cluster, leader, FINISH
    my_cluster = update.robots
    leader = update.leader
    FINISH = update.all_together
    if len(frontiers)==0:
        updateELFrontiers(rospy.wait_for_message('explore/frontiers', MarkerArray))
    updateGoal(0)

def add_data(time, robot, frontiers, execution):
    sql = 'INSERT INTO Exploration VALUES(?,?,?,?)'
    conn.cursor().execute(sql, (time, robot, frontiers, execution))
    conn.commit()

if __name__ == '__main__':
    time.sleep(0)
    robot_id = int(re.findall("[0-9]+", rospy.get_namespace())[0])-1
    rospy.init_node('move_controller')
    
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

    EXPLORE_LITE = rospy.get_param('/explore-lite')

    colors = [(255,0,255),(210,105,30),(34,139,34),(30,144,255),(220,20,60),(0,0,0)]
    colors = [getColorRGB(c) for c in colors]

    frontiers = [] #[(centroide1,costo1,frontiera1), ..., (centroide_n,costo_n,frontiera_n)]
    other_frontiers = [] #frontiere ereditate dal cluster
    blob_frontiers = [] #uguale a frontiers

    visited_area = Polygon() #area visitata (blob)

    old_max_frontier_centroid = Point()  #ultima frontiera scelta come goal (centroide)
    old_max_frontier = LineString() #ultima frontiera scelta come goal (figura)
    prev_distance = 0 #distanza del robot dall'old_max_frontier
    max_time = 15 # tempo massimo in secondi in cui il robot cerca di raggiungere la frontiera
    last_progress = rospy.Time.now()
    last_try = False #se le frontiere sono vuote, prima di bloccare provo a farlo girare

    my_pose = Odometry()
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    transform = tf2_ros.TransformStamped()
    rospy.Timer(rospy.Duration(0.33), updateTransform) #aggiorna la trasform tf

    gain_scale = rospy.get_param(f'/robot{robot_id+1}/explore/gain_scale')
    potential_scale = rospy.get_param(f'/robot{robot_id+1}/explore/potential_scale')

    my_cluster = [robot_id]
    my_cluster_aux = [robot_id]
    leader = True
    FINISH = False #RENDEVZOUS!

    marker_id = 0

    goal_rate = 5
    goal_state = 0

    #rospy.Timer(rospy.Duration(1), updateGoal)

    debug = rospy.Publisher('frontiers_debug', MarkerArray, queue_size=10)
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)

    rospy.wait_for_message('move_base/local_costmap/costmap', OccupancyGrid)

    rospy.Subscriber('odom', Odometry, updateMyPose)
    rospy.Subscriber('explore/frontiers', MarkerArray, updateELFrontiers)
    rospy.Subscriber('cluster', cluster, update_cluster)
    if not EXPLORE_LITE:
        rospy.Subscriber('blob_frontiers', MarkerArray, updateBFrontiers)
        rospy.Subscriber('visited_area', Marker, updateArea)

    try:
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()
        print(f'[{robot_id}] move-base raggiunto')
    except rospy.ROSInterruptException:
            rospy.loginfo("Navigation finished.")

    updateELFrontiers(rospy.wait_for_message('explore/frontiers', MarkerArray)) #aspetta di ricevere le prime frontiere
    
    for i in range(20):
        updateGoal(0) #inizia a muoversi
        time.sleep(.5)

    rospy.spin()