#!/usr/bin/env python3
import rospy, re, actionlib, tf2_ros, tf2_geometry_msgs, time, random, rospkg, sqlite3, pickle, zstd, os, rosnode
from sqlite3 import Error
from shapely import LineString, Point as sPoint, Polygon
from math import isnan
from numpy import round as npRound
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
from test_unknown_rendezvous.msg import cluster, logging
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Point, Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from gazebo_msgs.srv import DeleteModel # For deleting models from the environment
        
def mb_goal(goal):
    print(f'[{robot_id}] send Goal')
    '''
    Manda il goal a move-base
    '''
    print(f'[{robot_id}] GOAL: {(round(goal.x,2), round(goal.y,2))} ###########################') #così si nota
    g = MoveBaseGoal()
    g.target_pose.header.frame_id = f'robot{robot_id+1}/map'
    g.target_pose.header.stamp = rospy.Time.now()
    g.target_pose.pose.position = goal
    g.target_pose.pose.orientation.w = 1
    client.send_goal(g, done_cb=reachedGoal)

def reachedGoal(status, _):
    global banned_frontiers
    if status==actionlib.GoalStatus.SUCCEEDED:
        print(f'[{robot_id}] GOAL RAGGIUNTO ###########################') #così si nota
        idx = findFrontierByC(old_max_frontier_centroid, other_frontiers)
        if idx!=-1:
            print(f'[{robot_id}] rimossa frontiera da other {(old_max_frontier_centroid.x, old_max_frontier_centroid.y)}')
            other_frontiers.pop(idx)
    elif status==actionlib.GoalStatus.ABORTED:
        print(f'[{robot_id}] ABORTED ###########################')
    rospy.Timer(rospy.Duration(0.05), updateGoal, oneshot=True)

def getFrontiersEL(l):
    # dato un MarkerArray ne estrae le frontiere in una lista di tuple (centroide, costo, punti della frontiera, frontiera_blob)
    return list(
        map(
            lambda x: 
                (getCentroid(x.points),
                 getCost(LineString([(p.x,p.y) for p in x.points])),
                 LineString([(p.x, p.y) for p in x.points]),
                 False
                ),
            filter(
                lambda y: y.type==8,
                l.markers
            )
        )
    )

def getFrontiersB(markerArray):
    # dato un Marker ne estrae le frontiere in una lista di tuple (centroide, costo, frontiera, frontiera_blob)
    return [(getCentroid(m.points),
             getCost(LineString([(p.x,p.y) for p in m.points]), blob=True),
             LineString([(p.x,p.y) for p in m.points]),
             True
            ) 
            for m in markerArray.markers]

def updateELFrontiers(frontiersArray): #update Explore Lite frontiers
    global frontiers, my_cluster_aux, other_frontiers, blob_frontiers, marker_id
    if not leader or len(frontiersArray.markers)==0:
        print(f'[{robot_id}] not leader')
        return
    frontiers = getFrontiersEL(frontiersArray)
    #print(f'[{robot_id}] {[(round(f[0].x,2), round(f[0].y)) for f in frontiers]}')
    if len(my_cluster)!=len(my_cluster_aux) and not EXPLORE_LITE: #qualcuno si è aggiunto al cluster
        new_entries = [c for c in my_cluster if c not in my_cluster_aux]
        for ne in new_entries:
            try:
                other_frontiers += [(bo[0],bo[1]*10,bo[2]) for bo in getFrontiersEL(
                    rospy.wait_for_message(
                        f'/robot{ne+1}/explore/frontiers',
                        MarkerArray,
                        timeout = 5
                    )
                )]
            except:
                print(f'[{robot_id}] Frontiere non ricevute XXXXXXXXXXXXX')
        my_cluster_aux += new_entries
    
    MA = MarkerArray()
    MA.markers = [getMarker(marker_id-1, [], remove=True)]
    debug.publish(MA)
    MA.markers = [
        getMarker(
            marker_id, 
            [(f[0].x, f[0].y) 
             for f in (frontiers+blob_frontiers+other_frontiers)
             if (f[0].x, f[0].y) not in banned_frontiers
            ]
        )
    ]
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
    global execution_nr, goal_state, banned_frontiers
    print(f'[{robot_id}] updateGoal()')
    if BLOCCATO:
        return
    if FINISH:
        client.cancel_all_goals()
        print("RENDEZVOUS!")
        log_str = logging()
        log_str.str = f"RENDEZVOUS"
        topic_logger.publish(log_str)
        add_final_map(
            execution_nr,
            robot_id+1,
            zstd.compress(
                pickle.dumps(rospy.wait_for_message(f'/robot{robot_id+1}/map', OccupancyGrid))
            ),
            rospy.get_time()
        )
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
                if visited_area.contains(sPoint(point_to_tuple(all_frontiers[i][0]))):
                    print(f'[{robot_id}] rimozione frontiera {(round(all_frontiers[i][0].x,2),round(all_frontiers[i][0].y,2))}')
                    banned_frontiers[point_to_tuple(all_frontiers[i][0])]=True
                    #other_frontiers.pop(i) #tolgo
                    #i-=1
                    #length-=1
            except e:
                print(e)
                #print(f'f: {frontiers} || oth: {other_frontiers} || b: {blob_frontiers}')
            i+=1

    #print(f"[{robot_id}] {[f[1] for f in frontiers]} {[f[1] for f in blob_frontiers]}")

    if len(all_frontiers)==0: #frontiere vuote
        if last_try: #ha provato a girarsi ma niente, termino exploration
            print(f'[{robot_id}] frontiere finite')
            MA = MarkerArray()
            MA.markers = [getMarker(marker_id-1, [], remove=True)]
            debug.publish(MA)
            return
        turn_around()
        for _ in range(3):
            updateELFrontiers(rospy.wait_for_message('explore/frontiers', MarkerArray))
        last_try = True
        time.sleep(3) #girati per 3 secondi
        rospy.Timer(rospy.Duration(0.05), updateGoal, oneshot=True)
        return
    all_frontiers.sort(key=lambda x: x[1], reverse=True)

    i=0
    while i<len(all_frontiers): #non seleziono frontiere bannate
        if point_to_tuple(all_frontiers[i][0]) not in banned_frontiers:
            goal = all_frontiers[i][0]
            break
        i+=1

    same_goal = old_max_frontier_centroid==goal
    old_max_frontier_centroid = goal
    old_max_frontier = all_frontiers[i][2]
    robotFrontierDistance = getRobotDistance(old_max_frontier)
    if not same_goal:
        print(f'[{robot_id}] add DB frontiers $$$$$$$')
        add_data(
            rospy.get_time(),
            robot_id+1,
            zstd.compress(pickle.dumps(all_frontiers)),
            execution_nr,
            all_frontiers[i][-1] #boolean che indica se è una frontiera generata dal blobbing
        )
    if (not same_goal) or prev_distance>robotFrontierDistance:
        print(f'[{robot_id}] if 1')
        prev_distance = robotFrontierDistance
        last_progress = rospy.Time.now()
    print(f'[{robot_id}] tempo passato: {rospy.Time.to_sec(rospy.Time.now() - last_progress)}')
    if rospy.Time.to_sec(rospy.Time.now() - last_progress)>max_time: #frontiera complicata, cambio
        time.sleep(1)
        print(f'[{robot_id}] if 2')
        print(f'[{robot_id}] {[(round(x[0].x,3),round(x[0].y,3)) for x in all_frontiers]}')
        banned_frontiers[point_to_tuple(all_frontiers[i][0])]=True
        last_progress = rospy.Time.now()
        return
    if same_goal:
        if goal_state%goal_rate==0:
            mb_goal(goal)
        goal_state+=1
        return
    mb_goal(goal)
    rospy.Timer(rospy.Duration(0.05), updateGoal, oneshot=True)

def findFrontierByC(C,L): #restituisce l'indice della frontiera nella lista L con centroide C
    for i,f in enumerate(L):
        if f[0]==C:
            return i
    return -1

def getFrontierLenght(f, blob):
    return len(f.coords)*.05 if not blob else f.length

def getCost(frontier, blob=False): # preso una pose (Point) e una frontiera (Linestring) calcola il costo
    return (gain_scale*getFrontierLenght(frontier,blob) - potential_scale*getRobotDistance(frontier))

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

def point_to_tuple(P):
    return (P.x,P.y)

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
    global my_pose, last_my_pose, last_my_pose_time, incastrato, BLOCCATO
    if FINISH or BLOCCATO:
        return
    my_pose_rounded = npRound((odom.pose.pose.position.x, odom.pose.pose.position.y),1)
    same_pose = all(my_pose_rounded == last_my_pose)
    if not same_pose:
        last_my_pose_time = rospy.get_time()
        last_my_pose = my_pose_rounded
    if rospy.get_time()-last_my_pose_time > max_time_stationary:
        print(f'[{robot_id}] INCASTRATO')
        send_cmd_vel(-.1**random.randint(1,2),0)
        time.sleep(.05)
        send_cmd_vel(0,0)
        updateGoal(0)
    if rospy.get_time()-last_my_pose_time > time_to_be_stucked:
        log_str = logging()
        log_str.str = f"[{round(rospy.get_time()-start_time,2)}] \t\trobot {robot_id} BLOCCATO"
        topic_logger.publish(log_str)
        BLOCCATO = True
    my_pose = odom

def updateArea(marker):
    global visited_area
    visited_area = Polygon(
        [(p.x, p.y) for p in marker.points]
    )
    
def send_cmd_vel(x,z):
    msg=Twist()
    msg.linear.x = x
    msg.angular.z = z
    cmd_vel_pub.publish(msg)

def turn_around(): #gira il robot
    send_cmd_vel(0,-1.3)

#positions = [(x1,y1),(x2,y2)...]
def getMarker(id, positions, remove=False):
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
    
    if not leader:
        add_final_map(
            execution_nr,
            robot_id+1,
            zstd.compress(
                pickle.dumps(rospy.wait_for_message(f'/robot{robot_id+1}/map', OccupancyGrid))
            ),
            rospy.get_time()
        )
        time.sleep(5)
        #rospy.wait_for_message('explore/frontiers', MarkerArray) #prima di ucciderlo aspetto mandi le frontiere al leader
        del_model_prox = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        del_model_prox(f'turtlebot3_robot{robot_id+1}')
        nodes = os.popen(f'rosnode list | grep robot"{robot_id+1}"').readlines()
        for i in range(len(nodes)):
            nodes[i] = nodes[i].replace("\n","")
        for node in nodes:
            os.system("rosnode kill "+ node)
        rospy.signal_shutdown("")
        return

    FINISH = update.all_together
    if len(frontiers)==0:
        updateELFrontiers(rospy.wait_for_message('explore/frontiers', MarkerArray))
    updateGoal(0)

def add_data(time, robot, frontiers, execution, blob_frontier):
    sql = 'INSERT INTO Exploration VALUES(?,?,?,?,?)'
    conn.cursor().execute(sql, (time, robot, frontiers, execution, blob_frontier))
    conn.commit()

def add_final_map(execution, robot, map, time):
    sql = 'INSERT INTO FinalMaps VALUES(?,?,?,?)'
    conn.cursor().execute(sql, (execution, robot, map, time))
    conn.commit()

if __name__ == '__main__':
    robot_id = int(re.findall("[0-9]+", rospy.get_namespace())[0])-1
    rospy.init_node('move_controller', disable_signals=True)
    
    db_conn = None
    package_dir = rospkg.RosPack().get_path('test_unknown_rendezvous')
    try:
        conn = sqlite3.connect(package_dir+'/data/data_test.db', check_same_thread=False)
        print(f'Cluster Controller: creata connessione db')
    except Error as e:
        print(e)

    while not rosnode.rosnode_ping(f'/pos_aggregator', max_count=1): #aspetto pos_aggregator
        time.sleep(.5)

    cur = conn.cursor()
    execution_nr = None
    while execution_nr is None:
        cur.execute('SELECT MAX(id) FROM Executions')
        execution_nr = cur.fetchone()[0]

    EXPLORE_LITE = rospy.get_param('/explore-lite')

    colors = [(0,255,0),(0,255,255),(255,20,147),(255,255,0)]
    colors = [getColorRGB(c) for c in colors]

    frontiers = [] #[(centroide1,costo1,frontiera1), ..., (centroide_n,costo_n,frontiera_n)]
    other_frontiers = [] #frontiere ereditate dal cluster
    blob_frontiers = [] #uguale a frontiers
    #banned_frontiers = [] #lista dei centroidi delle frontiere tolte (tuple)
    banned_frontiers = {} #dizionario contenente i centroidi
    min_frontier_size = 1.5

    visited_area = Polygon() #area visitata (blob)

    old_max_frontier_centroid = Point()  #ultima frontiera scelta come goal (centroide)
    old_max_frontier = LineString() #ultima frontiera scelta come goal (figura)
    prev_distance = 0 #distanza del robot dall'old_max_frontier
    max_time = 15 # tempo massimo in secondi in cui il robot cerca di raggiungere la frontiera
    last_progress = rospy.Time.now()
    last_try = False #se le frontiere sono vuote, prima di bloccare provo a farlo girare

    my_pose = Odometry()
    last_my_pose = Odometry()
    last_my_pose_time = 0
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    transform = tf2_ros.TransformStamped()
    rospy.Timer(rospy.Duration(0.33), updateTransform) #aggiorna la trasform tf

    max_time_stationary = 10
    time_to_be_stucked = 60
    BLOCCATO = False
    incastrato = False

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

    topic_logger = rospy.Publisher('/logger', logging, queue_size=10)
    start_time = 0
    while not start_time:
        start_time = rospy.get_time()

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
    time.sleep(2)
    updateGoal(0)

    rospy.spin()