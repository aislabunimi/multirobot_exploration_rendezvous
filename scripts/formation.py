#!/usr/bin/env python3
import rospy, sys, numpy as np, re, time, rospkg, sqlite3, rosnode, math
from sqlite3 import Error
from test_unknown_rendezvous.msg import cluster, array_pos
from geometry_msgs.msg import Twist, Vector3, PoseStamped
from itertools import chain
from sensor_msgs.msg import LaserScan
from math import pi, sqrt, sin, cos
from numpy.linalg import norm
from actionlib_msgs.msg import GoalID

def get_yaw(current_orient):
    global yaw
    global yaw_old
    global yaw_old_old  
    x=current_orient.x
    y=current_orient.y
    z=current_orient.z
    w=current_orient.w
    siny_cosp = 2*(w*z + x*y)
    cosy_cosp = 1-2*(y*y + z*z)
    yaw_new = np.arctan2(siny_cosp, cosy_cosp)
    yaw_array = np.array([yaw_old_old, yaw_old, yaw_new])
    yaw_array_new = np.unwrap(yaw_array)
    yaw, yaw_old, yaw_old_old = yaw_array_new[2], yaw, yaw_old

def compute_action(cluster):
    global goal_status, distance, padre
    if goal_status:
        del_goal.publish(GoalID()) #elimina qualsiasi goal
        goal_status = False
    n = len(cluster)
    #raggio = 0.6
    raggio = 1
    distance_matrix = calc_dist_matrix(n, 0.5)
    my_pose = all_pos[robot_id].position.pose.pose.position
    my_orient = all_pos[robot_id].position.pose.pose.orientation
    u = np.zeros(3)
    x_i = np.array([my_pose.x, my_pose.y, my_pose.z])
    i = robot_id
    if FINISH: #shape formation
        x_to_pos = lambda x: cluster.index(x)
        for j in cluster:
            if (j!=i):
                other_pose = all_pos[j].position.pose.pose.position
                x_j = np.array([other_pose.x, other_pose.y, other_pose.z])
                u += (norm(x_i-x_j)**2-distance_matrix[x_to_pos(i),x_to_pos(j)]**2)*(x_j-x_i)
    else: #chain formation
        if padre==-1:
            padre = cluster[-2]
            print(f'[{robot_id}] formation: il padre è {padre}')
        other_pose = all_pos[padre].position.pose.pose.position
        x_j = np.array([other_pose.x, other_pose.y, other_pose.z])
        u = (norm(x_i-x_j)**2-raggio**2)*(x_j-x_i)

    u *= gain

    add_data(
        rospy.get_time(),
        robot_id+1, 
        u[0],
        u[1],
        execution_nr
    )

    action = Vector3()
    action.x = u[0]
    action.y = u[1]
    action.z = u[2]
    u_=np.array([action.x, action.y, action.z])
    get_yaw(my_orient)
    a = np.cos(yaw)
    b = np.sin(yaw)
    U = np.zeros(2)
    U[0] = 0.5*(a*u_[0]+b*u_[1])
    U[1] = (np.pi/8.0)*np.arctan2(-b*u_[0]+a*u_[1], u_[0])/(np.pi/2)
    if U[0]<0 and abs(U[0])>0.05: #non può andare all'indietro tranne per piccoli movimenti
        #print(f'[{robot_id}] {U}')
        send_input([0, -2.5]) #girati a destra
        return
    if U[0]>0.6: #limito la velocità
        U[0] = 0.6
    for k in range(45,135,5): #evito collisioni
        if distance[k] <= 0.15: #presenza ostacolo
            print(f'[{robot_id}] ostacolo a {k}° gradi distante {distance[k]}')
            send_input([0,0])    # fermati
            send_input([-0.2,0]) # indietreggia
            time.sleep(0.3)
            send_input([0,-2]) # girati
            time.sleep(1)
            send_input([0.2,0]) # avanza
            time.sleep(1.5)
            return
    send_input(U)

def avoid_obstacle():
    move = Twist()
    for i in range(360):
        if(distance[i] <= 0.3):
            move.linear.x = -0.2
            if distance[179] > distance[359]:
                move.angular.z = 0.9
            else:
               move.angular.z = -0.9
        else:
            move.linear.x = 0.1
            move.angular.z = 0.0
    pub.publish(move)

def send_input(u):
    #print(f'[{robot_id}]: {u}')
    msg=Twist()
    msg.linear.x = u[0]
    msg.angular.z = u[1]
    pub.publish(msg)

def update_laser_distance(msg):
    global distance
    if len(msg.ranges)==0:
        return
    distance = msg.ranges

def update_cluster(update):
    global stop_cmd_vel
    global my_cluster, leader, FINISH
    my_cluster = update.robots
    leader = update.leader
    FINISH = update.all_together
    #print(f'[{robot_id}] {my_cluster}')
    if leader and not stop_cmd_vel:
        stop_robot()

def update_all_pos(array_pos):
    global all_pos, stop_cmd_vel
    all_pos = array_pos.positions
    obstacle = False
    if FINISH:
        compute_action(my_cluster)
    if leader:
        return
    for k in range(360): #evito collisioni
        if distance[k] <= 0.2: #presenza ostacolo
            print(f'[{robot_id}] avoid obstacle {k}°')
            obstacle = True
    if False and obstacle: 
        avoid_obstacle()
        return
    if stop_cmd_vel:
        stop_cmd_vel = False
    if len(distance)!=0:
        compute_action(my_cluster)

def regularPolyDistance(N,L):
    '''
    Restituisce la matrice delle distanze di un poligono
    regolare con N lati lunghi L
    '''
    raggio = L/(sqrt(2-2*cos(2*pi/N)))
    M = np.zeros([N,N])
    for i in range(N):
        for j in range(N):
            if i<j:
                M[i,j] = sqrt(2*(1-cos((i-j)*2*pi/N)))
            elif i>j:
                M[i,j] = M[j,i]
    M = M*raggio
    return M

def stop_robot():
    global stop_cmd_vel
    send_input([0,0])
    stop_cmd_vel = True
    print(f'{robot_id} stoppato movimento')

def check_goal(_): #segnala la presenza di un goal
    global goal_status
    goal_status = True

def calc_dist_matrix(n,r): #calcola la matrice delle distanze di n robot circoscritti in un circ. di raggio r
    global distance_matrix
    if n==2:
        distance_matrix = np.array([[0,r],[r,0]])
    else:
        if n!=len(distance_matrix):
            distance_matrix = regularPolyDistance(n,r)
    return distance_matrix

def add_data(time, robot, ux, uy, execution):
    sql = 'INSERT INTO Formation VALUES(?,?,?,?,?)'
    conn.cursor().execute(sql, (time, robot, ux, uy, execution))
    conn.commit()

if __name__ == '__main__':
    gain = float(sys.argv[1])
    rospy.init_node('formation')

    robot_id = int(re.findall("[0-9]+", rospy.get_namespace())[0])-1
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

    rospy.wait_for_service('/gazebo/spawn_urdf_model') #Mi assicuro lo spawn del robot
    time.sleep(2)

    my_cluster = [robot_id]
    leader = True
    FINISH = False #RENDEVZOUS!

    padre = -1 #id del robot che seguirà

    yaw = 0.0
    yaw_old = 0.0
    yaw_old_old = 0.0
    distance = [10]*360
    all_pos = []
    distance_matrix = []
    goal_status = False #True il robot deve perseguire un goal
    stop_cmd_vel = False #Variabile usata per stoppare i robot che escono dal cluster
    rospy.Subscriber('cluster', cluster, update_cluster)
    rospy.Subscriber('scan', LaserScan, update_laser_distance)
    rospy.Subscriber('move_base/current_goal', PoseStamped, check_goal)
    rospy.Subscriber('/all_positions', array_pos, update_all_pos)
    del_goal = rospy.Publisher('move_base/cancel', GoalID, queue_size=10)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)

    while not rosnode.rosnode_ping(f'/pos_aggregator', max_count=1): #aspetto pos_aggregator
        break

    rospy.spin()
