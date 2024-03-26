#!/usr/bin/env python3
import rospy, sys, re, time, matplotlib.pyplot as plt, rosnode, rospkg, sqlite3, pickle, zstd, copy
from sqlite3 import Error
from shapely.geometry import Point, MultiPoint, LineString
from shapely.ops import unary_union
from math import sqrt, pi
from test_unknown_rendezvous.msg import array_pos, cluster, logging
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist

class Cluster:
    def __init__(self,n):
        self.clusters = [[i] for i in range(n)] #ogni robot appartiene a un cluster di 1 elemento
        self.n = n #numero di robot
    
    def union(self,a,b, log=True): #aggiunge b nel cluster di a, elimina il cluster di b
        i1 = self.find(a)
        i2 = self.find(b)
        if i1!=i2:
            self.clusters[i1] += self.cluster(b)
            self.clusters.pop(i2)
        self.clusters[i1].sort()
        if log:
            print(f'union({a},{b}) {clusters}')
    
    def find(self,x): #indica il cluster (tramite indice) a cui appartiene x, -1 se non c'è
        for i in range(len(self.clusters)):
            if x in self.clusters[i]:
                return i
        return -1
    
    def separate(self,a,log=True): #scorpora dal suo cluster a, lo mette in un cluster vuoto
        i1 = self.find(a)
        x = self.clusters[i1]
        x.pop(x.index(a))
        self.clusters.append([a])
        if log:
            print(f'separate({a}) {clusters}')

    def same(self,a,b): #indica se a e b sono nello stesso cluster
        return self.find(a)==self.find(b)

    def cluster(self,a): #restituisce il cluster a cui appartiene a
        return self.clusters[self.find(a)]

    def max_cluster(self):
        return max([len(c) for c in self.clusters])

    def all_together(self):
        return len(self.clusters)==1

    def __iter__(self):
        return self.clusters.__iter__()

    def __repr__(self):
        return repr(self.clusters)

def check_distances(all_positions):
    global first_add
    positions = all_positions.positions
    n = len(positions)
    for i in range(n): # verfico distanze robot
        for j in range(n):
            if i<j and clusters.cluster(i)[0]==i and clusters.cluster(j)[0]==j: # evito calcoli ridondanti
                pos1 = positions[i].position.pose.pose.position
                pos2 = positions[j].position.pose.pose.position
                dist = calc_dist(pos1, pos2)
                #print(f'x1: {pos1.x}, y1:{pos1.y} | x2: {pos2.x}, y2:{pos2.y}')
                same_cluster = clusters.same(i,j)
                if dist<sqrt((threshold**2*(2*len(clusters.cluster(i))-2+pi))/pi):
                    #print(f'[ClusterController] {i}-{j} vicini')
                    if same_cluster:
                        continue
                    obstacle = check_obstacle_map(i,(pos1.x, pos1.y),(pos2.x, pos2.y))
                    if not obstacle and not same_cluster: #se sono già nello stesso cluster evito
                        clusters.union(i,j)
                        log_str = logging()
                        log_str.str = f"[{round(rospy.get_time()-start_time,2)}] \t\trobot {i} ha trovato robot {j} -> {clusters}"
                        logger_pub.publish(log_str)
                        add_data(
                            rospy.get_time(),
                            i+1,
                            clusters.max_cluster(),
                            zstd.compress(pickle.dumps(rawmap[clusters.cluster(i)[0]])),
                            pickle.dumps([(pos1.x, pos1.y),(pos2.x, pos2.y)]),
                            0,
                            execution_nr
                        )
                        print(f"Cluster Controller: salvata mappa robot{i+1}")
                        update_cluster()
                    if obstacle and first_add:
                        add_data(
                            rospy.get_time(),
                            i+1,
                            clusters.max_cluster(),
                            zstd.compress(pickle.dumps(rawmap[clusters.cluster(i)[0]])),
                            pickle.dumps([(pos1.x, pos1.y),(pos2.x, pos2.y)]),
                            1,
                            execution_nr
                        )
                        print(f"Cluster Controller: salvata mappa robot{i+1}")
                        first_add = False
                elif dist>threshold+1 and False:
                    if same_cluster: #separazione
                        c = clusters.cluster(i)
                        if len(c)==2:
                            clusters.separate(j)
                            update_cluster()
                        else:
                            is_i = is_j = True
                            for x in c: #devo capire tra i e j chi si è staccato
                                if x!=i and x!=j: #confronto distanza con gli altri robot nel cluster
                                    if calc_dist(pos1, positions[x].position.pose.pose.position)<threshold:
                                        is_i = False # i vicino al cluster
                                    if calc_dist(pos2, positions[x].position.pose.pose.position)<threshold:
                                        is_j = False # j vicino al cluster
                                    if not (is_i or is_j):
                                        break
                            if is_i or is_j:
                                if is_i:
                                    clusters.separate(i)
                                else:
                                    clusters.separate(j)
                                update_cluster()

def update_cluster():
    cluster_update = cluster()
    for x in clusters:
        first = True
        for y in x:
            cluster_update.leader = first #il leader è sempre il primo del cluster
            first = False
            cluster_update.robots = x
            cluster_update.all_together = clusters.all_together()
            cluster_pubs[y].publish(cluster_update)

def check_obstacle_map(id_first, p1, p2): #return true se tra p1 e p2 c'è un ostacolo
    global clusters
    if obstacles_maps[id_first]==0: #mappa non ancora aggiornata
        #print(f"{[id_first]} mappa non aggiornata")
        return True
    #rimuovo i robot dalla mappa, altrimenti fungono da ostacolo
    map = obstacles_maps[clusters.cluster(id_first)[0]].difference(Point(p2).buffer(.4)).difference(Point(p1).buffer(.4))
    value = LineString([p1,p2]).intersects(map)
    return value

def calc_dist(p1, p2):
    return sqrt((p1.x-p2.x)**2 + (p1.y-p2.y)**2)

def getPoseStamped(covPos): #preso un Odometry restistuisce il relativo PoseStamped
    new = PoseStamped()
    new.header = covPos.header
    new.pose = covPos.pose.pose
    return new

def update_map(new_map):
    global rawmap, changed_raw
    id = int(re.findall("[0-9]+", new_map.header.frame_id)[0])-1
    obstacle = []
    occupancy = new_map.data
    resolution = new_map.info.resolution
    origin_x = new_map.info.origin.position.x
    origin_y = new_map.info.origin.position.y
    for i in range(new_map.info.width):
        for j in range(new_map.info.height):
            if occupancy[i+j * new_map.info.width] != -1 and occupancy[i+j * new_map.info.width] > 70:
                x = (i * resolution) + origin_x
                y = (j * resolution) + origin_y
                p = Point(x, y)
                obstacle.append(p)
    obstacle = MultiPoint(obstacle).buffer(.15)
    obstacles_maps[id] = unary_union(obstacle)
    rawmap[id] = new_map

def print_map(map, p1,p2): #mostra il grafico degli ostacoli
    for x in map.geoms:
        plt.plot(*x.exterior.xy,'-')
    x,y = p1
    plt.plot(x,y,"ok")
    x,y = p2
    plt.plot(x,y,"ok")
    line = LineString([p1,p2])
    plt.plot(*line.coords.xy, '-r')
    plt.show()

def add_data(time, robot, max, map, points, obstacle, execution): #map deve essere in bytes
    sql = 'INSERT INTO Clustering VALUES(?,?,?,?,?,?,?)'
    conn.cursor().execute(sql, (time, robot, max, map, points, obstacle, execution))
    conn.commit()

def logger(s):
    LOG_FILE.write(s.str+'\n')
    LOG_FILE.flush()

if __name__ == '__main__':
    threshold = float(sys.argv[1]) #se distanza<threshold si forma cluster
    robot_number = int(sys.argv[2]) #numero di robot presenti
    #map_control = False
    rospy.init_node('cluster')

    db_conn = None
    package_dir = rospkg.RosPack().get_path('test_unknown_rendezvous')
    try:
        conn = sqlite3.connect(package_dir+'/data/data_test.db', check_same_thread=False)
        print(f'Cluster Controller: creata connessione db')
    except Error as e:
        print(e)

    while not rosnode.rosnode_ping(f'/pos_aggregator', max_count=1): #aspetto pos_aggregator
        break
    time.sleep(.5)

    cur = conn.cursor()
    cur.execute('SELECT MAX(id) FROM Executions')
    execution_nr = cur.fetchone()[0]

    #LOGGING
    log_path = f'{package_dir}/log'
    EXPLORE_LITE = rospy.get_param('/explore-lite')
    world_name = rospy.get_param('/world')

    LOG_FILE = open(
        f'{log_path}/{execution_nr}_{world_name}_{robot_number}_{"el" if EXPLORE_LITE else "my"}.txt',
        'a'
    )
    l = logging()
    l.str = f'ESECUZIONE {execution_nr} SULLA MAPPA {world_name} CON {robot_number} E METODO {"el" if EXPLORE_LITE else "my"}\n' 
    logger(l)

    rospy.Subscriber('/logger', logging, logger)
    logger_pub = rospy.Publisher('/logger', logging, queue_size=10)

    #rospy.wait_for_service('/gazebo/spawn_urdf_model')
    time.sleep(8)
    rospy.Subscriber('/all_positions', array_pos, check_distances)
    clusters = Cluster(robot_number)
    cluster_pubs = []
    obstacles_maps = [0]*robot_number #figura rappresentante gli ostacoli

    rawmap = [0]*robot_number
    first_add = True

    for i in range(robot_number):
        cluster_pubs += [rospy.Publisher(f'/robot{i+1}/cluster', cluster, queue_size=10)]
        rospy.Subscriber(f'/robot{i+1}/map', OccupancyGrid, update_map)
        add_data(
            rospy.get_time(),
            i,
            1,
            zstd.compress(pickle.dumps([])),
            pickle.dumps([]),
            0,
            execution_nr
        )
    update_cluster()

    start_time = 0
    while not start_time:
        start_time = rospy.get_time()

    rospy.spin()
    LOG_FILE.close()