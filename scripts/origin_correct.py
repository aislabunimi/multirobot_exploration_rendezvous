#!/usr/bin/env python3
import rospy, time
from nav_msgs.msg import OccupancyGrid

def readMapMerge(mm):
    global mergedMap
    mergedMap = mm
    time.sleep(0.1)

# legge la map prodotta dallo SLAM di robot1, ne legge l'origine e la imposta per la correzione
def updateMapMerge(correctMap):
    correctPos = correctMap.info.origin
    mergedMap.info.origin = correctPos
    correctMapPub.publish(mergedMap)
    time.sleep(0.07)
    
if __name__ == '__main__':
    rospy.init_node("corrected_map")
    mergedMap = OccupancyGrid()
    mergedMapSub = rospy.Subscriber("/map", OccupancyGrid, readMapMerge)
    robot1MapSub = rospy.Subscriber("/robot1/map", OccupancyGrid, updateMapMerge)
    correctMapPub = rospy.Publisher("/correct_map", OccupancyGrid, queue_size=10)
    rospy.spin()
