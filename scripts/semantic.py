#!/usr/bin/env python3
import rospy, sys, coords_parser, time
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

#positions = [(x1,y1),(x2,y2)...]
def getMarker(id, positions, remove=False):
    M = Marker()
    M.ns = "semantica_area"
    M.header.frame_id = "map"
    M.header.stamp = rospy.Time.now()
    M.type = Marker.LINE_STRIP
    M.action = Marker.ADD if not remove else Marker.DELETE
    M.id = id
    for pos in positions:
        P = Point()
        x,y = pos.x, pos.y
        P.x = x
        P.y = y
        P.z = 0
        M.points += [P]
    close_point = Point()
    first = positions.points[0]
    x,y = first.x, first.y
    close_point.x = x
    close_point.y = y
    close_point.z = 0
    M.points += [close_point]
    M.pose.orientation.w = 1
    M.scale.x = 0.1
    M.color = color
    return M

def getColorRGB(rgb):
    r,g,b = rgb
    C = ColorRGBA()
    C.r = r/255
    C.g = g/255
    C.b = b/25
    C.a = 1
    return C

def pub_sem_area(_):
    semantic_area_pub.publish(marker)

if __name__ == '__main__':
    rospy.init_node('semantic_areas')

    file_path = sys.argv[1]
    color = getColorRGB((197,255,0))
    areas = coords_parser.parse_from_file(file_path)
    marker = MarkerArray()
    marker.markers = [getMarker(id, ext) for id, ext in enumerate(areas)]
    semantic_area_pub = rospy.Publisher('/semantic_areas', MarkerArray, queue_size=5)

    for _ in range(10):
        pub_sem_area(_)
        time.sleep(1)

    rospy.Timer(rospy.Duration(5), pub_sem_area)

    rospy.spin()