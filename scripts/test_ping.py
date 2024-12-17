import rospy, rosnode

if __name__ == '__main__':
    rospy.init_node('test')

    print(rospy.get_param('world'))
    print(type(rospy.get_param('world')))

    print(rosnode.rosnode_ping('rosout', max_count=1))

    rospy.spin()