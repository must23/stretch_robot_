#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
import csv

def path_callback(data):
    # Open the file in append mode
    with open('path.csv', mode='w') as file:
        writer = csv.writer(file, delimiter=',')
        # Write header
        writer.writerow(['x', 'y', 'z'])
        # Write each pose in the path
        for pose in data.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            z = pose.pose.position.z
            writer.writerow([x, y, z])
    rospy.loginfo("Path saved to path.csv")

def listener():
    rospy.init_node('path_saver', anonymous=True)
    rospy.Subscriber('/move_base/NavfnROS/plan', Path, path_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
