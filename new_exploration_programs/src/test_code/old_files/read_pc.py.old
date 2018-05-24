#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

def callback_pointcloud(data):
    assert isinstance(data, PointCloud2)
    gen = point_cloud2.read_points(data)
    print type(gen)
    for p in gen:
      print p

def listener():

    # in ROS, nodes are unique named. If two nodes with the same
    # node are launched, the previous one is kicked off. The 
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaenously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/camera/depth_registered/points", PointCloud2, callback_pointcloud)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spinOnce()


if __name__ == '__main__':
    listener()
