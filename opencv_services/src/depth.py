#! /usr/bin/env python

from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import rospy
import numpy as np
import time
import ros_numpy

def callback_pointcloud(data):
    assert isinstance(data, PointCloud2)
    # gen = point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
    # time.sleep(1)
    x_total=0
    y_total=0
    z_total=0
    x=[]
    y=[]
    z=[]
    number_of_sampling=0
    # cloud_points = ros_numpy.point_cloud2.get_xyz_points(ros_numpy.point_cloud2.pointcloud2_to_array(data), remove_nans=True, dtype=np.float32)
    cloud_points = list(point_cloud2.read_points(data, skip_nans=True, field_names = ("x", "y", "z")))
    # print(cloud_points)
    for i in cloud_points:
        number_of_sampling+=1
        x.append(i[2])
        y.append(i[0])
        z.append(i[1])
        # x_total+=i[0]
        # y_total+=i[1]
        # z_total+=i[2]

    print('minx',min(x))
    print('miny',min(y))
    print('minz',min(z))
    print('maxx',max(x))
    print('maxy',max(y))
    print('maxz',max(z))
    # x_average = x_total / number_of_sampling
    # y_average = 1*y_total / number_of_sampling
    # z_average = 1*z_total / number_of_sampling
    # print(x_average)
    # print(y_average)
    # print(z_average)

    # print(sum(x)/len(x),sum(y)/len(y),sum(z)/len(z))
def main():
    rospy.init_node('pcl_listener', anonymous=True)
    rospy.Subscriber('/camera/depth/points', PointCloud2, callback_pointcloud)
    rospy.spin()

if __name__ == "__main__":
    main()
