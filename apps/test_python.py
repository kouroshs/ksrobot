import sys
sys.path.append('../build/bin/')
from libksrobot import *

base_path = '/home/kourosh/test/pointclouds/'


p1 = common.KinectPointCloud()
p2 = common.KinectPointCloud()

pcl.io.loadPCDFile(base_path + '1.pcd', p1)
pcl.io.loadPCDFile(base_path + '2.pcd', p2)

p1.sensor_origin_ = eigen.Vector4f(0, 1, 2, 0)

print p1.sensor_origin_ * 2