import sys
import numpy
sys.path.append('../build/bin/')
from libksrobot import *

p = common.KinectPointCloud()

pcl.TransformPointCloud(p, p, eigen.Isometry3f.Identity)