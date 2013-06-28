import sys
import numpy
sys.path.append('../build/bin/')
from libksrobot import *

m = eigen.Matrix3f()
q = eigen.Quaternionf(m)

print q
