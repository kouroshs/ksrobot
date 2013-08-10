import sys
sys.path.append('../build/bin/')
import os
from libksrobot import *
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

if __name__ == "__main__":
    xmlConfigFile = 'build_pcd_file_settings.xml'
    datasetDir = ''
    #defaultDatasetDir = '/windows/E/Datasets/rgbd_dataset_freiburg2_pioneer_slam/'
    #defaultDatasetDir = '/home/kourosh/test/pointclouds/corridors1_ground_palne_extraction_test/'
    #defaultSaveDir = '/home/kourosh/test/pointclouds/corridors1_ground_palne_extraction_test/'
    
    defaultDatasetDir = '/home/kourosh/datasets/yaw-pitch-roll-test'
    defaultSaveDir = '/home/kourosh/pointclouds/yaw-pitch-roll-test'
    
    po = common.ProgramOptions()
    
    if os.path.exists(xmlConfigFile):
        po.LoadFromFile(xmlConfigFile)
    
    datasetDir = defaultDatasetDir
    saveDir = defaultSaveDir
    
    if not os.path.exists(datasetDir):
        print 'Path <', datasetDir, '> does not exist.'
        sys.exit(-1)
        
    saveDir = os.path.abspath(os.path.expanduser(saveDir))
    if not os.path.exists(saveDir):
        os.makedirs(saveDir)
    
    kinect = interfaces.KinectDatasetReader()
    fovis = interfaces.FovisInterface()
    robot_info = common.RobotInfo()
    
    isam = interfaces.iSAM2Interface()
    
    robot_info.ReadSettings(po.StartNode('RobotInfo'))
    
    fovis.ReadSettings(po.StartNode('Fovis'))
    isam.ReadSettings(po.StartNode('iSAM2'))
    
    kinect.Initialize(datasetDir)
    fovis.RegisterToKinect(kinect)
    fovis.SetRobotInfo(robot_info)
    
    isam.RegisterToVO(fovis)
    
    voxelEnabled = po.GetBool('VoxelGrid.Enabled', True)
    voxResolution = po.GetDouble('VoxelGrid.Resolution', 0.02)
    passFilter = pcl.PassThrough()
    passFilter.setFilterFieldName(po.GetString('PassThrough.FieldName', 'z'))
    passFilter.setFilterLimits(po.GetDouble('PassThrough.LowLimit', 0), po.GetDouble('PassThrough.HighLimit', 4))
    passEnabled = po.GetBool('PassThrough.Enabled', True)
    
    maxCycles = po.GetInt('MaxCycles', 10)
    
    final_pc = common.KinectPointCloud()
    
    for i in range(0, maxCycles):
        print 'Cycle ', i
        if kinect.RunSingleCycle() == False:
            print 'Kinect dataset finished.'
            break
        fovis.RunSingleCycle()
        print 'before isam'
        isam.RunSingleCycle()
        

    po.SaveToFile(xmlConfigFile)
    print 'Execution finished.'

