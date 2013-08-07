import sys
sys.path.append('../build/bin/')
import os
from libksrobot import *
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

if __name__ == "__main__":
    #octomap = interfaces.OctomapInterface()
    #octomap.LoadFromFile('/home/kourosh/test1_map.ot')
    #ocgrid = common.OccupancyMap(1000, 1000)
    
    #octomap.ConvertToOccupancyGrid(ocgrid, 200, 700, False)

    #ocgrid.SaveToFile('/home/kourosh/ocgrid.ocg')

    #grid = np.zeros((ocgrid.Width, ocgrid.Height), dtype=np.uint8)
    #frontier = grid.copy()
    
    #for i in xrange(0, ocgrid.Height):
        #for j in xrange(0, ocgrid.Width):
            ##idx = ocgrid.Index(i, j)
            #grid[i][j] = ocgrid.Data[ocgrid.Index(i, j)]

    #fa = common.OccupancyMap.FrontierArray()
    #ocgrid.ExtractFrontiers(fa, 0.40)
    
    #for i in xrange(0, ocgrid.Height):
        #for j in xrange(0, ocgrid.Width):
            ##idx = ocgrid.Index(i, j)
            #frontier[i][j] = ocgrid.Data[ocgrid.Index(i, j)]
            
    ##cp1 = (frontier > common.OccupancyMap.CellValue.FrontierStartID)

    ##plt.figure()
    ##plt.imshow(grid)
    
    #print 'Number of frontiers:', len(fa)
    
    #plt.figure()
    #plt.imshow(frontier)
    #plt.colorbar()
    
    
    #plt.show()
    
    
    ##raw_input('Press enter to continue...')
#else:
    xmlConfigFile = 'build_pcd_file_settings.xml'
    datasetDir = ''
    #defaultDatasetDir = '/windows/E/Datasets/rgbd_dataset_freiburg2_pioneer_slam/'
    defaultDatasetDir = '/home/kourosh/test/pointclouds/corridors1_ground_palne_extraction_test/'
    defaultSaveDir = '/home/kourosh/test/pointclouds/corridors1_ground_palne_extraction_test/'
    
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
    octomap = interfaces.OctomapInterface()
    robot_info = common.RobotInfo()
    
    robot_info.ReadSettings(po.StartNode('RobotInfo'))
    
    fovis.ReadSettings(po.StartNode('Fovis'))
    octomap.ReadSettings(po.StartNode('Octomap'))
    
    kinect.Initialize(datasetDir)
    fovis.RegisterToKinect(kinect)
    octomap.RegisterToVO(fovis)
    fovis.SetRobotInfo(robot_info)
    octomap.SetRobotInfo(robot_info)
       
    voxelEnabled = po.GetBool('VoxelGrid.Enabled', True)
    voxResolution = po.GetDouble('VoxelGrid.Resolution', 0.02)
    passFilter = pcl.PassThrough()
    passFilter.setFilterFieldName(po.GetString('PassThrough.FieldName', 'z'))
    passFilter.setFilterLimits(po.GetDouble('PassThrough.LowLimit', 0), po.GetDouble('PassThrough.HighLimit', 4))
    passEnabled = po.GetBool('PassThrough.Enabled', True)
    
    maxCycles = po.GetInt('MaxCycles', 10)
    
    final_pc = common.KinectPointCloud()
    
    for i in xrange(0, maxCycles):
        print 'Cycle ', i
        if kinect.RunSingleCycle() == False:
            print 'Kinect dataset finished.'
            break
        fovis.RunSingleCycle()
        #octomap.RunSingleCycle()

        #ground = octomap.GetGroundPoints()
        #nonground = octomap.GetNonGroundPoints()
        
        #ground_file = os.path.join(saveDir, 'ground_' + str(i) + ".pcd")
        #nonground_file = os.path.join(saveDir, 'nonground_' + str(i) + ".pcd")
        #pcl.io.savePCDFileBinaryCompressed(ground_file, ground);
        #pcl.io.savePCDFileBinaryCompressed(nonground_file, nonground);

        #tmp_pc = common.KinectPointCloud()
        #pcl.MergePointClouds(tmp_pc, nonground)
        #pcl.transformPointCloud(tmp_pc, tmp_pc, fovis.GetGlobalPose())
        #pcl.MergePointClouds(final_pc, tmp_pc)


    po.SaveToFile(xmlConfigFile)   
    #octomap.SaveToFile('/home/kourosh/test1_map.ot')
    #pcl.io.savePCDFileBinaryCompressed('/home/kourosh/final_pc.pcd', final_pc)
    
    print 'Hello!'