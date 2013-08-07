import sys
sys.path.append('../build/bin/')
import os
from libksrobot import *



if __name__ == "__main__":
    xmlConfigFile = 'build_pcd_file_settings.xml'
    datasetDir = ''
    #defaultDatasetDir = '/windows/E/Datasets/rgbd_dataset_freiburg2_pioneer_slam/'
    defaultDatasetDir = '/home/kourosh/test/pointclouds/corridors1_ground_palne_extraction_test/'
    defaultSaveDir = '/home/kourosh/test/pointclouds/corridors1_ground_palne_extraction_test/'
    
    po = common.ProgramOptions()
    
    if os.path.exists(xmlConfigFile):
        po.LoadFromFile(xmlConfigFile)
    #datasetDir = po.GetString('DatasetDir', defaultDatasetDir)
    #saveDir = po.GetString('SaveDir', defaultSaveDir)
    datasetDir = defaultDatasetDir
    saveDir = defaultSaveDir
    
    if not os.path.exists(datasetDir):
        print 'Path <', datasetDir, '> does not exist.'
        sys.exit(-1)
        
    saveDir = os.path.abspath(os.path.expanduser(saveDir))
    if not os.path.exists(saveDir):
        os.makedirs(saveDir)
    
    print 'Using dataset at ', os.path.abspath(datasetDir)
    print 'Save directory is: ', saveDir
    
    kinect = interfaces.KinectDatasetReader()
    fovis = interfaces.FovisInterface()
    
    fovis.ReadSettings(po.StartNode('Fovis'))
    kinect.Initialize(datasetDir)
    fovis.RegisterToKinect(kinect)
    
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
        pc = kinect.GetPointCloud()
        #pcl.removeNaNFromPointCloud(pc, pc)
        
        filteredCloud = common.KinectPointCloud()
        if voxelEnabled:
            voxFilter = pcl.VoxelGrid()
            voxFilter.setLeafSize(voxResolution, voxResolution, voxResolution)
            voxFilter.setInputCloud(pc)
            voxFilter.filter(filteredCloud)
        else:
            filteredCloud = pc
            
        if passEnabled:
            passFilter.setInputCloud(filteredCloud)
            passFilter.filter(filteredCloud)
        
        pcl.transformPointCloud(filteredCloud, filteredCloud, fovis.GetGlobalPose())
        pcl.MergePointClouds(final_pc, filteredCloud)
        #saveFile = os.path.join(saveDir, str(i) + '.pcd')
        #pcl.io.savePCDFileBinaryCompressed(saveFile, filteredCloud)
        
    #po.SaveToFile(xmlConfigFile)

    final_file_name = os.path.join(saveDir, 'final_pointcloud.pcd')
    pcl.io.savePCDFileBinaryCompressed(final_file_name, final_pc)
    