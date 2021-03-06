import numpy as np
import cv2

from roversupport import *
import ctelemetryreader

def process_image(POVRaw, telReader):
    procImage = {}
    procImage['POVRaw'] = POVRaw
    threshRGB_Nav =      [1, (160, 160, 160)]
    threshRGB_Obstacle = [0, (130, 130, 130)]
    threshHSV_Nug =      [1, (20, 150, 100), (50, 255, 255)]
  
    procImage['POVThreshNav'] = Thresh_ClipRGB(procImage['POVRaw'] , threshRGB_Nav) 
    procImage['POVThreshObs'] = Thresh_ClipRGB(procImage['POVRaw'] , threshRGB_Obstacle) 
    procImage['POVThreshNug'] = Thresh_ClipHSV(procImage['POVRaw'] , threshHSV_Nug) 
    
    # 1) Define source and destination points for perspective transform
    matViewToMap = GetViewToMapTransform()
    
    # 2) Apply perspective transform to POVRaw to get Map view of the raw
    procImage['Map_POVRaw']  = perspect_transform(POVRaw, matViewToMap)

    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    procImage['Map_POVThreshNav'] = Thresh_ClipRGB(procImage['Map_POVRaw'] , threshRGB_Nav) 
    procImage['Map_POVThreshObs'] = Thresh_ClipRGB(procImage['Map_POVRaw'] , threshRGB_Obstacle) 
    procImage['Map_POVThreshNug'] = Thresh_ClipHSV(procImage['Map_POVRaw'] , threshHSV_Nug) 

    # 4) Convert thresholded image pixel values to rover-centric coords
    navigable_xpix, navigable_ypix = rover_coords(procImage['Map_POVThreshNav'])
    obstacles_xpix, obstacles_ypix = rover_coords(procImage['Map_POVThreshObs'])
    rocks_xpix, rocks_ypix = rover_coords(procImage['Map_POVThreshNug'])
    
    # 5) Convert rover-centric pixel values to world coords
    scale = 10
    try:
        # This will fail for the last frame. idk.
        xpos, ypos = telReader.xpos[telReader.count], telReader.ypos[telReader.count]
        yaw = telReader.yaw[telReader.count]
    except IndexError:
        xpos, ypos = telReader.xpos[telReader.count-1], telReader.ypos[telReader.count-1]
        yaw = telReader.yaw[telReader.count-1]
        
    worldmap_size = telReader.worldmap.shape[0]
    navigable_x_world, navigable_y_world = pix_to_world(navigable_xpix, navigable_ypix,xpos, ypos, yaw, worldmap_size, scale)
    obstacles_x_world, obstacles_y_world = pix_to_world(obstacles_xpix, obstacles_ypix,xpos, ypos, yaw, worldmap_size, scale)
    rocks_x_world, rocks_y_world = pix_to_world(rocks_xpix, rocks_ypix, xpos, ypos, yaw, worldmap_size, scale)
    
    # 6) Update worldmap (to be displayed on right side of screen)
    telReader.worldmap[obstacles_y_world, obstacles_x_world, 0] = 255
    telReader.worldmap[rocks_y_world, rocks_x_world, 1] = 255
    telReader.worldmap[navigable_y_world, navigable_x_world, 2] = 255

    return  procImage

def CreateCompositeImage(procImage, telReader):
    POVRaw = procImage['POVRaw']
    # 7) Make a mosaic image, below is some example code
    # First create a blank image (can be whatever shape you like)
    compositeHeight = POVRaw.shape[0] +  procImage['POVThreshNav'].shape[0] + telReader.worldmap.shape[0] 
    compositeWidth = POVRaw.shape[1]*2
    compositeDepth = 3  
    imgCompositeLive = np.zeros((compositeHeight, compositeWidth, compositeDepth)).astype(np.uint8)
    procImage['CompositeLive'] = imgCompositeLive
    procImage['CompositePost'] = imgCompositeLive


    # ================== COL 0 POV View ==============
    rowFirstLast=[0, POVRaw.shape[0]]
    colFirstLast=[0, procImage['POVThreshNav'].shape[1]]
    imgCompositeLive[rowFirstLast[0]:rowFirstLast[1], colFirstLast[0]:colFirstLast[1]] = procImage['POVRaw'] 
    
    #
    rowFirstLast=[POVRaw.shape[0], POVRaw.shape[0] + procImage['POVThreshNav'].shape[0]]
    colFirstLast=[0, procImage['POVThreshNav'].shape[1]]
    imgCompositeLive[rowFirstLast[0]:rowFirstLast[1], colFirstLast[0]:colFirstLast[1]] = BinaryImgToRGB(procImage['POVThreshNav'])
    
    # 
    rowFirstLast=[POVRaw.shape[0] + procImage['POVThreshNav'].shape[0], 2*POVRaw.shape[0] + procImage['POVThreshNav'].shape[0]]
    colFirstLast=[0, procImage['POVThreshNav'].shape[1]]
    imgCompositeLive[rowFirstLast[0]:rowFirstLast[1], colFirstLast[0]:colFirstLast[1]] = BinaryImgToRGB(procImage['POVThreshObs'])
  
    # ================== COL 1 MAP View ==============
    rowFirstLast=[0, POVRaw.shape[0]]
    colFirstLast=[procImage['POVThreshNav'].shape[1], 2*procImage['POVThreshNav'].shape[1]]
    imgCompositeLive[rowFirstLast[0]:rowFirstLast[1], colFirstLast[0]:colFirstLast[1]] = procImage['Map_POVRaw']
    
    # 
    rowFirstLast=[POVRaw.shape[0], POVRaw.shape[0] + procImage['POVThreshNav'].shape[0]]
    colFirstLast=[procImage['POVThreshNav'].shape[1], 2*procImage['POVThreshNav'].shape[1]]
    imgCompositeLive[rowFirstLast[0]:rowFirstLast[1], colFirstLast[0]:colFirstLast[1]] = BinaryImgToRGB(procImage['Map_POVThreshNav'])
    # 
    
    rowFirstLast=[POVRaw.shape[0] + procImage['POVThreshNav'].shape[0], 2*POVRaw.shape[0] + procImage['POVThreshNav'].shape[0]]
    colFirstLast=[procImage['POVThreshNav'].shape[1], 2*procImage['POVThreshNav'].shape[1]]
    imgCompositeLive[rowFirstLast[0]:rowFirstLast[1], colFirstLast[0]:colFirstLast[1]] = BinaryImgToRGB(procImage['Map_POVThreshObs'])


    # Overlay worldmap with ground truth map
    map_add = cv2.addWeighted(telReader.worldmap, 1, telReader.ground_truth, 0.5, 0)
    
    # Flip map overlay so y-axis points upward and add to output_image 
    #imgCompositeLive[POVRaw.shape[0]:, 0:telReader.worldmap.shape[1]] = np.flipud(map_add)

    # Then putting some text over the image
    #cv2.putText(output_image,"Navigable: Blue/Pink; Obstacle: Red; Sample Rock: Yellow", (20, 20),cv2.FONT_HERSHEY_COMPLEX, 0.4, (255, 255, 255), 1)
    telReader.count += 1 # Keep track of the index in the Databucket()
    
    return  imgCompositeLive

def BinaryImgToRGB(imgIn):
    whitePixels = (imgIn[:,:] == 1)
    shape = (imgIn.shape[0], imgIn.shape[1], 3)
    imgOut = np.zeros(shape, dtype=np.uint8)
    imgOut[whitePixels] = (255,255,255)
    return imgOut
