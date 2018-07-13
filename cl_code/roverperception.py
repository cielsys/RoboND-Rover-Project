import numpy as np
import cv2
import time

from roversupport import *

# 1) Define source and destination points for perspective transform
# Retrieve the empirical pre-calculated Rover POV image to map transform matrix
matViewToMap = GetViewToMapTransform()

threshRGB_Nav =      [1, (160, 160, 160)]
threshRGB_Obstacle = [0, (130, 130, 130)]
threshHSV_Nug =      [1, (20, 150, 100), (50, 255, 255)]

def perception_step(Rover):
    """Perform perception steps to update Rover()
    :param Rover: Most recent rover state
    :return: Same rover state, updated
    """

    # Shortnames for the Rover's camera raw POV image and the processed image container
    POVRaw = Rover.img
    procImage = Rover.procImage

    # 2) Apply perspective transform to POVRaw to get Map view of the raw
    procImage['Map_POVRaw']  = perspect_transform(POVRaw, matViewToMap)

    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    procImage['Map_POVThreshNav'] = Thresh_ClipRGB(procImage['Map_POVRaw'] , threshRGB_Nav)
    procImage['Map_POVThreshObs'] = Thresh_ClipRGB(procImage['Map_POVRaw'] , threshRGB_Obstacle)
    procImage['Map_POVThreshNug'] = Thresh_ClipHSV(procImage['Map_POVRaw'] , threshHSV_Nug)
    
    # POV Threshold images not used in sim control
    #procImage['POVThreshNav'] = Thresh_ClipRGB(POVRaw , threshRGB_Nav)
    #procImage['POVThreshObs'] = Thresh_ClipRGB(POVRaw , threshRGB_Obstacle)
    #procImage['POVThreshNug'] = Thresh_ClipRGB(POVRaw , threshHSV_Nug)

    # 4) Update Rover.vision_image (InsetLeft)
    Rover.vision_image[:,:,2] = procImage['Map_POVThreshNav'] * 255
    Rover.vision_image[:,:,1] = procImage['Map_POVThreshNug'] * 255
    Rover.vision_image[:,:,0] = procImage['Map_POVThreshObs'] * 255

    # 5) Convert map image pixel values to rover-centric coords
    navigable_xpix, navigable_ypix = rover_coords(procImage['Map_POVThreshNav'])
    obstacles_xpix, obstacles_ypix = rover_coords(procImage['Map_POVThreshObs'])
    rocks_xpix, rocks_ypix = rover_coords(procImage['Map_POVThreshNug'])

    # 6) Convert rover-centric pixel values to world coordinates
    scale = 10
    xpos, ypos = Rover.pos
    yaw = Rover.yaw
    worldmap_size = Rover.worldmap.shape[0]

    navigable_x_world, navigable_y_world = pix_to_world(navigable_xpix, navigable_ypix,xpos, ypos, yaw, worldmap_size, scale)
    obstacles_x_world, obstacles_y_world = pix_to_world(obstacles_xpix, obstacles_ypix,xpos, ypos, yaw, worldmap_size, scale)
    rocks_x_world, rocks_y_world = pix_to_world(rocks_xpix, rocks_ypix, xpos, ypos, yaw, worldmap_size, scale)

    # 7) Update Rover worldmap (InsetRight)
    if (Rover.pitch < 0.5 or Rover.pitch > 359.5) and (Rover.roll < 0.5 or Rover.roll > 359.5):
        # Limit world map updates to only images that have limited roll and pitch
        Rover.worldmap[obstacles_y_world, obstacles_x_world, 0] += 1
        Rover.worldmap[rocks_y_world, rocks_x_world, 1] = 255
        Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1

    # 8) Convert rover-centric pixel positions to polar coordinates
    #navDistances, navAngles = to_polar_coords(x_pixel=navigable_xpix, y_pixel=navigable_ypix)
    
    AnalyzeNavFields(Rover, procImage['Map_POVThreshNav'])

    # Analyze rock nuggets 
    Rover.isNugVisible = (len(rocks_xpix) > 5)
    if (Rover.isNugVisible):
        nugDistances, nugAnglesRad = to_polar_coords(x_pixel=rocks_xpix, y_pixel=rocks_ypix)
        Rover.nugDistance, Rover.nugBearingDeg = np.min(nugDistances), np.mean(nugAnglesRad)*180/np.pi
    else:
        Rover.nugDistance, Rover.nugBearingDeg = 0,0

    return Rover

def AnalyzeNavFields(Rover, imgFull):
    AnalyzeNavField(Rover, 'Full', imgFull)
    AnalyzeNavField(Rover, 'RightHalf', imgFull)
    AnalyzeNavField(Rover, 'LeftHalf', imgFull)
    AnalyzeNavField(Rover, 'MiddleHalf', imgFull)

def AnalyzeNavField(Rover, fieldName, imgFull):
    minNavigableFieldSize = 50

    imgNavField = np.copy(imgFull) # Copy the full nav field binary image
    mask0 = GetMask(fieldName)
    imgNavField[mask0] = 0 # Zero out the pixels using the mask
    navPixX, navPixY = rover_coords(imgNavField) # Get 1 pixels set for plotting

    navField = Rover.navFields[fieldName]
    navField.navPixX = navPixX
    navField.navPixY = navPixY    

    fieldSize = (len(navField.navPixX)) 
    if fieldSize > 2:
        navField.meanX = np.mean(navField.navPixX)
        navField.meanY = np.mean(navField.navPixY)
        navField.meanLen = np.sqrt(np.square(navField.meanX) + np.square(navField.meanY))
        navField.meanDirRad = np.arctan2(navField.meanY, navField.meanX)
        navField.meanDirDeg = navField.meanDirRad * 180/np.pi
    else:
        navField.meanX = 0
        navField.meanY = 0
        navField.meanLen = 0
        navField.meanDirRad = 0
        navField.meanDirDeg = 0
    
    navField.isNavigable = (fieldSize > minNavigableFieldSize) and (navField.meanLen > 2)
    return