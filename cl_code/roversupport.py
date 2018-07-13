import matplotlib.image as mpimg
#import matplotlib.colors as mpcolors # https://matplotlib.org/examples/color/named_colors.html
import numpy as np
import math
import cv2
from PIL import Image, ImageDraw
from io import BytesIO, StringIO
import base64
import time
import types

from utils import convert_to_float, Log, Clip
from roverdecision import SelectBestNavField

# Define pixels RGB thresholds for navigable, obstacle and nugget pixels
threshRGB_Nav =      [1, (160, 160, 160)]
threshRGB_Obstacle = [0, (130, 130, 130)]
threshHSV_Nug =      [1, (20, 150, 100), (50, 255, 255)]

colors = types.SimpleNamespace()
colors.black = (0, 0, 0)
colors.red = (255, 0, 0)
colors.green = (0, 255, 0)
colors.blue = (0, 0, 255)
colors.yellow = (255, 255, 0)
colors.cyan = (255, 255, 0)
colors.magenta = (255, 0, 255)
colors.white = (255, 0, 255)

def update_rover(Rover, data):
    # Retrieve current kinematic and control values
    Rover.vel = convert_to_float(data["speed"]) # meters/sec
    Rover.pos = [convert_to_float(pos.strip()) for pos in data["position"].split(';')]
    Rover.yaw = convert_to_float(data["yaw"])
    Rover.pitch = convert_to_float(data["pitch"])
    Rover.roll = convert_to_float(data["roll"])
    Rover.throttle = convert_to_float(data["throttle"])
    Rover.steer = convert_to_float(data["steering_angle"])

    # Initialize start time and sample positions
    if Rover.start_time == None:
        Rover.start_time = time.time()
        Rover.total_time = 0
        samples_xpos = np.int_([convert_to_float(pos.strip()) for pos in data["samples_x"].split(';')])
        samples_ypos = np.int_([convert_to_float(pos.strip()) for pos in data["samples_y"].split(';')])
        Rover.samples_pos = (samples_xpos, samples_ypos)
        Rover.samples_to_find = np.int(data['sample_count'])
        Rover.homePos = (Rover.pos[0], Rover.pos[1])
        Log("Home=" + str(Rover.homePos))
    else:
        tot_time = time.time() - Rover.start_time
        if np.isfinite(tot_time):
            Rover.total_time = tot_time

    # Near sample flag
    Rover.near_sample = np.int(data["near_sample"])
    # Picking up flag
    Rover.picking_up = np.int(data["picking_up"])

    # Update number of rocks collected
    Rover.samples_collected = Rover.samples_to_find - np.int(data["sample_count"])

    # Get the current image from the center camera of the rover
    imgString = data["image"]
    image = Image.open(BytesIO(base64.b64decode(imgString)))
    Rover.img = np.asarray(image)

    # Create dict conta8iner for various processed images
    Rover.procImage = {}
    Rover.procImage['POVRaw'] = Rover.img

    #LogRoverState(Rover, data)
    # Return updated Rover and separate image for optional saving
    return Rover, image

def LogRoverState(Rover, data):
      print(
        'speed =',Rover.vel,
        'position =', Rover.pos,
        'throttle =', Rover.throttle,
        'steer_angle =', Rover.steer,
        'near_sample:', Rover.near_sample,
        'picking_up:', data["picking_up"],
        'sending pickup:', Rover.send_pickup,
        'total time:', Rover.total_time,
        'samples remaining:', data["sample_count"],
        'samples collected:', Rover.samples_collected)


# Define a function to create display output given worldmap results
def create_output_images(Rover):
    # Create a scaled map for plotting and clean up obs/nav pixels a bit
    if np.max(Rover.worldmap[:,:,2]) > 0:
        nav_pix = Rover.worldmap[:,:,2] > 0
        navigable = Rover.worldmap[:,:,2] * (255 / np.mean(Rover.worldmap[nav_pix, 2]))
    else:
        navigable = Rover.worldmap[:,:,2]

    if np.max(Rover.worldmap[:,:,0]) > 0:
        obs_pix = Rover.worldmap[:,:,0] > 0
        obstacle = Rover.worldmap[:,:,0] * (255 / np.mean(Rover.worldmap[obs_pix, 0]))
    else:
        obstacle = Rover.worldmap[:,:,0]

    likely_nav = navigable >= obstacle
    obstacle[likely_nav] = 0

    plotmap = np.zeros_like(Rover.worldmap)
    plotmap[:, :, 0] = obstacle
    plotmap[:, :, 2] = navigable
    plotmap = plotmap.clip(0, 255)

    # Overlay obstacle and navigable terrain map with ground truth map
    map_add = cv2.addWeighted(plotmap, 1, Rover.ground_truth, 0.5, 0)

    # Check whether any rock detections are present in worldmap
    rock_world_pos = Rover.worldmap[:,:,1].nonzero()

    # If there are, we'll step through the known sample positions
    # to confirm whether detections are real
    samples_located = 0
    if rock_world_pos[0].any():
        #Log('rockcheck=' + str(samples_located))
        rock_size = 2
        for idx in range(len(Rover.samples_pos[0])):
            test_rock_x = Rover.samples_pos[0][idx]
            test_rock_y = Rover.samples_pos[1][idx]
            rock_sample_dists = np.sqrt((test_rock_x - rock_world_pos[1])**2 + \
                                  (test_rock_y - rock_world_pos[0])**2)
            # If rocks were detected within 3 meters of known sample positions
            # consider it a success and plot the location of the known
            # sample on the map
            if np.min(rock_sample_dists) < 3:
                samples_located += 1
                map_add[test_rock_y-rock_size:test_rock_y + rock_size, test_rock_x-rock_size:test_rock_x+rock_size, :] = 255

    # Calculate some statistics on the map results
    # First get the total number of pixels in the navigable terrain map
    tot_nav_pix = np.float(len((plotmap[:,:,2].nonzero()[0])))

    # Next figure out how many of those correspond to ground truth pixels
    good_nav_pix = np.float(len(((plotmap[:,:,2] > 0) & (Rover.ground_truth[:,:,1] > 0)).nonzero()[0]))

    # Next find how many do not correspond to ground truth pixels
    bad_nav_pix = np.float(len(((plotmap[:,:,2] > 0) & (Rover.ground_truth[:,:,1] == 0)).nonzero()[0]))

    # Grab the total number of map pixels
    tot_map_pix = np.float(len((Rover.ground_truth[:,:,1].nonzero()[0])))

    # Calculate the percentage of ground truth map that has been successfully found
    perc_mapped = round(100*good_nav_pix/tot_map_pix, 1)

    # Calculate the number of good map pixel detections divided by total pixels
    # found to be navigable terrain
    if tot_nav_pix > 0:
        fidelity = round(100*good_nav_pix/(tot_nav_pix), 1)
    else:
        fidelity = 0

    # Flip the map for plotting so that the y-axis points upward in the display
    map_add = np.flipud(map_add).astype(np.float32)

    # Add some text about map and rock sample detection results
    cv2.putText(map_add,"Time: "+str(np.round(Rover.total_time, 1))+' s', (0, 10), cv2.FONT_HERSHEY_COMPLEX, 0.4, (255, 255, 255), 1)
    cv2.putText(map_add,"Mapped: "+str(perc_mapped)+'%', (0, 25), cv2.FONT_HERSHEY_COMPLEX, 0.4, (255, 255, 255), 1)
    cv2.putText(map_add,"Fidelity: "+str(fidelity)+'%', (0, 40),cv2.FONT_HERSHEY_COMPLEX, 0.4, (255, 255, 255), 1)
    cv2.putText(map_add,"Nugs", (0, 55),cv2.FONT_HERSHEY_COMPLEX, 0.4, (255, 255, 255), 1)
    cv2.putText(map_add,"  Located: "+str(samples_located), (0, 70),cv2.FONT_HERSHEY_COMPLEX, 0.4, (255, 255, 255), 1)
    cv2.putText(map_add,"  Collected: "+str(Rover.samples_collected), (0, 85),cv2.FONT_HERSHEY_COMPLEX, 0.4, (255, 255, 255), 1)

    #------- InsetRight -------
    # Convert ground truth / discovered terrain map to base64 string
    pilImgRight = Image.fromarray(map_add.astype(np.uint8))
    buff = BytesIO()
    pilImgRight.save(buff, format="JPEG")
    enc64StrRightInset = base64.b64encode(buff.getvalue()).decode("utf-8")

    #------ InsetLeft -------
    pilImgLeft = Image.fromarray(Rover.vision_image.astype(np.uint8))

    # Draw nav arrow
    navFieldName, navField = SelectBestNavField(Rover)

    if navField.isNavigable:
        arrowRightX = 160 - navField.meanY # Convert from rover to Img CS
        arrowRightY = 160 - navField.meanX
        pilDrawLeft = ImageDraw.Draw(pilImgLeft) # Create a pil drawing context for drawing the nav arrows
        pilDrawLeft.line((160,160, arrowRightX, arrowRightY), fill=colors.magenta, width=3)
        #Log("arrow=" + str(navField.meanDirDeg) + "   " + str(navField.meanLen))

    buff = BytesIO()
    pilImgLeft.save(buff, format="JPEG")
    enc64StrLeftInset = base64.b64encode(buff.getvalue()).decode("utf-8")

    return enc64StrRightInset, enc64StrLeftInset

#===================================================
# Oy, this is roundabout...
def CreateMasks(imgTemplate):
    """ Create zeroing mask,
    ie all 1 valued mask pixels will zero out the corresponding image pixel
    This masks are in warped image CS, so the camera location is r160 c160 
    """
    mask0s = {}
    maskArr = np.zeros_like(imgTemplate, dtype=bool)

    # Full
    maskArr.fill(0)
    maskArr[:,:] = 1
    mask0s['Full'] = (maskArr == 0)

    # RightHalf
    maskArr.fill(0)
    maskArr[100:,160:] = 1
    mask0s['RightHalf'] = (maskArr == 0)

    # LeftHalf
    maskArr.fill(0)
    maskArr[100:,:159] = 1
    mask0s['LeftHalf'] = (maskArr == 0)

    # MiddleHalf
    maskArr.fill(0)
    maskArr[100:,80:240] = 1
    mask0s['MiddleHalf'] = (maskArr == 0)

    return mask0s

g_mask0s = CreateMasks(np.zeros((160,320),dtype=bool))

def GetMask(fieldName):
    return g_mask0s[fieldName]

def Thresh_ClipHSV(imgInRGB, thrSpecMinMax):
    """Create an binary image mask that selects pixels within the two
    the thrSpecMinMax HSV values.

    :param imgInRGB: Numpy 3d array (x, y, RGB layers)
    :param thrSpecMinMax: [maskInVal0/1, (hsvMin), (hsvMaxoptional)]
    :return: Numpy 2d array (x, y) of the binary image
    """
    #matchValue = thrSpecMinMax[0] # Todo: implement match inversion

    # Define range of yellow colors in HSV
    thrMin = np.array(thrSpecMinMax[1], dtype='uint8')
    thrMax = np.array(thrSpecMinMax[2], dtype='uint8')

    imgInHSV = cv2.cvtColor(imgInRGB, cv2.COLOR_RGB2HSV, 3)
    imgMaskOut = cv2.inRange(imgInHSV, thrMin, thrMax)
    return imgMaskOut

def Thresh_ClipRGB(imgInRGB, thrSpecMinMax):
    """Create an binary image mask that selects pixels within the two
    the thrSpecMinMax RGB values.

    :param imgInRGB: Numpy 3d array (x, y, RGB layers)
    :param thrSpecMinMax: [maskInVal0/1, (rgbMin), (rgbMaxoptional)]
    :return: Numpy 2d array (x, y) of the binary image
    """
    matchValue = thrSpecMinMax[0]

    thrMin = thrSpecMinMax[1]
    brightEnoughPixels = (imgInRGB[:,:,0] > thrMin[0]) & (imgInRGB[:,:,1] > thrMin[1]) & (imgInRGB[:,:,2] > thrMin[2])

    hasMaxThresh = len(thrSpecMinMax) == 3
    if (hasMaxThresh):
        thrMax = thrSpecMinMax[2]
        darkEnoughPixels = (imgInRGB[:,:,0] < thrMax[0]) & (imgInRGB[:,:,1] < thrMax[1]) & (imgInRGB[:,:,2] < thrMax[2])
        matchingPixels = brightEnoughPixels & darkEnoughPixels
    else:
        matchingPixels = brightEnoughPixels

    if (matchValue==0):
        imgMaskOut = np.ones_like(imgInRGB[:,:,0])
    else:
        imgMaskOut = np.zeros_like(imgInRGB[:,:,0])

    imgMaskOut[matchingPixels] = matchValue
    return imgMaskOut

def GetViewToMapTransform():
    """ Empirical calculation of Rover POV image to map transform matrix
        Uses a captured POV calibration image (height=160, width=320) with a "SquareMeter"
        grid. Returns a transform matrix to be called by cv2.warpPerspective
    """
    # Define the location of points of the "SquareMeter" trapezoid
    # as seen in the rover POV image. These are determined empirically
    # by manually locating the pixels in the calibration image
    # example_grid1.jpg
    srcTrapezoid = np.float32([
                     [119, 96], # TL
                     [14,140],  # BL
                     [302, 140],# BR
                     [200, 96]])# TR

    # Attributes of the dest square in our desired destination image
    destGridSizePix = 10.0 # Each "SquareMeter" maps to square of this many pixels
    destCenterX=160
    # Set a bottom offset to account for the fact that the bottom of the image
    #  is not the position of the rover but a bit in front of it
    bottom_offset=6
    destCenterY=160-bottom_offset

    # Define the location to where each those points will map to
    # in the destination image
    destL=destCenterX-destGridSizePix/2
    destR=destCenterX+destGridSizePix/2
    destT=destCenterY-destGridSizePix/2
    destB=destCenterY+destGridSizePix/2

    dstSquare = np.float32([
                     [destL, destT],
                     [destL, destB],
                     [destR, destB],
                     [destR, destT], ])

    matrix = cv2.getPerspectiveTransform(srcTrapezoid, dstSquare)
    return matrix

def BinaryImgToPolarMeans(imgIn):
    xpix, ypix = imgIn
    dist, angles = to_polar_coords(xpix, ypix)
    mean_angle = np.mean(angles)
    mean_dist = np.mean(dist)
    return mean_dist, mean_angle

def rover_coords(binary_img):
    """ Return the pixel coords of non-zero pixels
        in Robo CoordSys (Cam @ 0,0, X Forward, Y Left)
    """
    numImgRows = binary_img.shape[0]
    numImgCols = binary_img.shape[1]
    yImg, xImg = binary_img.nonzero()
    xRet = -yImg + numImgRows     # Img Y axis -> Neg Robo X axis
    yRet = -xImg + numImgCols/2   # Img X axis -> Neg Robo Y axis
    return xRet, yRet

def perspect_transform(img, matTransform):
    # Warp image using cv2.warpPerspective()
    # keep same size as input image
    warped = cv2.warpPerspective(img, matTransform, (img.shape[1], img.shape[0]))
    # Return the result
    return warped

def rotate_pix(xpix, ypix, yaw):
    yawRad = np.pi * yaw/180
    xpix_rotated = xpix * math.cos(yawRad) - ypix * math.sin(yawRad)
    ypix_rotated = xpix * math.sin(yawRad) + ypix * math.cos(yawRad)
    # Return the result
    return xpix_rotated, ypix_rotated

# Define a function to perform a translation
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale):
    scale = scale
    xpix_translated = xpos + xpix_rot/scale
    ypix_translated = ypos + ypix_rot/scale
    # Return the result
    return xpix_translated, ypix_translated

# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Clip to world_size
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

def to_polar_coords(x_pixel, y_pixel):
    """Convert pixels from RovCS XY to RovCS distance angle in Rads
    """
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    anglesRad = np.arctan2(y_pixel, x_pixel)
    return dist, anglesRad

def ShowRandomImage(path):
    """ Expects a folder with image files and a glob name like '../my_dataset/IMG/*'"""
    img_list = glob.glob(path)
    # Grab a random image and display it
    idx = np.random.randint(0, len(img_list)-1)
    image = mpimg.imread(img_list[idx])
    plt.imshow(image)

