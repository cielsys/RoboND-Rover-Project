import matplotlib.image as mpimg
import matplotlib.pyplot as plt
from matplotlib import gridspec
import numpy as np

from roversupport import *

def plotMeanArrows(navPixX, navPixY):
    dist, angles = to_polar_coords(navPixX, navPixY)
    mean_dir = np.mean(angles)
    arrow_length = np.mean(dist)
    x_arrow = arrow_length * np.cos(mean_dir)
    y_arrow = arrow_length * np.sin(mean_dir)
    plt.arrow(0, 0, x_arrow, y_arrow, color='red', zorder=2, head_width=4, width=1)

def plotNavArea(title, imgIn, navPixX, navPixY):
    numImgRows = imgIn.shape[0]
    numImgCols = imgIn.shape[1]
    
    plt.title(title)
    plt.ylim(-numImgCols/2, numImgCols/2) # PlotY <- Img X
    plt.xlim(0, numImgRows) # PlotYX <- Img Y
    plt.plot(navPixX, navPixY, '.', markersize=3)

def plotNavField(fieldName, plotSelection, imgSrc, mask0):
    imgNav = np.copy(imgSrc) # Copy the full nav field binary image
    imgNav[mask0] = 0 # Zero out the pixels using the mask
    navPixX, navPixY = rover_coords(imgNav) # Get 1 pixels set for plotting

    ax = plt.subplot(plotSelection)
    plotNavArea(fieldName, imgNav, navPixX, navPixY)
    plotMeanArrows(navPixX, navPixY)
    ax.set_aspect(1)