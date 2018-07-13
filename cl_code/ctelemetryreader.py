# Import pandas and read in csv file as a dataframe
import pandas as pd
import matplotlib.image as mpimg
import numpy as np

# Will read in saved telemetry data and filenames from csv file and populate this object
# Worldmap is instantiated as 200 x 200 grids corresponding 
# to a 200m x 200m space (same size as the ground truth map: 200 x 200 pixels)
# This encompasses the full range of output position values in x and y from the sim
class CTelemetryReader:
    def __init__(self):
        self.telemetryInFileName = None
        self.truthMapInFileName = None

    def SetInFileNames(self, telemetry=None, truthMap=None):
        # Todo:check file exists. Logic for leaving truthMap as is
        self.telemetryInFileName = telemetry
        self.truthMapInFileName = truthMap

    def ReadInFiles(self):
        df = pd.read_csv(self.telemetryInFileName, delimiter=';', decimal='.')
        self.images = df["Path"].tolist() # Create list of image pathnames
        self.xpos = df["X_Position"].values
        self.ypos = df["Y_Position"].values
        self.yaw = df["Yaw"].values

        # Read in ground truth map and create a 3-channel image with it
        ground_truth = mpimg.imread(self.truthMapInFileName)
        self.ground_truth = np.dstack((ground_truth*0, ground_truth*255, ground_truth*0)).astype(np.float)       
        self.worldmap = np.zeros((200, 200, 3)).astype(np.float)

        self.Reset(0)

    def Reset(self, curIndex=0):
        self.count = curIndex
    
    def DumpCurData(self):
        print(self.xpos[self.count], self.ypos[self.count], self.yaw[self.count])
