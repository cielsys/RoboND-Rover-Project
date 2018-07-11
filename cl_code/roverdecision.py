import numpy as np
import time
import pprint

#from roversupport import Clip
#Wtf wont this import!
def Clip(x, lo, hi):
    return max(lo, min(hi, x))

pp = pprint.PrettyPrinter(indent=4)

def decision_step(Rover):
    """ Decision tree for determining throttle, brake and steer 
    commands based on the output of the perception_step() function

    :param Rover: Most recent rover state
    :return: Same rover state, updated
    """
    steeringBias = -0.5
    steeringGain = 0.4
   
    navFieldName, navField = SelectBestNavField(Rover)
    navField = Rover.navFields['Full']

    #+++++++++++++++++++++++ MODE: FORWARD +++++++++++++++++++++++++++++
    # Check for Rover.mode status
    if Rover.mode == 'forward': 
        
        # Check the extent of navigable terrain
        if navField.isNavigable:  
            # If mode is forward, navigable terrain looks good 
            # and velocity is below max, then throttle 
            if Rover.vel < Rover.max_vel:
                # Set throttle value to throttle setting
                Rover.throttle = Rover.throttle_set
            else: # Else coast
                Rover.throttle = 0
            Rover.brake = 0
            # Set steering to average angle clipped to the range +/- 15 
            Rover.steer = navField.meanDirDeg * steeringGain + steeringBias
        
        # If there's a lack of navigable terrain pixels then go to 'stop' mode
        elif len(Rover.nav_angles) < Rover.stop_forward:
                # Set mode to "stop" and hit the brakes!
                Rover.throttle = 0
                # Set brake to stored brake value
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.mode = 'stop'

    #+++++++++++++++++++++++ MODE: STOP +++++++++++++++++++++++++++++
    # If we're already in "stop" mode then make different decisions
    elif Rover.mode == 'stop':
        # If we're in stop mode but still moving keep braking
        if Rover.vel > 0.2:
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
            Rover.steer = 0
        
        # If we're not moving (vel < 0.2) then do something else
        elif Rover.vel <= 0.2:
            # Stopped & no way forward so turn
            if not navField.isNavigable:
                Rover.throttle = 0               
                Rover.brake = 0 # Release the brake to allow turning
                # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                Rover.steer = +15 # Could be more clever here about which way to turn

            # Stopped but it looks like we can go forward
            else: 
                # Set throttle back to stored value
                Rover.throttle = Rover.throttle_set               
                Rover.brake = 0 # Release the brake
                # Set steer to mean angle
                Rover.steer = navField.meanDirDeg * steeringGain + steeringBias
                Rover.mode = 'forward'
 
    Rover.steer = Clip(Rover.steer, -15, 15)
          
    #+++++++++++++++++++++++ NUGGET MANAGEMENT +++++++++++++++++++++++++++++
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True

    #if Rover.isNugVisible:
    
    return Rover

def SelectBestNavField(Rover):
    for navFieldName in ['RightHalf', 'MiddleHalf', 'LeftHalf']:
        navField = Rover.navFields[navFieldName]
        if navField.isNavigable:
            break

    return navFieldName, navField