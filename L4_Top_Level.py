# This is a top level program used to help debug, control, and ultimately run the robot.
# The goal is to have multiple mini funcitons within this program that utilize other known libraries
# (such as the L2_speed_control.py as found in the SCUTTLE main github)
# most of these libraries will have to be imported if you did not do them on your own scuttle for lab

# please be very VERY precise with the comments you add


# keep global variables, may keep them local but global just in case for now
# ----- K definition below -----
#K is defined as a loop counter for having not detected a golfball. Ideally, we want a loop counter
# for this because it may misfire and not detect a golfball in one time instance, but would the next
#( say like 1ms later) so a time counter that iterates and if say like .5 seconds have passed with no 
#balls, then rotate
#------ end of K defition ---------
global k
k = 0
rotation_counter = 0 # counter rotations if no balls detected
global h
h = 0  # similar counter for rotaion but for relocation

# Import external programs
import time         # keeping time
import numpy as np  # for handling matrices

# Import internal programs (these are on the scuttle github)
import L2_track_target as track
import L2_speed_control as sc
import L2_inverse_kinematics as inv
import L1_lidar as li
from L2_vector import getNearest, polar2cart

# Declare relevant variables (most of these are from the follow colored target program)
radius = 0                          # radius of target
cruiseRate = 0.6                    # speed for cruising (fraction)
turnRate = 0.8                      # speed for turning (fraction)
r1 = 28                             # radius desired (pixels)
tol = 3                             # radius tolerance (pixels)
centerBand = 0.20                   # portion of FOV to consider target centered

# Declare variables for time-keeping
t1 = time.monotonic()  # declare vars
t0 = t1              # for keeping time
print("initial time: ", t1)




def forwardFunction(r0):            # when the target is straight on, approach          
    xdt = 0                         # initial x_dot target is zero
    if r0 < (r1 - tol):             # if target looks too small
        xdt = cruiseRate            # x_dot target, drive fwd       
    elif r0 > (r1 + tol):           # if target looks too big
        xdt = -1* cruiseRate        # x_dot target, drive backwards
    return xdt

def turnAndGo(xVal):                                           # turn towards object and approach
    if abs(x_offset) > centerBand:                             # if x_offset is outside the band, make a turn
        pdt = np.sign(x_offset) * x_offset * x_offset * turnRate # square the offset, keep the sign, & scale by constant
        chassisTargets = inv.map_speeds(np.array([0, pdt]))    # generate xd, td
    else: 
        xdt = forwardFunction(radius)                          # determine forward speed
        chassisTargets = inv.map_speeds(np.array([xdt,0]))     # set td zero
    return chassisTargets

def handleRotation(): # should rotate 45 degrees, need to calculate proper thetadot and time for this, 0, 2, 0.5 is for testing
    print("Performing 45-degree rotation")
    movementCommand(0,2,0.5) # 0 xdot since no movement, 2 theta dot, 0.5 seconds of thetadot movement (needs testing)
    print("Rotated!" )


def movementCommand(xdot,thetadot,time) # move the robot xdot, thetadot, and the time to do those commands
    B = np.array([xdot, thetadot]) # make  inputs into an array
    phis = inv.getPdTargets(B)
    sc.driveOpenLoop(phis)
    sleep(time)
    sc.driveOpenLoop(np.array([0.,0.]))
    print("Movement!" )


def relocation()
    # relocation away to another spot after doing rotations
    movementCommand(2.5,0,1) # move the robot 2.5 m/s forward randomly for a second


def scanObject():
    global rotation_counter # global rotation counter
    while True:
        target = track.colorTarget(track.color_range)           # generate a target
        t1 = time.monotonic()                                   # measure loop end time
        t = round(t1 - t0, 3)                                   # compute loop in seconds
        t0 = t1                                                 # reset loop base

        if target[0] is None:                                   # if there is no colored target detected
            k += 1                                              # increment the interval counter
            print("interval: ", t, "\tno target located. Interval k: delaying for .25 seconds", k)
            time.sleep(0.25)
            if k == 5:
                handleRotation()                                # perform 45-degree rotation
                k = 0                                           # reset the interval counter
                rotation_counter += 1 # increment the rotation counter
                if rotation_counter == 4: # if it has done 4 45 degree rotations
                    return 1 # break out of scanObject() with value of 1
                
        else:
            if not closestobject(): # if closest object isnt too close
                k = 0                                               # reset the interval counter when target is detected
                x_offset = round(track.getAngle(target[0]),2)       # find angle of the target's x_offset (% of FOV)
                radius = int(target[2])                             # find out the radius of the target
                print("interval: ", t, "\t", "Target position: ", x_offset, "\t radius", radius)
                chassisTargets = turnAndGo(x_offset)                # take the x target location & generate turning
                pdTargets = inv.convert(chassisTargets)             # phi dot targets (rad/s)
                sc.driveOpenLoop(pdTargets)                         # command motors in open-loop fashion
                time.sleep(0.07) # run the motors for 0.07 seconds and then it looks for the object? Not sure if this is exactly where this goes
            else # if closest object is too close
                avoidobject()

def closestobject():
    scan = getNearest()
    distancem = scan[0] # distance in meters
    angletheta = scan[1] # distance in degrees
    
    if distancem < 0.25:
        return True
    else:
        return False

def turnoffmotor() # code to turn off main motor

def turnonmotor() # code to turn on main motor


def avoidobject()
    # code to check the side ultrasonics and move based on the feedback
    # this is the main avoid object, it should restart its search or return home after this



def gohome():
    print("Going Home!")
    # need code for going home here


def look_for_balls():
    global h
    while True:
        observe = scanObject()

        if observe == 1: # if no golfballs have been detected
            relocation()
            h += 1

            # Check if relocation has been called 3 times
            if h == 3:
                break  # Exit the loop if gohome() is called
            else:
                # Run scanObject again if relocation is not called 3 times
                continue
        if observe == 2 : # if detected an object nearby





# THIS SECTION ONLY RUNS IF THE PROGRAM IS CALLED DIRECTLY, THIS SHOULD BE THE MAIN AUTONOMOUS PROGRAM
if __name__ == "__main__":
    
    look_for_balls() # look for balls part of the program, which includes scanning, rotating, and relocating. Breaks from program after it doesnt find balls and must go home
    gohome() # goes home if it does not find any balls
    



