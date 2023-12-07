# L3_color_tracking.py
# This program was designed to have SCUTTLE following a target using a USB camera input

import cv2              # For image capture and processing
import numpy as np      
import L2_speed_control as sc
import L2_inverse_kinematics as inv
import L2_kinematics as kin
import netifaces as ni
from time import sleep
from math import radians, pi
import sys
import L1_lidar as li
from L2_vector import getNearest, polar2cart
from L1_sweeper import set_sweeper_signal, cleanup_resources, set_servo_signal


# Gets IP to grab MJPG stream

def getIp():
    for interface in ni.interfaces()[1:]:   #For interfaces eth0 and wlan0
    
        try:
            ip = ni.ifaddresses(interface)[ni.AF_INET][0]['addr']
            return ip
            
        except KeyError:                    #We get a KeyError if the interface does not have the info
            continue                        #Try the next interface since this one has no IPv4
        
    return 0
    
#    Camera
stream_ip = getIp()
if not stream_ip: 
    print("Failed to get IP for camera stream")
    exit()

camera_input = 'http://' + stream_ip + ':8090/?action=stream'   # Address for stream

size_w  = 240   # Resized image width. This is the image width in pixels.
size_h = 160	# Resized image height. This is the image height in pixels.

fov = 1         # Camera field of view in rad (estimate)

#    Color Range, described in HSV, golf ball
v1_min = 0      # Minimum H value
v2_min = 25     # Minimum S value usually 105
v3_min = 220     # Minimum V value

v1_max = 225     # Maximum H value
v2_max = 255    # Maximum S value
v3_max = 255    # Maximum V value

target_width = 100      # Target pixel width of tracked object
angle_margin = 0.2      # Radians object can be from image center to be considered "centered"
width_margin = 10       # Minimum width error to drive forward/back
# trial 1, change angle margin from 0.2 to 0.1
# badda idea, changing to .2 and changing the tatget width to 50 from 100
#    Color Range, described in HSV, home
home_v1_min = 150     # Minimum H value
home_v2_min = 90     # Minimum S value usually 105
home_v3_min = 65     # Minimum V value

home_v1_max = 205     # Maximum H value
home_v2_max = 255    # Maximum S value
home_v3_max = 255    # Maximum V value

home_target_width = 100      # Target pixel width of tracked object
home_angle_margin = 0.2      # Radians object can be from image center to be considered "centered"
home_width_margin = 10       # Minimum width error to drive forward/back





def main():
    global k
    global h    
    k = 0
    rotation_counter = 0
    # Try opening camera with default method
    try:
        camera = cv2.VideoCapture(0)    
    except: pass

    # Try opening camera stream if default method failed
    if not camera.isOpened():
        camera = cv2.VideoCapture(camera_input)    

    camera.set(3, size_w)                       # Set width of images that will be retrived from camera
    camera.set(4, size_h)                       # Set height of images that will be retrived from camera

    try:
        while True:
            sleep(.05)                                          

            ret, image = camera.read()  # Get image from camera

            # Make sure image was grabbed
            if not ret:
                print("Failed to retrieve image!")
                break

            image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)              # Convert image to HSV

            height, width, channels = image.shape                       # Get shape of image

            thresh = cv2.inRange(image, (v1_min, v2_min, v3_min), (v1_max, v2_max, v3_max))   # Find all pixels in color range

            kernel = np.ones((5,5),np.uint8)                            # Set kernel size
            mask = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)     # Open morph: removes noise w/ erode followed by dilate
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)      # Close morph: fills openings w/ dilate followed by erode
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                    cv2.CHAIN_APPROX_SIMPLE)[-2]                        # Find closed shapes in image
            sleep(0.09)
            if len(cnts) and len(cnts) < 3 and (closestobject() == False):                             # If more than 0 and less than 3 closed shapes exist

                c = max(cnts, key=cv2.contourArea)                      # return the largest target area
                x,y,w,h = cv2.boundingRect(c)                           # Get bounding rectangle (x,y,w,h) of the largest contour
                center = (int(x+0.5*w), int(y+0.5*h))                   # defines center of rectangle around the largest target area
                angle = round(((center[0]/width)-0.5)*fov, 3)           # angle of vector towards target center from camera, where 0 deg is centered

                wheel_measured = kin.getPdCurrent()                     # Wheel speed measurements

                # If robot is facing target
                if abs(angle) < angle_margin:                                 
                    e_width = target_width - w                          # Find error in target width and measured width

                    # If error width is within acceptable margin
                    if abs(e_width) < width_margin:
                        sc.driveOpenLoop(np.array([0.,0.]))             # Stop when centered and aligned
                        print("Aligned! ",w)
                        continue

                    fwd_effort = e_width/target_width                   
                    
                    wheel_speed = inv.getPdTargets(np.array([1.2*fwd_effort, -0.5*angle]))   # Find wheel speeds for approach and heading correction
                    if (closestobject() == True):
                        k = 0
                        return 2
                    sc.driveClosedLoop(wheel_speed, wheel_measured, 0)  # Drive closed loop
                    print("Angle: ", angle, " | Target L/R: ", *wheel_speed, " | Measured L\R: ", *wheel_measured)
                    continue

                wheel_speed = inv.getPdTargets(np.array([0, -1.1*angle]))    # Find wheel speeds for only turning
                if (closestobject() == True):
                        k = 0
                        return 2
                sc.driveClosedLoop(wheel_speed, wheel_measured, 0)          # Drive robot
                print("Angle: ", angle, " | Target L/R: ", *wheel_speed, " | Measured L\R: ", *wheel_measured)
            
            



            else:
                print("No targets")
                sc.driveOpenLoop(np.array([0.,0.]))         # stop if no targets detected
                k += 1
                print(k)
                sleep(0.02)
                if (k > 2):
                    if (closestobject() == True):
                        k = 0
                        return 2
                    handleRotation()                                # perform 45/2 -degree rotation
                    sleep(0.15)
                    k = 0                                           # reset the interval counter
                    rotation_counter += 1 # increment the rotation counter
                    print("The rotation counter is at: ", rotation_counter)
                    if rotation_counter > 16: # if it has done 16 (45/4) degree rotations +2 for extra coverage of the circle and randomness of relocation
                        rotation_counter = 0
                        return 1 # break out of scanObject() with value of 1

             
    except KeyboardInterrupt: # condition added to catch a "Ctrl-C" event and exit cleanly
        print("Ctrl+C pressed. Cleaning up...")
        # Add cleanup code here, for example:
        sc.driveOpenLoop(np.array([0., 0.]))  # Stop the robot
        camera.release()  # Release the camera resource
        cv2.destroyAllWindows()  # Close any open CV2 windows
        sleep(1)  # Optional: Allow some time for cleanup
        print("Cleanup complete. Exiting Color Tracking.")
        
    finally:
    	print("Exiting Color Tracking.")
    sys.exit()



def avoidobject():
    print("Avoiding object!")
    scan = getNearest
    distancem = scan[0] # distance in meters
    angletheta = scan[1] # distance in degrees
    if(not(angletheta < 5)) and ((angletheta > -5)): # while its not centered
        if (angletheta > -135) and (angletheta < 0):
            print("The object is detected to the left of me!")
            sleep(.1)
            movementCommand(0,-pi/2, 0.15)
        if (angletheta < 0) and (angletheta > 135):
            sleep(.1)
            movementCommand(0,-pi/2, 0.15)
            print("The object is detected to the right of me!")
    else:
        print("Centered and backing")
        movementCommand(-0.25,0,0.15)
        sleep(.1)
    if (distancem > 1):
        return

    

 

def gohome():
    print("Going home!")
    set_sweeper_signal(False) # false is off
    set_servo_signal(True) # True is home, false is balls
# Try opening camera with default method
    try:
        camera = cv2.VideoCapture(0)    
    except: pass

    # Try opening camera stream if default method failed
    if not camera.isOpened():
        camera = cv2.VideoCapture(camera_input)    

    camera.set(3, size_w)                       # Set width of images that will be retrived from camera
    camera.set(4, size_h)                       # Set height of images that will be retrived from camera
    try:
        while True:
            sleep(.05)                                          
            ret, image = camera.read()  # Get image from camera
            # Make sure image was grabbed
            if not ret:
                print("Failed to retrieve image!")
                break
            image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)              # Convert image to HSV
            height, width, channels = image.shape                       # Get shape of image
            thresh2 = cv2.inRange(image, (home_v1_min, home_v2_min, home_v3_min), (home_v1_max, home_v2_max, home_v3_max))   # Find all pixels in color range
            kernel = np.ones((5,5),np.uint8)                            # Set kernel size
            mask = cv2.morphologyEx(thresh2, cv2.MORPH_OPEN, kernel)     # Open morph: removes noise w/ erode followed by dilate
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)      # Close morph: fills openings w/ dilate followed by erode
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                    cv2.CHAIN_APPROX_SIMPLE)[-2]                        # Find closed shapes in image
            
            if len(cnts) and len(cnts) < 3 and (closestobject() == False):                             # If more than 0 and less than 3 closed shapes exist

                c = max(cnts, key=cv2.contourArea)                      # return the largest target area
                x,y,w,h = cv2.boundingRect(c)                           # Get bounding rectangle (x,y,w,h) of the largest contour
                center = (int(x+0.5*w), int(y+0.5*h))                   # defines center of rectangle around the largest target area
                angle = round(((center[0]/width)-0.5)*fov, 3)           # angle of vector towards target center from camera, where 0 deg is centered

                wheel_measured = kin.getPdCurrent()                     # Wheel speed measurements

                # If robot is facing target
                if abs(angle) < angle_margin:                                 
                    e_width = target_width - w                          # Find error in target width and measured width

                    # If error width is within acceptable margin
                    if abs(e_width) < width_margin:
                        sc.driveOpenLoop(np.array([0.,0.]))             # Stop when centered and aligned
                        print("Aligned! ",w)
                        continue

                    fwd_effort = e_width/target_width                   
                    
                    wheel_speed = inv.getPdTargets(np.array([1.2*fwd_effort, -0.5*angle]))   # Find wheel speeds for approach and heading correction
                    if (closestobject() == True):
                        avoidobject()
                    sc.driveClosedLoop(wheel_speed, wheel_measured, 0)  # Drive closed loop
                    print("Angle: ", angle, " | Target L/R: ", *wheel_speed, " | Measured L\R: ", *wheel_measured)
                    continue

                wheel_speed = inv.getPdTargets(np.array([0, -1.1*angle]))    # Find wheel speeds for only turning
                if (closestobject() == True):
                        avoidobject()
                sc.driveClosedLoop(wheel_speed, wheel_measured, 0)          # Drive robot
                print("Angle: ", angle, " | Target L/R: ", *wheel_speed, " | Measured L\R: ", *wheel_measured)
            
            



            else:
                print("Home not detected")
                sc.driveOpenLoop(np.array([0.,0.]))         # stop if no targets detected
                sleep(0.25)
                movementCommand(0,pi/4,0.25) # just do a rotaiton and see if it can find home
                if (closestobject() == True):
                    avoidobject()
                    

             
    except KeyboardInterrupt: # condition added to catch a "Ctrl-C" event and exit cleanly
        print("Ctrl+C pressed. Cleaning up...")
        sc.driveOpenLoop(np.array([0., 0.]))  # Stop the robot
        camera.release()  # Release the camera resource
        cv2.destroyAllWindows()  # Close any open CV2 windows
        sleep(1)  # Optional: Allow some time for cleanup
        print("Cleanup complete. Exiting Color Tracking.")
        
    finally:
    	print("Exiting Going Home.")
    sys.exit()

def closestobject(): # looks for closest object and if its within a meter, return true, if something in way, return false
    scan = getNearest()
    distancem = scan[0] # distance in meters
    angletheta = scan[1] # distance in degrees
    print("The closest object is: ", distancem)

    if distancem < 1:
        print("The distance of the object that is too close is ", distancem)
        set_sweeper_signal(False)
        avoidobject()
        return True
    else:
        set_sweeper_signal(True)
        return False
    
def handleRotation(): # should rotate 45 degrees, need to calculate proper thetadot and time for this, 0, 2, 0.5 is for testing
    print("Performing 45-degree rotation")
    movementCommand(0, pi , .25) # 0 xdot since no movement, 2 theta dot, 0.5 seconds of thetadot movement (needs testing) v2 halves time, doubles k values v3 doubles again to .25
    print("Rotated!" )


def movementCommand(xdot,thetadot,time): # move the robot xdot, thetadot, and the time to do those commands
    B = np.array([xdot, thetadot]) # make  inputs into an array
    phis = inv.getPdTargets(B)
    sc.driveOpenLoop(phis)
    sleep(time)
    sc.driveOpenLoop(np.array([0.,0.]))
    print("Movement!" )


def relocation():
    # relocation away to another spot after doing rotations
    print("Relocating() ")
    movementCommand(3,0,1) # move the robot 2.5 m/s forward randomly for a second V2 changes the last variable to a 1
    sleep(.1)

def loop():
    global h
    while True:  # Infinite loop
        observe = main()
        if observe == 1:  # if no golf balls have been detected
            relocation()
            h += 1
            print("H has been incremented, it is ", h)
            # Check if relocation has been called 3 times
            if h == 3:
                gohome()  # Exit the loop if gohome() is called
                break  # Break out of the infinite loop
        elif observe == 2:  # if detected an object nearby
            print("An object has been detected in look_for_balls")
            avoidobject()


if __name__ == '__main__':
    global h
    h = 0
    #set_servo_signal(False) # True is home, false is balls
    #set_sweeper_signal(True)
   # print("Starting movement")
   # movementCommand(2 , 0 , 1)
   # loop()
   # print("Done with loop")
   # gohome()
    while(True):
        closestobject()

    





    



