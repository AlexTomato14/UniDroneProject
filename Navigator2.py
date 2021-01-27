#!/usr/bin/env python

#import statements:
import rospy
import math
import sys
import time
import numpy as np
from mavros_msgs.msg import OpticalFlowRad #import optical flow message structure
from mavros_msgs.msg import State  #import state message structure
from sensor_msgs.msg import Range  #import range message structure
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose #import position message structures
from geometry_msgs.msg import TwistStamped #used to set velocity messages
from mavros_msgs.srv import *   #import for arm and flight mode setting
from nav_msgs.msg import Odometry #importing odometry node
from tf.transformations import euler_from_quaternion, quaternion_from_euler






class velControl:
    def __init__(self, attPub):  #attPub = attitude publisher
        self._attPub = attPub
        self._setVelMsg = TwistStamped()
        self._targetVelX = 0
        self._targetVelY = 0
        self._targetVelZ = 0

    
    def setVel(self, coordinates):
        self._targetVelX = float(coordinates[0])
        self._targetVelY = float(coordinates[1])
        self._targetVelZ = float(coordinates[2])
        #rospy.logwarn("Target velocity is \nx: {} \ny: {} \nz: {}".format(self._targetVelX,self._targetVelY, self._targetVelZ))


    def publishTargetPose(self, stateManagerInstance):
        self._setVelMsg.header.stamp = rospy.Time.now()    #construct message to publish with time, loop count and id
        self._setVelMsg.header.seq = stateManagerInstance.getLoopCount()
        self._setVelMsg.header.frame_id = 'fcu'

        self._setVelMsg.twist.linear.x = self._targetVelX
        self._setVelMsg.twist.linear.y = self._targetVelY
        self._setVelMsg.twist.linear.z = self._targetVelZ
        
        self._attPub.publish(self._setVelMsg) 
        
        
        
        
        
class stateManager: #class for monitoring and changing state of the controller
    def __init__(self, rate):
        self._rate = rate
        self._loopCount = 0
        self._isConnected = 0
        self._isArmed = 0
        self._mode = None
    
    def incrementLoop(self):
        self._loopCount = self._loopCount + 1

    def getLoopCount(self):
        return self._loopCount

    def stateUpdate(self, msg):
        self._isConnected = msg.connected
        self._isArmed = msg.armed
        self._mode = msg.mode
        rospy.logwarn("Connected is {}, armed is {}, mode is {} ".format(self._isConnected, self._isArmed, self._mode)) #some status info

    def armRequest(self):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            modeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode) #get mode service and set to offboard control
            modeService(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print("Service mode set faild with exception: %s"%e)
    
    def offboardRequest(self):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            arm = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool) #get arm command service and arm
            arm(True)
        except rospy.ServiceException as e:   #except if failed
            print("Service arm failed with exception :%s"%e)


    def waitForPilotConnection(self):   #wait for connection to flight controller
        rospy.logwarn("Waiting for pilot connection")
        while not rospy.is_shutdown():  #while not shutting down
            if self._isConnected:   #if state isConnected is true
                rospy.logwarn("Pilot is connected")
                return True
            self._rate.sleep
        rospy.logwarn("ROS shutdown")
        return False

def distanceCheck(msg):
    global range    #import global range
    #print(msg.range) #for debugging
    range = msg.range #set range = recieved range

def localpos(msg):
	global orient, odom, roll, pitch, yaw, odomx_s, odomy_s, odomz_s, odomr_s, orientp_s, orientr_s, orienty_s
	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y
	z = msg.pose.pose.position.z
	orientation_q = msg.pose.pose.orientation
	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	(roll, pitch, yaw) = euler_from_quaternion(orientation_list)
	odom[0] = x
	odom[1] = y
	odom[2] = z

def localvel(msg):
    global vodom
    vx = msg.twist.twist.linear.x;
    vy = msg.twist.twist.linear.y;
    vz = msg.twist.twist.linear.z;

    vodom[0] = vx
    vodom[1] = vy
    vodom[2] = vz



def main():
    rospy.init_node('navigator')   # make ros node
    

    rate = rospy.Rate(20) # rate will update publisher at 20hz, higher than the 2hz minimum before tieouts occur
    stateManagerInstance = stateManager(rate) #create new statemanager

    #Subscriptions
    rospy.Subscriber("/mavros/state", State, stateManagerInstance.stateUpdate)  #get autopilot state including arm state, connection status and mode
    rospy.Subscriber("/mavros/distance_sensor/hrlv_ez4_pub", Range, distanceCheck)  #get current distance from ground 
    rospy.Subscriber("/mavros/local_position/odom", Odometry, localpos)  #get current distance from ground
    rospy.Subscriber("/mavros/local_position/odom", Odometry, localvel) #get current velocity
    global range #import global range variable
    global orient, odom, roll, pitch, yaw, odomx_s, odomy_s, odomz_s, odomr_s, orientp_s, orientr_s, orienty_s, vodom
    ###rospy.Subscriber("/mavros/px4flow/raw/optical_flow_rad", OpticalFlowRad, callback)  #subscribe to position messages


    #Publishers
    velPub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=2) ###Change to atti


    controller = velControl(velPub) #create new controller class and pass in publisher and state manager
    stateManagerInstance.waitForPilotConnection()   #wait for connection to flight controller


    odom = [0,0,0,0]
    orient = [0,0,0]
    roll = pitch = yaw = 0.00
    ct = 0
    odomx_s = [0.0]
    odomy_s = [0.0]
    odomz_s = [0.0]
    odomr_s = [0.0]
    orientp_s = [0.0]
    orientr_s = [0.0]
    orienty_s = [0.0]

    vodom = [0,0,0]
    vodomx_s = [0.0]
    vodomy_s = [0.0]
    vodomz_s = [0.0]

    xDes = [0,0,0]
    posDes = [0,0,0]
    posRel = [0,0,0]
    posRelRec = [0,0,0]
    K = np.array([0.1,0.0,0.0,0.0,0.1,0.0,0.0,0.0,0.1]).reshape(3,3)


    phase1 = False
    phase2 = False
    phase3 = False
    phase4 = False
    phase5 = False
    phase6 = False
    phase7 = False


    t_stmp = 0
    
    t0 = rospy.get_rostime()
    t1 = 0
    t6 = 0

    while not rospy.is_shutdown():
        
        #if range < 10: 
        #    controller.setVel([0,0,0.5])
        #else:
        #    controller.setVel([0.2,0,0])
        
	global orient, odom, roll, pitch, yaw, odomx_s, odomy_s, odomz_s, odomr_s, orientp_s, orientr_s, orienty_s, t_stmp, xRel, yRel, zRel,xDes
        orient = [pitch, roll, yaw]
        odom[3] = (range)
        odomx_s.append(odom[0])
        orientp_s.append(orient[0])
        odomy_s.append(odom[1])
        orientr_s.append(orient[1])
        odomz_s.append(odom[2])
        orienty_s.append(orient[2])
        odomr_s.append(odom[3])
        vodomx_s.append(odom[0])
        vodomy_s.append(odom[1])
        vodomz_s.append(odom[2])

        print('Odometry:')
        print(odom)
        print('Orientation:')
        print(orient)

        vodomx_s.append(vodom[0])
        vodomy_s.append(vodom[1])
        vodomz_s.append(vodom[2])
        print ('Velocity (odom): ')
        print(vodom)
	
	if t_stmp > 1:
	  xRel = (2 * odomx_s[t_stmp] + odomx_s[t_stmp-1] + odomx_s[t_stmp-2])/4
	  yRel = (2 * odomy_s[t_stmp] + odomy_s[t_stmp-1] + odomy_s[t_stmp-2])/4
	  zRel = (2 * odomz_s[t_stmp] + odomz_s[t_stmp-1] + odomz_s[t_stmp-2])/4
	  print("DERP")
	else:
	  xRel = (2 * odomx_s[t_stmp])/4
	  yRel = (2 * odomy_s[t_stmp])/4
	  zRel = (2 * odomz_s[t_stmp])/4
	print("xRel " + str(xRel))
	print("yRel " + str(yRel))
	print("zRel " + str(zRel))

	
	posRel = [xRel, yRel, zRel]


	#Flight Path
	#Phase 1
	if zRel	< 1.5 and xRel < 0.1 and phase1 == False:
	  xDes = [0,0,1.5]
	  posDes = [xDes[0] - posRel[0],xDes[1] - posRel[1],xDes[2] - posRel[2]]
	  controller.setVel(np.dot(K,posDes))
	#Phase 2 (Hover)
	if zRel >= 1.5 and phase2 == False:
	  phase1 = True
	  controller.setVel([0,0,0])
	  t1 = t1 + now

	  if t1 >= 2.0:
	    phase2 = True
	#Phase 3 (Forward by 1.5m)
	if phase2 == True:
	  xDes = [1.5,0,1.5]
	  posDes = [xDes[0] - posRel[0],xDes[1] - posRel[1],xDes[2] - posRel[2]]
	  controller.setVel(np.dot(K,posDes))
	if xRel >= 1.5 and phase2 == True:
	  phase3 = True
	if phase3 == True and phase4 == False:
	  dzdx = (posRel[2] - posRelRec[2]) / (posRel[0] - posRelRec[0])
	  if dzdx < 2:
	    posDes[0] = posDes[0] + 0.2 #min Step
	  elif dzdx > 4:
	    posDes[0] = posDes[0] + 1 #max step
	  else:
	    posDes[0] = posDes[0] + 0.5/dzdx

	  controller.setVel(np.dot(K,posDes))
	if xRel >= 3.5:
	  phase4 = True
	if phase4 == True and phase5 == False:
	  xDes = [6,0,1.5]
	  posDes = [xDes[0] - posRel[0],xDes[1] - posRel[1],xDes[2] - posRel[2]]
  	  controller.setVel(np.dot(K,posDes))
	if xRel >= 5.9:
	  phase5 = True
	if xRel >= 5.9 and phase6 == False:
	  phase1 = True
	  controller.setVel([0,0,0])
	  t6 = t6 + now

	  if t6 >= 2.0:
	    phase6 = True
	if phase6 == True:
	  xDes = [6,0,0]
	  posDes = [xDes[0] - posRel[0],xDes[1] - posRel[1],xDes[2] - posRel[2]]
	  controller.setVel(np.dot(K,posDes))
	posRelRec = [xRel, yRel, zRel]

	nowDur = (rospy.get_rostime() - t0)
	now = nowDur.to_sec()
	print(now)

        ct = ct + 1
        controller.publishTargetPose(stateManagerInstance)
        stateManagerInstance.incrementLoop()
        rate.sleep()    #sleep at the set rate
        t_stmp += 1

        if stateManagerInstance.getLoopCount() > 100:   #need to send some position data before we can switch to offboard mode otherwise offboard is rejected
            stateManagerInstance.offboardRequest()  #request control from external computer
            stateManagerInstance.armRequest()   #arming must take place after offboard is requested

    rospy.spin()    #keeps python from exiting until this node is stopped
    xPos = open("xPos.txt","w")
    xPos.write("odomx_s")
    xPos.write(str(odomx_s))
    xPos.close()
    yPos = open("yPos.txt","w")
    yPos.write(str(odomy_s))
    yPos.close()
    zPos = open("zPos.txt","w")
    zPos.write(str(odomz_s))
    zPos.close()

    xVel = open("xVel.txt","w")
    xVel.write(str(vodomx_s))
    xVel.close()
    yVel = open("yVel.txt","w")
    yVel.write(str(vodomy_s))
    yVel.close()
    zVel = open("zVel.txt","w")
    zVel.write(str(vodomz_s))
    zVel.close()
    """output.write(odomr_s)
    output.write(orientp_s)
    output.write(orientr_s)
    output.write(orienty_s)
    output.write("vodomx_s")
    output.write(str(vodomx_s))
    output.write(vodomy_s)
    output.write(vodomz_s)
    output.close()"""


if __name__ == '__main__':
	global orient, odom, roll, pitch, yaw, odomx_s, odomy_s, odomz_s, odomr_s, orientp_s, orientr_s, orienty_s, vodom
	main()



