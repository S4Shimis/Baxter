#!/usr/bin/python
import rospy, cv_bridge, cv2
import cv2.cv as cv
import numpy as np
import time
import struct
import baxter_interface
import baxter_external_devices
from baxter_interface import (
    DigitalIO,
    Gripper,
    Navigator,
    CHECK_VERSION,
)
from geometry_msgs.msg import (
    Point,
    Quaternion,
    PoseStamped,
    Pose,
)
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from sensor_msgs.msg import Range

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

bridge = cv_bridge.CvBridge()

x1 = 0
y1 = 0
rad1 = 0

fgbg = cv2.BackgroundSubtractorMOG2(history=10, varThreshold=7000,bShadowDetection=False)

def image_callback_mog2(ros_img1):
    global fgbg

    img = bridge.imgmsg_to_cv2(ros_img1, desired_encoding="passthrough")
    fgmask = fgbg.apply(img)
    img = fgmask

    img = cv2.GaussianBlur(img,(3,3),cv2.BORDER_DEFAULT)

    global x1,y1,rad1

    circles = cv2.HoughCircles(img,cv.CV_HOUGH_GRADIENT,1.3,50,param1=50,param2=34,minRadius=20,maxRadius=30)
    if circles is not None:
        circles = np.uint16(np.around(circles))

        for i in circles[0,:]:
            

            if x1 == 0:
                x1=i[0]
                y1=i[1]
                rad1 = i[2]

                if (x1 >= 260 or (((x1 >= 230) and (x1 <=300)) and ((y1 >= 0) and (y1 <=80)))):
                    x1=0
                    y1=0
                    rad1=0

            else:
                if x1 < i[0]:
                    x1 = i[0]
                    y1 = i[1]
                    rad1 = i[2]
            print('x = ',x1,'; y = ', y1)
        cv2.circle(img,(x1,y1),rad1,(0,255,0),2)
        cv2.circle(img,(x1,y1),2,(0,0,255),3)
    else:
        x1 = 0
        y1 = 0
        rad1 = 0

    if ((y1 == 0) or (((x1 >= 230) and (x1 <=300)) and ((y1 >= 0) and (y1 <=80)))):
        return 0
    else :
        return 1

def image_callback_TFD(ros_img1, ros_img2, ros_img3):

    img1 = bridge.imgmsg_to_cv2(ros_img1, desired_encoding="passthrough")
    img2 = bridge.imgmsg_to_cv2(ros_img2, desired_encoding="passthrough")
    img3 = bridge.imgmsg_to_cv2(ros_img3, desired_encoding="passthrough")

    img = frame_diff(img1, img2, img3)

    img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    global x1,y1,rad1

    circles = cv2.HoughCircles(img,cv.CV_HOUGH_GRADIENT,1.3,30,param1=50,param2=20,minRadius=22,maxRadius=26)
    if circles is not None:
        circles = np.uint16(np.around(circles))

        for i in circles[0,:]:            

            if x1 == 0:
                x1=i[0]
                y1=i[1]
                rad1 = i[2]

                if (x1 >= 260 or (((x1 >= 230) and (x1 <=300)) and ((y1 >= 0) and (y1 <=80)))):
                    x1=0
                    y1=0
                    rad1=0

            else:
                if x1 < i[0]:
                    x1 = i[0]
                    y1 = i[1]
                    rad1 = i[2]
            print('x = ',x1,'; y = ', y1)
        cv2.circle(img,(x1,y1),rad1,(0,255,0),2)
        cv2.circle(img,(x1,y1),2,(0,0,255),3)
    else:
        x1 = 0
        y1 = 0
        rad1 = 0

    if ((y1 == 0) or (((x1 >= 230) and (x1 <=300)) and ((y1 >= 0) and (y1 <=80)))):
        return 0
    else :
        return 1

def frame_diff(prev_frame, cur_frame, next_frame):

    diff_frames_1 = cv2.absdiff(next_frame, cur_frame)

    diff_frames_2 = cv2.absdiff(cur_frame, prev_frame)

    return cv2.bitwise_and(diff_frames_1, diff_frames_2)
    
    cv2.waitKey(1)
def ik_move_hand1(limb, id):
    print("hmm")
    time.sleep(1)
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    global y1
    if y1 != 0:
        delta = 119-y1
    else:
        delta = 0
    print(delta)
    delta1 = 1.0/87.0
    print(delta1)
    delta1 = delta * delta1
    print(delta1)
    delta1 = delta1 / 10.0
    print(delta1)
    if id == 1:
        pose = {
            'right': PoseStamped(
                header=hdr,
                pose=Pose(
                    position=Point(
                        x=0.7 + delta1,
                        y=-0.2,
                        z=-0.03,
                    ),
                    orientation=Quaternion(
                        x=0,
                        y=1,
                        z=0,
                        w=0,
                    ),
                ),
            ),
        }

        ikreq.pose_stamp.append(pose[limb])
        try:
            rospy.wait_for_service(ns, 5.0)
            resp = iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return 1

    # Check if result valid, and type of seed ultimately used to get solution
    # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                               resp.result_type)
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                    ikreq.SEED_USER: 'User Provided Seed',
                    ikreq.SEED_CURRENT: 'Current Joint Angles',
                    ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                   }.get(resp_seeds[0], 'None')
            print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
              (seed_str,))
        # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            right = baxter_interface.Limb(limb)
            right.move_to_joint_positions(limb_joints)
            print "\nIK Joint Solution:\n", limb_joints
            print "------------------"
            print "Response Message:\n", resp
        else:
            print("INVALID POSE - No Valid Joint Solution Found.")

    return 0

def ik_move_hand(limb, id):
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    if id == 1:
        pose = {
            'right': PoseStamped(
                header=hdr,
                pose=Pose(
                    position=Point(
                        x=0.45036923630096765,
                        y=-0.6029034213165805,
                        z=-0.0946448076956774,
                    ),
                    orientation=Quaternion(
                        x=-0.0054052860719728795,
                        y=0.9997779868892971,
                        z=0.020354369689796925,
                        w=0.000677826376040635,
                    ),
                ),
            ),
        }

    elif id == 2:

        pose = {
            'right': PoseStamped(
                header=hdr,
                pose=Pose(
                    position=Point(
                        x=0.4552564328301657,
                        y=-0.6133778381085472,
                        z=0.015164699598485848,
                    ),
                    orientation=Quaternion(
                        x=-0.010288648924110909,
                        y=0.9998536033926507,
                        z=0.00027905464896251197,
                        w=0.01366885564161323,
                    ),
                ),
            ),
        }
    else:
        pose = {
            'right': PoseStamped(
                header=hdr,
                pose=Pose(
                    position=Point(
                        x=0.7,
                        y=-0.2,
                        z=0.05,
                    ),
                    orientation=Quaternion(
                        x=0,
                        y=1,
                        z=0,
                        w=0,
                    ),
                ),
            ),
        }



    ikreq.pose_stamp.append(pose[limb])
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1

    # Check if result valid, and type of seed ultimately used to get solution
    # convert rospy's string representation of uint8[]'s to int's
    resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                               resp.result_type)
    if (resp_seeds[0] != resp.RESULT_INVALID):
        seed_str = {
                    ikreq.SEED_USER: 'User Provided Seed',
                    ikreq.SEED_CURRENT: 'Current Joint Angles',
                    ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                   }.get(resp_seeds[0], 'None')
        print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
              (seed_str,))
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        right = baxter_interface.Limb(limb)
        right.move_to_joint_positions(limb_joints)
        print "\nIK Joint Solution:\n", limb_joints
        print "------------------"
        print "Response Message:\n", resp
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")

    return 0

def fk_move_hand(limb, id):
    right = baxter_interface.Limb(limb)
    if id == 1:
        pose = {'right_s0': -0.27650003701634585, 'right_s1': -0.820296226321725, 'right_w0': -0.44715539966859813, 'right_w1': 0.5748593002600588, 'right_w2': -0.37697577862284043, 'right_e0': 0.400368985638093, 'right_e1': 1.9320488023416786}
    elif id == 2:
        pose = {'right_s0': -0.2500388684253224, 'right_s1': -1.10791762405024, 'right_w0': -0.21705828148578604, 'right_w1': 0.705247667230319, 'right_w2': -0.5583690067902906, 'right_e0': 0.36393694192581444, 'right_e1': 1.9918740530692098}
    else:
        pose = {'right_s0': 0.7800292306397328, 'right_s1': -0.972160324322381, 'right_w0': -0.08168447695489828, 'right_w1': 0.9832816850345502, 'right_w2': 0.1491796316218565, 'right_e0': 0.12118448224294769, 'right_e1': 1.5612089468703798}
    right.move_to_joint_positions(pose)

def main():
    time.sleep(15)
    
    global fgbg

    fgbg = cv2.BackgroundSubtractorMOG2(history=10, varThreshold=7000,bShadowDetection=False)

    rospy.init_node('hmmm')
    right = baxter_interface.Limb('right')
    rightGripper = baxter_interface.Gripper('right')
    pose = right.endpoint_pose()
    print "current pose quaternion: ", pose
    print ""
    print "current pose angles: ", right.joint_angles()
    right.set_joint_position_speed(1.0)
    
    rightGripper = Gripper('right', False)
    
    if rightGripper.calibrate():
        print "Calibration success"
    else:
        print "Calibration failed"
    rightGripper.open()
    for x in range(10):
        ik_move_hand('right',2)
        ik_move_hand('right',1)
        rightGripper.close()
        ik_move_hand('right',2)
        ik_move_hand('right',3)

        image1 = rospy.wait_for_message('/cameras/right_hand_camera/image', Image)
        image2 = rospy.wait_for_message('/cameras/right_hand_camera/image', Image)
        image3 = rospy.wait_for_message('/cameras/right_hand_camera/image', Image)
        img = bridge.imgmsg_to_cv2(image1, desired_encoding="passthrough")
        fgmask = fgbg.apply(img)

    
        while(not image_callback_mog2(image1)):
            image1 = rospy.wait_for_message('/cameras/right_hand_camera/image', Image)
        
        '''
        while(not image_callback_TFD(image1,image2,image3)):

            image1 = image2
            image2 = image3
            image3 = rospy.wait_for_message('/cameras/right_hand_camera/image', Image)
        '''
        ik_move_hand1('right',1)
        rangeSensor = rospy.wait_for_message('/robot/range/right_hand_range/state', Range)
        while (rangeSensor.range > 65.0) or (rangeSensor.range < 0.2):
            rangeSensor = rospy.wait_for_message('/robot/range/right_hand_range/state', Range)
        rightGripper.open()
if __name__ == '__main__':
    main()