#!/usr/bin/env python2
# coding: utf-8

import numpy
import os.path
from scipy import interpolate
from copy import deepcopy
from shape_learning.shape_modeler import ShapeModeler
from letter_learning_interaction.interaction_settings import InteractionSettings

import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import logging; generatedWordLogger = logging.getLogger("word_logger")

#get appropriate angles for looking at things
headAngles_lookAtTablet_down, headAngles_lookAtTablet_right, headAngles_lookAtTablet_left, headAngles_lookAtPerson_front, headAngles_lookAtPerson_right, headAngles_lookAtPerson_left = InteractionSettings.getHeadAngles()

FRONT_INTERACTION = True

# -------------------------------------------------------------- HELPER METHODS

def configure_logging(path = "/tmp"):

    if path:
        if os.path.isdir(path):
            path = os.path.join(path, "words_generated.log")
        handler = logging.FileHandler(path)
        handler.setLevel(logging.DEBUG)
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        handler.setFormatter(formatter)
    else:
        handler = logging.NullHandler()

    generatedWordLogger.addHandler(handler)
    generatedWordLogger.setLevel(logging.DEBUG)

  
def downsampleShape(shape, NUMPOINTS_SHAPEMODELER):
    #downsample user-drawn shape so appropriate size for shapeLearner
    numPointsInShape = len(shape)/2
    x_shape = shape[0:numPointsInShape]
    y_shape = shape[numPointsInShape:]

    if isinstance(x_shape,numpy.ndarray): #convert arrays to lists for interp1d
        x_shape = (x_shape.T).tolist()[0]
        y_shape = (y_shape.T).tolist()[0]

    #make shape have the same number of points as the shape_modeler
    t_current = numpy.linspace(0, 1, numPointsInShape)
    t_desired = numpy.linspace(0, 1, NUMPOINTS_SHAPEMODELER)
    f = interpolate.interp1d(t_current, x_shape, kind='linear')
    x_shape = f(t_desired)
    f = interpolate.interp1d(t_current, y_shape, kind='linear')
    y_shape = f(t_desired)

    shape = []
    shape[0:NUMPOINTS_SHAPEMODELER] = x_shape
    shape[NUMPOINTS_SHAPEMODELER:] = y_shape

    shape = ShapeModeler.normaliseShapeHeight(numpy.array(shape))
    shape = numpy.reshape(shape, (-1, 1)) #explicitly make it 2D array with only one column

    return shape
    
    
def make_bounding_box_msg(bbox, selected=False):

    bb = Float64MultiArray()
    bb.layout.data_offset = 0
    dim = MultiArrayDimension()
    dim.label = "bb" if not selected else "select" # we use the label of the first dimension to carry the selected/not selected infomation
    bb.layout.dim = [dim]
    
    x_min, y_min, x_max, y_max = bbox
    bb.data = [x_min, y_min, x_max, y_max]

    return bb

def separate_strokes_with_density(shapedWord):

    pen_ups = []
    paths = shapedWord.get_letters_paths()

    for path in paths:
        dists = []        
        for (x1,y1),(x2,y2) in zip(path[:-1],path[1:]):
            dists.append(numpy.sqrt((x1-x2)**2+(y1-y2)**2))
        mean_dist = numpy.mean(dists)
        density = numpy.array(dists)/mean_dist
        pen_up = numpy.zeros(len(density)+1)
        pen_up[density>3] = 1
        #pen_up[-1] = 1
        pen_ups.append(pen_up)

    return pen_ups



def make_traj_msg(shapedWord, deltaT, FRAME, delayBeforeExecuting, t0,pen_ups, log = False):

    traj = Path()
    traj.header.frame_id = FRAME
    traj.header.stamp = rospy.Time.now() + rospy.Duration(delayBeforeExecuting)

    pointIdx = 0
    paths = shapedWord.get_letters_paths()

    if log:
        generatedWordLogger.info("%s" % [[(x,-y) for x, y in path] for path in paths])

    j = -1
    for path in paths:
        j+=1
        pen_up = pen_ups[j]
        first = True
        i = 1
        for x, y in path[1:-1]:
            point = PoseStamped()

            point.pose.position.x = x
            point.pose.position.y = y   
            point.header.frame_id = FRAME
            print point.header.frame_id
            point.header.stamp = rospy.Time(t0 + pointIdx * deltaT) #@TODO allow for variable time between points for now

            if first or pen_up[i]==1 or pen_up[i-1]==1:# or pen_up[i+1]==1:
                point.header.seq = 1 # set header.seq=1 each time new stroke !!!!
                first = False

            traj.poses.append(deepcopy(point))

            pointIdx += 1
            i+=1

    return traj


def lookAtTablet(motionProxy, effector):
    if FRONT_INTERACTION:
        motionProxy.setAngles(["HeadYaw", "HeadPitch"],headAngles_lookAtTablet_down,0.2)

    else:
        if(effector=="RArm"):   #tablet will be on our right
            motionProxy.setAngles(["HeadYaw", "HeadPitch"],headAngles_lookAtTablet_right,0.2)
        else: 
            motionProxy.setAngles(["HeadYaw", "HeadPitch"],headAngles_lookAtTablet_left,0.2)

def lookAndAskForFeedback(toSay, side, naoWriting, naoSpeaking, textToSpeech, motionProxy, armJoints_standInit, effector):
    if naoWriting:
        #put arm down
        motionProxy.angleInterpolationWithSpeed(effector,armJoints_standInit, 0.3)

    if FRONT_INTERACTION:
        motionProxy.setAngles(["HeadYaw", "HeadPitch"],headAngles_lookAtPerson_front,0.2)
    else:
        if(side=="Right"):   #person will be on our right
            motionProxy.setAngles(["HeadYaw", "HeadPitch"],headAngles_lookAtPerson_right,0.2)
        else:                   #person will be on our left
            motionProxy.setAngles(["HeadYaw", "HeadPitch"],headAngles_lookAtPerson_left,0.2)

    if naoSpeaking:
        textToSpeech.say(toSay)
        rospy.loginfo('NAO: '+toSay)
