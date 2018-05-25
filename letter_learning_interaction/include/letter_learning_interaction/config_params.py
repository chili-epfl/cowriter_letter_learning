#!/usr/bin/env python2
# coding: utf-8
"""
Created on Sat Mar 21 20:16:25 2015

@author: ferran
"""

import rospy

# -- interaction config parameters come from launch file

#Nao parameters
NAO_IP = rospy.get_param('/activityManager/nao_ip','127.0.0.1') #default behaviour is to connect to simulator locally
naoSpeaking = rospy.get_param('/activityManager/nao_speaking',True) #whether or not the robot should speak
naoWriting = rospy.get_param('/activityManager/nao_writing',True) #whether or not the robot should move its arms
naoStanding = rospy.get_param('/activityManager/nao_standing', True) #whether or not the robot should stand or rest on its knies 
naoConnected = rospy.get_param('/activityManager/use_robot_in_interaction',True) #whether or not the robot is being used for the interaction (looking, etc.)
naoWriting = naoWriting and naoConnected #use naoConnected var as the stronger property
naoSpeaking = naoSpeaking and naoConnected

LANGUAGE = rospy.get_param('/activityManager/language','english')
NAO_HANDEDNESS = rospy.get_param('/activityManager/nao_handedness','right')
ACTIVITY_TOPIC = rospy.get_param('/activityManager/current_activity_topic','activity') #Controls the activity switch
TIME_ACTIVITY_TOPIC = rospy.get_param('/activityManager/time_activity_topic','time_activity') #Controls the activity switch
NEW_CHILD_TOPIC = rospy.get_param('/activityManager/new_teacher_topic','new_child');#Welcome a new teacher but don't reset learning algorithm's 'memory'
PUBLISH_STATUS_TOPIC = rospy.get_param('/activityManager/camera_publishing_status_topic','camera_publishing_status') #Controls the camera based on the interaction state (turn it off for writing b/c CPU gets maxed)
CLEAR_SURFACE_TOPIC = rospy.get_param('/activityManager/clear_writing_surface_topic','clear_screen')
personSide = rospy.get_param('/activityManager/person_side', NAO_HANDEDNESS.lower()) #side where person is (left/right)

FRONT_INTERACTION = True

if NAO_HANDEDNESS.lower()=='right':
    effector = "RArm"
elif NAO_HANDEDNESS.lower()=='left':
    effector = "LArm"
else: 
    print ('error in handedness param')
    
alternateSidesLookingAt = False #if true, nao will look to a different side each time. (not super tested)
global nextSideToLookAt
nextSideToLookAt = 'Right'

# -- technical parameters come from the interaction_settings module
demo_response_phrases_counter = 0
asking_phrases_after_feedback_counter = 0
asking_phrases_after_word_counter = 0
word_response_phrases_counter = 0
word_again_response_phrases_counter = 0

NUMDESIREDSHAPEPOINTS = 7.0 #Number of points to downsample the length of shapes to 
NUMPOINTS_SHAPEMODELER = 70 #Number of points used by ShapeModelers (@todo this could vary for each letter)
DOWNSAMPLEFACTOR = float(NUMPOINTS_SHAPEMODELER-1)/float(NUMDESIREDSHAPEPOINTS-1)

drawingLetterSubstates = ['WAITING_FOR_ROBOT_TO_CONNECT', 'WAITING_FOR_TABLET_TO_CONNECT', 'PUBLISHING_LETTER']
