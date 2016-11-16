#!/usr/bin/env python
# coding: utf-8

import rospy
from std_msgs.msg import String, Empty, Int32
import time
from letter_learning_interaction.state_machine import StateMachine 
from letter_learning_interaction.helper import configure_logging, lookAndAskForFeedback
from letter_learning_interaction.config_params import *
from letter_learning_interaction.set_connexion import ConnexionToNao
from letter_learning_interaction.interaction_settings import InteractionSettings

rospy.init_node("activity_manager")

# HACK: should properly configure the path from an option
configure_logging()

# -- interaction config parameters come from launch file
STOP_TOPIC = rospy.get_param('~stop_request_topic','stop_learning');#Listen for when stop card has been shown to the robot

#get appropriate angles for looking at things
headAngles_lookAtTablet_down, headAngles_lookAtTablet_right, headAngles_lookAtTablet_left, headAngles_lookAtPerson_front, headAngles_lookAtPerson_right, headAngles_lookAtPerson_left = InteractionSettings.getHeadAngles()
#initialise arrays of phrases to say at relevant times
introPhrase, demo_response_phrases, asking_phrases_after_feedback, asking_phrases_after_word, word_response_phrases, word_again_response_phrases, testPhrase, thankYouPhrase, introLearningWordsPhrase, introDrawingPhrase, againLearningWordsPhrase, againDrawingPhrase, introJokePhrase, againJokePhrase,refusing_response_phrases, wrong_way_response_phrases = InteractionSettings.getPhrases(LANGUAGE)
#trajectory publishing parameters
t0, dt, delayBeforeExecuting = InteractionSettings.getTrajectoryTimings(naoWriting)

start_time = time.time()

pub_activity = rospy.Publisher(ACTIVITY_TOPIC, String, queue_size=10) #Publishes the current activity which is performed
pub_activity_time = rospy.Publisher('/activity_time', Int32, queue_size=10)  #Publishes the time spend in the current activity

# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

stopRequestReceived = False
def onStopRequestReceived(message):
    global stopRequestReceived
    stopRequestReceived = True

changeActivityReceived = None
def onChangeActivity(message):
    global changeActivityReceived
    changeActivityReceived = message.data

# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
def startInteraction(infoFromPrevState):
    global nextSideToLookAt
    #print('------------------------------------------ STARTING_INTERACTION')
    rospy.loginfo("STATE: STARTING_INTERACTION")
    if naoSpeaking:
        if(alternateSidesLookingAt): 
            lookAndAskForFeedback(introPhrase,nextSideToLookAt, naoWriting, naoSpeaking, textToSpeech, motionProxy, armJoints_standInit, effector)
            pub_activity.publish("learning_words_nao")
            rospy.sleep(1) 
        else:
            lookAndAskForFeedback(introPhrase,personSide, naoWriting, naoSpeaking, textToSpeech, motionProxy, armJoints_standInit, effector)
            rospy.sleep(1)            
            pub_activity.publish("learning_words_nao")

    nextState = "ACTIVITY"
    infoForNextState = {'state_cameFrom': "STARTING_INTERACTION"}
    if stopRequestReceived:
        nextState = "STOPPING"
    return nextState, infoForNextState

infoToRestore_waitForTabletToConnect = None
def waitForTabletToConnect(infoFromPrevState):
    global infoToRestore_waitForTabletToConnect
    #FORWARDER STATE
    if infoFromPrevState['state_cameFrom'] != "WAITING_FOR_TABLET_TO_CONNECT":
        #print('------------------------------------------ waiting_for_tablet_to_connect')
        rospy.loginfo("STATE: waiting_for_tablet_to_connect")
        infoToRestore_waitForTabletToConnect = infoFromPrevState

    nextState = "WAITING_FOR_TABLET_TO_CONNECT"
    infoForNextState = {'state_cameFrom': "WAITING_FOR_TABLET_TO_CONNECT"}

    if(tabletWatchdog.isResponsive()): #reconnection - send message to wherever it was going
        infoForNextState = infoToRestore_waitForTabletToConnect
        nextState = infoForNextState['state_goTo'].pop(0)
    else:
        rospy.sleep(0.1) #don't check again immediately

    if stopRequestReceived:
        nextState = "STOPPING"
    return nextState, infoForNextState    

infoToRestore_waitForRobotToConnect = None
def waitForRobotToConnect(infoFromPrevState):
    global infoToRestore_waitForRobotToConnect
    #FORWARDER STATE
    if infoFromPrevState['state_cameFrom'] != "WAITING_FOR_ROBOT_TO_CONNECT":
        #print('------------------------------------------ waiting_for_robot_to_connect')
        rospy.loginfo("STATE: waiting_for_robot_to_connect")
        infoToRestore_waitForRobotToConnect = infoFromPrevState

    nextState = "WAITING_FOR_ROBOT_TO_CONNECT"
    infoForNextState = {'state_cameFrom': "WAITING_FOR_ROBOT_TO_CONNECT"}

    #if robotWatchdog.isResponsive() or not naoConnected:
    if(True): #don't use watchdog for now
        infoForNextState = infoToRestore_waitForRobotToConnect
        nextState = infoForNextState['state_goTo'].pop(0)
    else:
        rospy.sleep(0.1) #don't check again immediately

    if stopRequestReceived:
        nextState = "STOPPING"
    return nextState, infoForNextState
       
def activity(infoFromPrevState):
    global start_time 
    global changeActivityReceived

    
    if infoFromPrevState['state_cameFrom'] != "ACTIVITY":
        #print('------------------------------------------ waiting_for_robot_to_connect')
        rospy.loginfo("STATE: ACTIVITY")    
    
    if changeActivityReceived == 'drawing_nao':
        changeActivityReceived = " "
        start_time = time.time()

        nextState = "ACTIVITY"
        infoForNextState = {'state_cameFrom': "ACTIVITY"}
    elif changeActivityReceived == 'learning_words_nao':
        changeActivityReceived = " "
        start_time = time.time()

        nextState = "ACTIVITY"
        infoForNextState = {'state_cameFrom': "ACTIVITY"}
    elif changeActivityReceived == 'joke_nao':
        changeActivityReceived = " "
        start_time = time.time()

        nextState = "ACTIVITY"
        infoForNextState = {'state_cameFrom': "ACTIVITY"}
    else:
        nextState = "ACTIVITY"
        infoForNextState = {'state_cameFrom': "ACTIVITY"}
    rospy.sleep(1)        
    end_time = time.time()
    elapsed = end_time - start_time        
    pub_activity_time.publish(elapsed)     
        
    if stopRequestReceived:
        nextState = "STOPPING"
    return nextState, infoForNextState
    
def stopInteraction(infoFromPrevState):
    rospy.loginfo("STATE: STOPPING")
    if naoSpeaking:
        textToSpeech.say(thankYouPhrase)
    if naoConnected:
        motionProxy.wbEnableEffectorControl(effector,False)
        motionProxy.rest()
    nextState = "EXIT"
    infoForNextState = 0
    rospy.signal_shutdown('Interaction exited')
    return nextState, infoForNextState
    
    
### --------------------------------------------------------------- MAIN
settings_shapeLearners = []


if __name__ == "__main__":
    stateMachine = StateMachine()
    stateMachine.add_state("ACTIVITY", activity)
    stateMachine.add_state("WAITING_FOR_ROBOT_TO_CONNECT", waitForRobotToConnect)
    stateMachine.add_state("WAITING_FOR_TABLET_TO_CONNECT", waitForTabletToConnect)
    stateMachine.add_state("STARTING_INTERACTION", startInteraction)
    stateMachine.add_state("STOPPING", stopInteraction)
    stateMachine.add_state("EXIT", None, end_state=True)
    stateMachine.set_start("WAITING_FOR_ROBOT_TO_CONNECT")
    infoForStartState = {'state_goTo': ["STARTING_INTERACTION"], 'state_cameFrom': None}

    #listen for when to stop
    stop_subscriber = rospy.Subscriber(STOP_TOPIC, Empty, onStopRequestReceived)
    #listen for an activity change
    change_activity_subscriber = rospy.Subscriber(ACTIVITY_TOPIC, String, onChangeActivity)

    from letter_learning_interaction.watchdog import Watchdog #TODO: Make a ROS server so that *everyone* can access the connection statuses
    tabletWatchdog = Watchdog('watchdog_clear/tablet', 0.4)
    
    #initialise display manager for shapes (manages positioning of shapes)
    rospy.loginfo('Waiting for display manager services to become available')

    rospy.sleep(2.0)  #Allow some time for the subscribers to do their thing, 
                        #or the first message will be missed (eg. first traj on tablet, first clear request locally)

    rospy.loginfo("Nao configuration: writing=%s, speaking=%s (%s), standing=%s, handedness=%s" % (naoWriting, naoSpeaking, LANGUAGE, naoStanding, NAO_HANDEDNESS))

    myBroker, postureProxy, motionProxy, textToSpeech, armJoints_standInit = ConnexionToNao.setConnexion(naoConnected, naoWriting, naoStanding, NAO_IP, LANGUAGE, effector)

    stateMachine.run(infoForStartState)   
    rospy.spin()   
    tabletWatchdog.stop()
