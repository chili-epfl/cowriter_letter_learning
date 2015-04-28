#!/usr/bin/env python
#coding: utf-8

import rospy

from letter_learning_interaction.state_machine import StateMachine
from letter_learning_interaction.interaction_settings import InteractionSettings
from shape_learning.shape_modeler import ShapeModeler
from shape_learning.shape_learner_manager import ShapeLearnerManager
from letter_learning_interaction.text_shaper import TextShaper, ScreenManager
from letter_learning_interaction.helper import lookAndAskForFeedback, configure_logging
from letter_learning_interaction.config_params import *
from std_msgs.msg import String, Empty
from letter_learning_interaction.set_connexion import ConnexionToNao

from naoqi import ALProxy

rospy.init_node("joke_nao")

motion = ALProxy("ALMotion", NAO_IP, 9559)
tracker = ALProxy("ALTracker", NAO_IP, 9559)
    
# HACK: should properly configure the path from an option
configure_logging()

STOP_TOPIC = rospy.get_param('~stop_request_topic','stop_learning');#Listen for when stop card has been shown to the robot
SHAPE_LOGGING_PATH = rospy.get_param('~shape_log','') # path to a log file where all learning steps will be stored

#get appropriate angles for looking at things
headAngles_lookAtTablet_down, headAngles_lookAtTablet_right, headAngles_lookAtTablet_left, headAngles_lookAtPerson_front, headAngles_lookAtPerson_right, headAngles_lookAtPerson_left = InteractionSettings.getHeadAngles()
#initialise arrays of phrases to say at relevant times
introPhrase, demo_response_phrases, asking_phrases_after_feedback, asking_phrases_after_word, word_response_phrases, word_again_response_phrases, testPhrase, thankYouPhrase, introLearningWordsPhrase, introDrawingPhrase, againLearningWordsPhrase, againDrawingPhrase, introJokePhrase, againJokePhrase = InteractionSettings.getPhrases(LANGUAGE)
#trajectory publishing parameters
t0, dt, delayBeforeExecuting = InteractionSettings.getTrajectoryTimings(naoWriting)

pub_clear = rospy.Publisher(CLEAR_SURFACE_TOPIC, Empty, queue_size=10)

# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

stopRequestReceived = False
def onStopRequestReceived(message):
    global stopRequestReceived
    stopRequestReceived = True

changeActivityReceived = None
def onChangeActivity(message):
    global changeActivityReceived
    changeActivityReceived = message.data

def trackFace():
    global motion
    global tracker
    targetName = "Face"
    faceWidth = 0.1
    tracker.registerTarget(targetName, faceWidth)
    # Then, start tracker.
    motion.setStiffnesses("Head", 1.0)
    tracker.track(targetName)
    
# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

def startInteraction(infoFromPrevState):
    global nextSideToLookAt
    global changeActivityReceived
    
    if infoFromPrevState['state_cameFrom'] != "STARTING_INTERACTION":
        #print('------------------------------------------ WAITING_FOR_WORD')
        rospy.loginfo("STATE: STARTING_INTERACTION")
        
    if changeActivityReceived == 'joke_nao':
        if naoSpeaking:
            if(alternateSidesLookingAt): 
                lookAndAskForFeedback(introJokePhrase,nextSideToLookAt, naoWriting, naoSpeaking, textToSpeech, motionProxy, armJoints_standInit, effector)
            else:
                lookAndAskForFeedback(introJokePhrase,personSide, naoWriting, naoSpeaking, textToSpeech, motionProxy, armJoints_standInit, effector)
        nextState = "TELL_JOKE"
        infoForNextState = {'state_cameFrom': "STARTING_INTERACTION"}
    else:
        nextState = "STARTING_INTERACTION"
        infoForNextState = {'state_cameFrom': "STARTING_INTERACTION"}

    if stopRequestReceived:
        nextState = "STOPPING"
        
    return nextState, infoForNextState


def pauseInteraction(infoFromPrevState):
    global changeActivityReceived
    if changeActivityReceived == 'joke_nao':
        if naoSpeaking:
            if(alternateSidesLookingAt): 
                lookAndAskForFeedback(againJokePhrase, nextSideToLookAt, naoWriting, naoSpeaking, textToSpeech, motionProxy, armJoints_standInit, effector)
            else:
                lookAndAskForFeedback(againJokePhrase, nextSideToLookAt, naoWriting, naoSpeaking, textToSpeech, motionProxy, armJoints_standInit, effector)        
        nextState = "TELL_JOKE"
        infoForNextState = {'state_cameFrom': "PAUSE_INTERACTION"}
    else:
        nextState = "PAUSE_INTERACTION"
        infoForNextState = {'state_cameFrom': "PAUSE_INTERACTION"} 
    if stopRequestReceived:
        nextState = "STOPPING"
        
    return nextState, infoForNextState 


def tellJoke(infoFromPrevState):
    global changeActivityReceived
      
    if changeActivityReceived != 'joke_nao':
        nextState = "PAUSE_INTERACTION"
        infoForNextState = {'state_cameFrom': "TELL_JOKE"}
        
        #clear screen
        screenManager.clear()
        pub_clear.publish(Empty())
        rospy.sleep(0.5)
    else:
        if naoSpeaking:
            trackFace()
            toSay = '\RSPD=70\A husband and his wife are trying to set up a new password for their new computer. The husband puts, My penis. And the wife falls on the ground laughing because on the screen it says, Error. Not long\RSPD=100\ enough.'
            textToSpeech.say(toSay)
            
            # Stop tracker.
            tracker.stopTracker()
            tracker.unregisterAllTargets()
            
            # Switch to learning words
            msg = String()
            msg.data = "learning_words_nao"
            pub_activity.publish(msg)
            
        nextState = "PAUSE_INTERACTION"
        changeActivityReceived = "  "
        infoForNextState = {'state_cameFrom': "TELL_JOKE"}
    
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
    


def stopInteraction(infoFromPrevState):
    #print('------------------------------------------ STOPPING')
    rospy.loginfo("STATE: STOPPING")
    if naoConnected:
        motionProxy.wbEnableEffectorControl(effector,False)
        motionProxy.rest()
    nextState = "EXIT"
    infoForNextState = 0
    rospy.signal_shutdown('Interaction exited')
    return nextState, infoForNextState   

    
### --------------------------------------------------------------- MAIN
shapesLearnt = []
wordsLearnt = []
shapeLearners = []
currentWord = []
settings_shapeLearners = []


if __name__ == "__main__":
    
    datasetDirectory = rospy.get_param('~dataset_directory','default')
    if(datasetDirectory.lower()=='default'): #use default
        import inspect
        fileName = inspect.getsourcefile(ShapeModeler)
        installDirectory = fileName.split('/lib')[0]
        datasetDirectory = installDirectory + '/share/shape_learning/letter_model_datasets/uji_pen_chars2'


    stateMachine = StateMachine()
    stateMachine.add_state("STARTING_INTERACTION", startInteraction)
    stateMachine.add_state("WAITING_FOR_ROBOT_TO_CONNECT", waitForRobotToConnect)
    stateMachine.add_state("PAUSE_INTERACTION", pauseInteraction)
    stateMachine.add_state("WAITING_FOR_TABLET_TO_CONNECT", waitForTabletToConnect)
    stateMachine.add_state("TELL_JOKE", tellJoke)
    stateMachine.add_state("STOPPING", stopInteraction)
    stateMachine.add_state("EXIT", None, end_state=True)
    
    stateMachine.set_start("WAITING_FOR_ROBOT_TO_CONNECT")
    infoForStartState = {'state_goTo': ["STARTING_INTERACTION"], 'state_cameFrom': None}

    #listen for an activity change
    change_activity_subscriber = rospy.Subscriber(ACTIVITY_TOPIC, String, onChangeActivity)
    stop_subscriber = rospy.Subscriber(STOP_TOPIC, Empty, onStopRequestReceived)

    #initialise display manager for shapes (manages positioning of shapes)
    from letter_learning_interaction.srv import *
    rospy.loginfo('Waiting for display manager services to become available')
    rospy.wait_for_service('clear_all_shapes')

    rospy.sleep(2.0)  #Allow some time for the subscribers to do their thing, 
                        #or the first message will be missed (eg. first traj on tablet, first clear request locally)

    from letter_learning_interaction.watchdog import Watchdog #TODO: Make a ROS server so that *everyone* can access the connection statuses
    tabletWatchdog = Watchdog('watchdog_clear/tablet', 0.4)
    #robotWatchdog = Watchdog('watchdog_clear/robot', 0.8)

    rospy.loginfo("Nao configuration: writing=%s, speaking=%s (%s), standing=%s, handedness=%s" % (naoWriting, naoSpeaking, LANGUAGE, naoStanding, NAO_HANDEDNESS))

    myBroker, postureProxy, motionProxy, textToSpeech, armJoints_standInit = ConnexionToNao.setConnexion(naoConnected, naoWriting, naoStanding, NAO_IP, LANGUAGE, effector)
    
    #initialise word manager (passes feedback to shape learners and keeps history of words learnt)
    InteractionSettings.setDatasetDirectory(datasetDirectory)
    wordManager = ShapeLearnerManager(InteractionSettings.generateSettings, SHAPE_LOGGING_PATH)
    textShaper = TextShaper()
    screenManager = ScreenManager(0.2, 0.1395)

    stateMachine.run(infoForStartState)   
    rospy.spin()

    tabletWatchdog.stop()