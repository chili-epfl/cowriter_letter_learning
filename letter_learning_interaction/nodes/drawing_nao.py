#!/usr/bin/env python2
#coding: utf-8

import rospy

from letter_learning_interaction.state_machine import StateMachine
from letter_learning_interaction.interaction_settings import InteractionSettings
from letter_learning_interaction.text_shaper import ScreenManager
from letter_learning_interaction.helper import lookAndAskForFeedback, configure_logging
from letter_learning_interaction.config_params import *
from std_msgs.msg import String, Empty
from letter_learning_interaction.set_connexion import ConnexionToNao

from naoqi import ALProxy

motion = ALProxy("ALMotion", NAO_IP, 9559)
tracker = ALProxy("ALTracker", NAO_IP, 9559)
tts = ALProxy("ALTextToSpeech", NAO_IP, 9559)

rospy.init_node("drawing_nao")

# HACK: should properly configure the path from an option
configure_logging()

STOP_TOPIC = rospy.get_param('~stop_request_topic','stop_learning');#Listen for when stop card has been shown to the robot
#SHAPE_LOGGING_PATH = rospy.get_param('~shape_log','') # path to a log file where all learning steps will be stored

#get appropriate angles for looking at things
headAngles_lookAtTablet_down, headAngles_lookAtTablet_right, headAngles_lookAtTablet_left, headAngles_lookAtPerson_front, headAngles_lookAtPerson_right, headAngles_lookAtPerson_left = InteractionSettings.getHeadAngles()
#initialise arrays of phrases to say at relevant times
introPhrase, demo_response_phrases, asking_phrases_after_feedback, asking_phrases_after_word, word_response_phrases, word_again_response_phrases, testPhrase, thankYouPhrase, introLearningWordsPhrase, introDrawingPhrase, againLearningWordsPhrase, againDrawingPhrase, introJokePhrase, againJokePhrase,refusing_response_phrases, wrong_way_response_phrases = InteractionSettings.getPhrases(LANGUAGE)
#trajectory publishing parameters
t0, dt, delayBeforeExecuting = InteractionSettings.getTrajectoryTimings(naoWriting)

pub_clear = rospy.Publisher(CLEAR_SURFACE_TOPIC, Empty, queue_size=10)
pub_activity = rospy.Publisher("activity", String, queue_size=10)

first_story = True

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

    if changeActivityReceived == 'drawing_nao':
        if naoSpeaking:
            if(alternateSidesLookingAt):
                rospy.sleep(2)
                lookAndAskForFeedback(introDrawingPhrase,nextSideToLookAt, naoWriting, naoSpeaking, textToSpeech, motionProxy, armJoints_standInit, effector)
            else:
                rospy.sleep(2)
                lookAndAskForFeedback(introDrawingPhrase,personSide, naoWriting, naoSpeaking, textToSpeech, motionProxy, armJoints_standInit, effector)
        nextState = "DRAWING"
        infoForNextState = {'state_cameFrom': "STARTING_INTERACTION"}
    else:
        nextState = "STARTING_INTERACTION"
        infoForNextState = {'state_cameFrom': "STARTING_INTERACTION"}

    if stopRequestReceived:
        nextState = "STOPPING"

    return nextState, infoForNextState


def pauseInteraction(infoFromPrevState):
    global changeActivityReceived
    if changeActivityReceived == 'drawing_nao':
        if naoSpeaking:
            if(alternateSidesLookingAt):
                rospy.sleep(2)
                lookAndAskForFeedback(againDrawingPhrase, nextSideToLookAt, naoWriting, naoSpeaking, textToSpeech, motionProxy, armJoints_standInit, effector)
            else:
                rospy.sleep(2)
                lookAndAskForFeedback(againDrawingPhrase, nextSideToLookAt, naoWriting, naoSpeaking, textToSpeech, motionProxy, armJoints_standInit, effector)
        nextState = "DRAWING"
        infoForNextState = {'state_cameFrom': "PAUSE_INTERACTION"}
    else:
        nextState = "PAUSE_INTERACTION"
        infoForNextState = {'state_cameFrom': "PAUSE_INTERACTION"}

    if stopRequestReceived:
        nextState = "STOPPING"

    return nextState, infoForNextState


def drawing(infoFromPrevState):
    global changeActivityReceived
    global first_story

    textToSpeech.setLanguage("French")

    if changeActivityReceived != 'drawing_nao':
        nextState = "PAUSE_INTERACTION"
        infoForNextState = {'state_cameFrom': "DRAWING"}

        #clear screen
        screenManager.clear()
        pub_clear.publish(Empty())
        rospy.sleep(0.5)

    else:
        names  = "Body"
        fractionMaxSpeed  = 0.2
        resting_sitting = [-0.007711887359619141, 0.07972598075866699, 1.3805580139160156, 0.15949392318725586, -0.7808480262756348, -1.061486005783081, 0.09506607055664062, 0.014799952507019043, -0.24846601486206055, -0.07205605506896973, -0.7009961605072021, 2.112546443939209, -1.1894419193267822, 0.07501578330993652, -0.24846601486206055, 0.06447005271911621, -0.7133519649505615, 2.112546443939209, -1.186300277709961, -0.07512402534484863, 1.4067201614379883, -0.16878199577331543, 0.7822980880737305, 1.0769100189208984, -0.10128593444824219, 0.018800020217895508]
        resting_standing = [-0.004643917083740234, 0.01683211326599121, 1.4388500452041626, 0.27454400062561035, -1.3699040412902832, -0.98785400390625, 0.010695934295654297, 0.26080000400543213, 4.1961669921875e-05, 4.1961669921875e-05, -0.4494199752807617, 0.6994619369506836, -0.3497939109802246, 4.1961669921875e-05, 4.1961669921875e-05, 4.1961669921875e-05, -0.44950389862060547, 0.7010800838470459, -0.34970998764038086, 4.1961669921875e-05, 1.4205260276794434, -0.27922987937927246, 1.389762043952942, 0.9910058975219727, 0.042910099029541016, 0.25760000944137573]

        # Reset speech parameters to nominal.
        tts.setParameter("pitchShift", 0)
        tts.setVolume(0.5)

        if first_story:
            if naoSpeaking:
                trackFace()
                #toSay = '\RSPD=60\ A mouse was having a very bad time.'
                toSay = '\RSPD=60\ Il était une fois une petite souris qui était affamée.'
                textToSpeech.say(toSay)
                motionProxy.setStiffnesses("Body", 1.0)
                angles = []

                if naoStanding:
                    angles = [-0.05373191833496094, 0.5047179460525513, 1.3176640272140503, -0.31297802925109863, -1.376039981842041, -0.4049339294433594, -0.0353238582611084, 0.26080000400543213, 4.1961669921875e-05, 4.1961669921875e-05, -0.4494199752807617, 0.6994619369506836, -0.3497939109802246, 4.1961669921875e-05, 4.1961669921875e-05, 4.1961669921875e-05, -0.4510378837585449, 0.7010800838470459, -0.34970998764038086, 4.1961669921875e-05, 1.37910795211792, 0.2990880012512207, 1.4480540752410889, 0.4970579147338867, 0.009161949157714844, 0.25760000944137573]
                else:
                    angles = [0.0014920234680175781, 0.5147144794464111, 1.406636118888855, 0.09966802597045898, -2.046398162841797, -0.27761197090148926, 0.970980167388916, 0.012799978256225586, -0.30062198638916016, -0.059783935546875, -0.6641800403594971, 2.112546443939209, -1.1894419193267822, 0.07501578330993652, -0.30062198638916016, 0.042994022369384766, -0.6734678745269775, 2.112546443939209, -1.186300277709961, -0.06438612937927246, 1.6337518692016602, -0.02611994743347168, 1.8269520998001099, 0.5553500652313232, -0.05373191833496094, 0.019999980926513672]

                motionProxy.post.setAngles(names, angles, fractionMaxSpeed)
                #toSay = 'She could find no food at all.'
                toSay = 'Impossible de trouver de quoi manger!'
                textToSpeech.say(toSay)

                if naoStanding:
                    motionProxy.setAngles(names, resting_standing, fractionMaxSpeed)
                else:
                    motionProxy.setAngles(names, resting_sitting, fractionMaxSpeed)

                #+++++++++++++++++++++++++++++++++++++++++++++++

                #toSay = 'She looked everywhere, but there was no food, and she grew'
                toSay = 'Elle avait cherché partout, mais impossible de mettre la patte sur le moindre petit bout de fromage, et elle était maintenant devenue'
                textToSpeech.say(toSay)
                motionProxy.setStiffnesses("Body", 1.0)

                if naoStanding:
                    angles = [-0.07674193382263184, 0.009161949157714844, 0.8743381500244141, -0.257753849029541, -1.3806419372558594, -0.7224719524383545, -0.10895586013793945, 0.7791999578475952, 4.1961669921875e-05, 4.1961669921875e-05, -0.4494199752807617, 0.7025301456451416, -0.3497939109802246, 4.1961669921875e-05, 4.1961669921875e-05, 4.1961669921875e-05, -0.44950389862060547, 0.6995458602905273, -0.34970998764038086, 4.1961669921875e-05, 0.8836259841918945, 0.3141592741012573, 1.5554341077804565, 0.7455658912658691, -0.029187917709350586, 0.8320000171661377]
                else:
                    angles = [-0.007711887359619141, 0.14568805694580078, 1.0492141246795654, -0.3141592741012573, -1.42972993850708, -0.894279956817627, 0.21011614799499512, 0.20639997720718384, -0.28067994117736816, -0.056715965270996094, -0.6166260242462158, 2.112546443939209, -1.1894419193267822, 0.07501578330993652, -0.28067994117736816, 0.04146003723144531, -0.6305160522460938, 2.112546443939209, -1.186300277709961, -0.06898808479309082, 1.075376033782959, 0.3141592741012573, 1.5769100189208984, 0.754770040512085, -0.2654240131378174, 0.07599997520446777]
                motionProxy.post.setAngles(names, angles, fractionMaxSpeed)
                #toSay = 'very thin.'
                toSay = 'toute maigre.'
                textToSpeech.say(toSay)

                if naoStanding:
                    motionProxy.setAngles(names, resting_standing, fractionMaxSpeed)
                else:
                    motionProxy.setAngles(names, resting_sitting, fractionMaxSpeed)

                #+++++++++++++++++++++++++++++++++++++++++++++++

                #toSay = 'Finally, the mouse found a basket,'
                toSay = 'Pourtant, un matin, la petite souris découvre un panier,'
                textToSpeech.say(toSay)
                motionProxy.setStiffnesses("Body", 1.0)

                if naoStanding:
                    angles = [-0.07827591896057129, 0.009161949157714844, 0.19631004333496094, 0.3849921226501465, -1.5601201057434082, -0.6488399505615234, -0.2623560428619385, 0.7791999578475952, 4.1961669921875e-05, 4.1961669921875e-05, -0.45095396041870117, 0.7009961605072021, -0.35132789611816406, 4.1961669921875e-05, 4.1961669921875e-05, 4.1961669921875e-05, -0.44950389862060547, 0.6980118751525879, -0.34970998764038086, 4.1961669921875e-05, 0.23014187812805176, -0.5123980045318604, 1.6566780805587769, 0.48325204849243164, 0.06898808479309082, 0.8320000171661377]
                else:
                    angles = [-0.0061779022216796875, 0.08432793617248535, -0.11355805397033691, 0.6457719802856445, -2.03719425201416, -1.0952341556549072, 1.004728078842163, 0.015200018882751465, -0.2791459560394287, -0.056715965270996094, -0.6181600093841553, 2.112546443939209, -1.1894419193267822, 0.07501578330993652, -0.2791459560394287, 0.027653932571411133, -0.6289820671081543, 2.112546443939209, -1.186300277709961, -0.05058002471923828, -0.052114009857177734, -0.8636841773986816, 1.8361561298370361, 1.0968518257141113, 0.21625208854675293, 0.020400047302246094]

                motionProxy.post.setAngles(names, angles, fractionMaxSpeed)
                #toSay = 'full of food.'
                toSay = 'rempli de fromage!'
                textToSpeech.say(toSay)

                if naoStanding:
                    motionProxy.setAngles(names, resting_standing, fractionMaxSpeed)
                else:
                    motionProxy.setAngles(names, resting_sitting, fractionMaxSpeed)

                #+++++++++++++++++++++++++++++++++++++++++++++++

                #toSay = 'Being very hungry, she ate a great deal'
                toSay = 'Comme il y avait un petit trou dans le panier, la souris se faufile'
                textToSpeech.say(toSay)
                motionProxy.setStiffnesses("Body", 1.0)

                if naoStanding:
                    angles = [-0.07827591896057129, 0.029103994369506836, 0.49390602111816406, -0.3141592741012573, -0.7992560863494873, -1.5446163415908813, -0.9572582244873047, 0.7791999578475952, 4.1961669921875e-05, 4.1961669921875e-05, -0.4494199752807617, 0.6994619369506836, -0.3497939109802246, 4.1961669921875e-05, 4.1961669921875e-05, 4.1961669921875e-05, -0.44950389862060547, 0.7010800838470459, -0.34970998764038086, 4.1961669921875e-05, 1.3990497589111328, -0.1503739356994629, 1.2302260398864746, 0.6596620082855225, 0.1518239974975586, 0.8320000171661377]
                else:
                    angles = [-0.007711887359619141, 0.08125996589660645, 1.3851600885391235, 0.15642595291137695, -0.8038580417633057, -1.0461461544036865, 0.1257460117340088, 0.014799952507019043, -0.2469320297241211, -0.07512402534484863, -0.7025301456451416, 2.112546443939209, -1.1894419193267822, 0.07501578330993652, -0.2469320297241211, 0.07520794868469238, -0.7102839946746826, 2.112546443939209, -1.186300277709961, -0.07512402534484863, 0.4755818843841553, 0.2960200309753418, 0.6749181747436523, 1.5446163415908813, 1.0139319896697998, 0.0196000337600708]

                motionProxy.post.setAngles(names, angles, fractionMaxSpeed)
                #toSay = 'and went on eating and eating.'
                toSay = 'et commence à manger le fromage.'
                textToSpeech.say(toSay)

                if naoStanding:
                    motionProxy.setAngles(names, resting_standing, fractionMaxSpeed)
                else:
                    motionProxy.setAngles(names, resting_sitting, fractionMaxSpeed)

                #+++++++++++++++++++++++++++++++++++++++++++++++

                #toSay = 'She had grown very fat before she felt that'
                toSay = 'Comme elle avait vraiment très faim, la souris mange, mange, mange, et devient grosse,'
                textToSpeech.say(toSay)
                motionProxy.setStiffnesses("Body", 1.0)

                if naoStanding:
                    angles = [-0.007711887359619141, 0.018366098403930664, 0.970980167388916, 0.7516181468963623, -1.5079641342163086, -0.3573801517486572, -0.3666679859161377, 0.26200002431869507, 4.1961669921875e-05, 4.1961669921875e-05, -0.45095396041870117, 0.6979279518127441, -0.35132789611816406, 0.0015759468078613281, 4.1961669921875e-05, 4.1961669921875e-05, -0.4525718688964844, 0.6980118751525879, -0.35277795791625977, 4.1961669921875e-05, 0.7624399662017822, -0.5123980045318604, 1.6766201257705688, 0.5476799011230469, -0.07674193382263184, 0.2580000162124634]
                else:
                    angles = [-0.029187917709350586, 0.03063797950744629, 1.061486005783081, 0.19477605819702148, -0.17798590660095215, -1.4618600606918335, -1.0109481811523438, 0.012799978256225586, -0.2990880012512207, -0.056715965270996094, -0.667248010635376, 2.112546443939209, -1.1894419193267822, 0.07501578330993652, -0.2990880012512207, 0.038392066955566406, -0.678070068359375, 2.112546443939209, -1.186300277709961, -0.06131792068481445, 1.3376898765563965, -0.18719005584716797, 0.5061781406402588, 1.4481382369995117, 0.8160459995269775, 0.020400047302246094]

                motionProxy.post.setAngles(names, angles, fractionMaxSpeed)
                #toSay = 'she had had enough.'
                toSay = 'énorme même.'
                textToSpeech.say(toSay)

                if naoStanding:
                    motionProxy.setAngles(names, resting_standing, fractionMaxSpeed)
                else:
                    motionProxy.setAngles(names, resting_sitting, fractionMaxSpeed)

                #+++++++++++++++++++++++++++++++++++++++++++++++

                #toSay = 'When the mouse tried'
                toSay = 'Une fois rassasiée, elle voudrait sortir,'
                textToSpeech.say(toSay)

                #+++++++++++++++++++++++++++++++++++++++++++++++

                #toSay = 'She was too fat to pass through the hole...'
                toSay = "mais elle est devenue tellement grosse qu'elle ne peut plus passer par le petit trou"
                textToSpeech.say(toSay)
                motionProxy.setStiffnesses("Body", 1.0)

                if naoStanding:
                    angles = [-0.09208202362060547, -0.6719517707824707, -1.359166145324707, 0.1487560272216797, -0.7026140689849854, -0.36658406257629395, 0.49390602111816406, 0.7791999578475952, 4.1961669921875e-05, 4.1961669921875e-05, -0.4494199752807617, 0.6994619369506836, -0.35132789611816406, 4.1961669921875e-05, 4.1961669921875e-05, 4.1961669921875e-05, -0.4510378837585449, 0.6980118751525879, -0.3512439727783203, 0.0015759468078613281, -0.21011614799499512, -0.08901405334472656, 1.6167941093444824, 1.4910898208618164, -1.1244640350341797, 0.8320000171661377]
                else:
                    angles = [-0.0583338737487793, -0.6719517707824707, -0.6596620082855225, 0.14722204208374023, -0.8912959098815918, -0.7669579982757568, 0.6196939945220947, 0.012799978256225586, -0.29755401611328125, -0.05364799499511719, -0.6718499660491943, 2.112546443939209, -1.1894419193267822, 0.07501578330993652, -0.29755401611328125, 0.03992605209350586, -0.6811380386352539, 2.112546443939209, -1.186300277709961, -0.06438612937927246, -1.1075060367584229, -0.056799888610839844, 1.3222661018371582, 0.9173741340637207, 0.19937801361083984, 0.020400047302246094]


                motionProxy.post.setAngles(names, angles, fractionMaxSpeed)
                #toSay = 'How should I climb? said the mouse.'
                toSay = "et elle n'arrive pas non plus à escalader le panier! La souris crie a l'aide !"
                textToSpeech.say(toSay)

                if naoStanding:
                    motionProxy.setAngles(names, resting_standing, fractionMaxSpeed)
                else:
                    motionProxy.setAngles(names, resting_sitting, fractionMaxSpeed)

                #+++++++++++++++++++++++++++++++++++++++++++++++

                #toSay = 'Just then, a rat came along, and he heard the mouse. Mouse, said the rat.'
                toSay = 'Juste à ce moment là passait un rat. Il entend la souris...Souris, dit-il,'
                textToSpeech.say(toSay)
                motionProxy.setStiffnesses("Body", 1.0)

                if naoStanding:
                    angles = [-0.10128593444824219, -0.127363920211792, -0.28690004348754883, 0.8636000156402588, -0.05373191833496094, -0.28835010528564453, 0.43868207931518555, 0.7791999578475952, 4.1961669921875e-05, 4.1961669921875e-05, -0.4540219306945801, 0.6979279518127441, -0.35439586639404297, 4.1961669921875e-05, 4.1961669921875e-05, 4.1961669921875e-05, -0.4525718688964844, 0.6934099197387695, -0.35277795791625977, 4.1961669921875e-05, 1.3913798332214355, -0.4249598979949951, 0.3849921226501465, 0.9189081192016602, 0.01683211326599121, 0.8320000171661377]
                else:
                    angles = [-0.047595977783203125, -0.033789873123168945, -0.1104898452758789, 0.897348165512085, -0.8406739234924316, -0.6457719802856445, 0.8467259407043457, 0.8256000280380249, -0.2469320297241211, -0.07512402534484863, -0.6549761295318604, 2.112546443939209, -1.1894419193267822, 0.07501578330993652, -0.2469320297241211, 0.07367396354675293, -0.6627299785614014, 2.112546443939209, -1.186300277709961, -0.07586973160505295, 1.4051861763000488, -0.16571402549743652, 0.8114440441131592, 1.0232200622558594, -0.107421875, 0.020400047302246094]

                motionProxy.post.setAngles(names, angles, fractionMaxSpeed)
                #toSay = 'If you want to climb out of the basket...'
                toSay = 'si tu veux sortir, il va falloir attendre!'
                textToSpeech.say(toSay)

                if naoStanding:
                    motionProxy.setAngles(names, resting_standing, fractionMaxSpeed)
                else:
                    motionProxy.setAngles(names, resting_sitting, fractionMaxSpeed)

                #+++++++++++++++++++++++++++++++++++++++++++++++

                #toSay = 'you must wait till you have grown'
                toSay = "Attendre jusqu'à ce que tu sois"
                textToSpeech.say(toSay)
                motionProxy.setStiffnesses("Body", 1.0)

                if naoStanding:
                    angles = [-0.10128593444824219, -0.127363920211792, 1.142788052558899, -0.297637939453125, -1.4082541465759277, -1.0292720794677734, 0.0858621597290039, 0.7791999578475952, 4.1961669921875e-05, 4.1961669921875e-05, -0.4494199752807617, 0.6994619369506836, -0.35132789611816406, 4.1961669921875e-05, 4.1961669921875e-05, 4.1961669921875e-05, -0.4510378837585449, 0.6995458602905273, -0.34970998764038086, 4.1961669921875e-05, 1.1582121849060059, 0.2852821350097656, 1.351412057876587, 0.9986758232116699, -0.06907200813293457, 0.8320000171661377]
                else:
                    angles = [-0.004643917083740234, 0.08432793617248535, 1.2823820114135742, -0.3141592741012573, -1.4880218505859375, -1.0216021537780762, 0.17636799812316895, 0.8276000022888184, -0.24846601486206055, -0.07665801048278809, -0.7316761016845703, 2.112546443939209, -1.1894419193267822, 0.07501578330993652, -0.24846601486206055, 0.07213997840881348, -0.7424979209899902, 2.112546443939209, -1.186300277709961, -0.07512402534484863, 1.282465934753418, 0.3141592741012573, 1.4587920904159546, 0.9925398826599121, -0.03225588798522949, 0.8387999534606934]

                motionProxy.post.setAngles(names, angles, fractionMaxSpeed)
                #toSay = 'as thin as you were when you went\RSPD=100\ in!'
                toSay = 'redevenue toute fine! Coquin de rat!'
                textToSpeech.say(toSay)

                if naoStanding:
                    motionProxy.setAngles(names, resting_standing, fractionMaxSpeed)
                else:
                    motionProxy.setAngles(names, resting_sitting, fractionMaxSpeed)

                first_story = False

        else:

            #C'est l'histoire d'un chat qui avait un rêve bien curieux : il voulait manger la lune !
            #Toute les nuits, en se laichant les babines, il la contemplait, si flottante, si appétissante !
            #Mais le pauvre chat avait beau sauter aussi haut qu'il pouvait, de toutes ses forces, jamais il ne parvenait à attraper la lune.
            #Un beau jour, alors que les nuages cachaient le ciel nocturne, une vieille dame cuisinait des crêpes au beurre.
            #Afin d'en dorer les deux côtés, elle envoyait ses crêpe en l'air pour les faire pivoter.
            #Quand soudain, par maladresse, elle envoya une crêpe par la fenêtre.
            #Le chat, qui passait par là, vit la crêpe dans son vole, ronde et dorée, et la pris pour la lune.
            #Habilement il sauta, et toutes griffes sorties, il l'attrapa.
            #Enfin, la lune était à lui !
            #Goulument, il la dévora. Puis, un peu déçu, il se dit :
            #"Ma foi, je préfèrais les souris !"

            if naoSpeaking:
                trackFace()
                toSay = "\RSPD=60\ C'est l'histoire d'un chat qui avait un rêve bien curieux"
                textToSpeech.say(toSay)
                motionProxy.setStiffnesses("Body", 1.0)
                angles = []

                if naoStanding:
                    angles = [-0.0061779022216796875, 0.02143406867980957, -0.7992560863494873, 0.6319661140441895, -1.0631041526794434, -0.3650500774383545, 1.0599520206451416, 0.8023999929428101, 4.1961669921875e-05, 4.1961669921875e-05, -0.45095396041870117, 0.6994619369506836, -0.3497939109802246, 4.1961669921875e-05, 4.1961669921875e-05, 4.1961669921875e-05, -0.4510378837585449, 0.7026140689849854, -0.34970998764038086, 4.1961669921875e-05, 1.5555181503295898, -0.018450021743774414, 1.3682861328125, 0.8560140132904053, 0.03984212875366211, 0.258400022983551]
                else:
                    angles = [0.0014920234680175781, 0.5147144794464111, 1.406636118888855, 0.09966802597045898, -2.046398162841797, -0.27761197090148926, 0.970980167388916, 0.012799978256225586, -0.30062198638916016, -0.059783935546875, -0.6641800403594971, 2.112546443939209, -1.1894419193267822, 0.07501578330993652, -0.30062198638916016, 0.042994022369384766, -0.6734678745269775, 2.112546443939209, -1.186300277709961, -0.06438612937927246, 1.6337518692016602, -0.02611994743347168, 1.8269520998001099, 0.5553500652313232, -0.05373191833496094, 0.019999980926513672]

                motionProxy.post.setAngles(names, angles, fractionMaxSpeed)
                toSay = 'il voulait manger la lune!'
                textToSpeech.say(toSay)

                if naoStanding:
                    motionProxy.setAngles(names, resting_standing, fractionMaxSpeed)
                else:
                    motionProxy.setAngles(names, resting_sitting, fractionMaxSpeed)

                #+++++++++++++++++++++++++++++++++++++++++++++++
                toSay = 'Toute les nuits, en se léchant les babines, il la contemplait : si flottante !'
                textToSpeech.say(toSay)
                motionProxy.setStiffnesses("Body", 1.0)

                if naoStanding:
                    angles = [-0.0061779022216796875, 0.02143406867980957, 0.7132680416107178, 0.1088719367980957, 0.07665801048278809, -1.5446163415908813, -1.5141000747680664, 0.8023999929428101, 4.1961669921875e-05, 4.1961669921875e-05, -0.4494199752807617, 0.6994619369506836, -0.3497939109802246, 4.1961669921875e-05, 4.1961669921875e-05, 4.1961669921875e-05, -0.44950389862060547, 0.6980118751525879, -0.34970998764038086, 4.1961669921875e-05, 1.5846638679504395, 0.0014920234680175781, 1.3698201179504395, 0.8360719680786133, 0.03984212875366211, 0.258400022983551]
                else:
                    angles = [-0.004643917083740234, 0.08432793617248535, 1.2823820114135742, -0.3141592741012573, -1.4880218505859375, -1.0216021537780762, 0.17636799812316895, 0.8276000022888184, -0.24846601486206055, -0.07665801048278809, -0.7316761016845703, 2.112546443939209, -1.1894419193267822, 0.07501578330993652, -0.24846601486206055, 0.07213997840881348, -0.7424979209899902, 2.112546443939209, -1.186300277709961, -0.07512402534484863, 1.282465934753418, 0.3141592741012573, 1.4587920904159546, 0.9925398826599121, -0.03225588798522949, 0.8387999534606934]

                motionProxy.post.setAngles(names, angles, fractionMaxSpeed)
                toSay = 'si appétissante !'
                textToSpeech.say(toSay)

                if naoStanding:
                    motionProxy.setAngles(names, resting_standing, fractionMaxSpeed)
                else:
                    motionProxy.setAngles(names, resting_sitting, fractionMaxSpeed)
                #+++++++++++++++++++++++++++++++++++++++++++++++

                toSay = "Mais le pauvre chat avait beau sauter aussi haut qu'il pouvait"
                textToSpeech.say(toSay)
                motionProxy.setStiffnesses("Body", 1.0)

                if naoStanding:
                    angles = [-0.010779857635498047, 0.015298128128051758, -0.48325204849243164, 1.1918760538101196, -1.199629783630371, -1.5446163415908813, -1.3545641899108887, 0.2627999782562256, 4.1961669921875e-05, 4.1961669921875e-05, -0.4494199752807617, 0.7009961605072021, -0.3467259407043457, 4.1961669921875e-05, 4.1961669921875e-05, 0.0015759468078613281, -0.44950389862060547, 0.7026140689849854, -0.34970998764038086, 4.1961669921875e-05, 1.4420018196105957, -0.694943904876709, -4.1961669921875e-05, 1.5309739112854004, 0.01222991943359375, 0.258400022983551]
                else:
                    angles = [-0.004643917083740234, 0.08432793617248535, 1.2823820114135742, -0.3141592741012573, -1.4880218505859375, -1.0216021537780762, 0.17636799812316895, 0.8276000022888184, -0.24846601486206055, -0.07665801048278809, -0.7316761016845703, 2.112546443939209, -1.1894419193267822, 0.07501578330993652, -0.24846601486206055, 0.07213997840881348, -0.7424979209899902, 2.112546443939209, -1.186300277709961, -0.07512402534484863, 1.282465934753418, 0.3141592741012573, 1.4587920904159546, 0.9925398826599121, -0.03225588798522949, 0.8387999534606934]

                motionProxy.post.setAngles(names, angles, fractionMaxSpeed)
                toSay = 'de toutes ses forces, jamais il ne parvenait à attraper la lune.'
                textToSpeech.say(toSay)

                if naoStanding:
                    motionProxy.setAngles(names, resting_standing, fractionMaxSpeed)
                else:
                    motionProxy.setAngles(names, resting_sitting, fractionMaxSpeed)
                #+++++++++++++++++++++++++++++++++++++++++++++++

                toSay = "Un beau jour, alors que les nuages cachaient le ciel nocturne,"
                textToSpeech.say(toSay)
                motionProxy.setStiffnesses("Body", 1.0)

                if naoStanding:
                    angles = [-0.010779857635498047, 0.015298128128051758, 1.498676061630249, 0.06438612937927246, -1.1812219619750977, -0.7500841617584229, -0.0061779022216796875, 0.2627999782562256, 4.1961669921875e-05, 4.1961669921875e-05, -0.4494199752807617, 0.7009961605072021, -0.35132789611816406, 4.1961669921875e-05, 4.1961669921875e-05, 4.1961669921875e-05, -0.44950389862060547, 0.7010800838470459, -0.35277795791625977, 4.1961669921875e-05, 0.32524991035461426, 0.13648414611816406, 1.6704840660095215, 0.32831788063049316, 0.3711860179901123, 0.258400022983551]
                else:
                    angles = [-0.004643917083740234, 0.08432793617248535, 1.2823820114135742, -0.3141592741012573, -1.4880218505859375, -1.0216021537780762, 0.17636799812316895, 0.8276000022888184, -0.24846601486206055, -0.07665801048278809, -0.7316761016845703, 2.112546443939209, -1.1894419193267822, 0.07501578330993652, -0.24846601486206055, 0.07213997840881348, -0.7424979209899902, 2.112546443939209, -1.186300277709961, -0.07512402534484863, 1.282465934753418, 0.3141592741012573, 1.4587920904159546, 0.9925398826599121, -0.03225588798522949, 0.8387999534606934]

                motionProxy.post.setAngles(names, angles, fractionMaxSpeed)
                toSay = 'une vieille dame cuisinait des crêpes'
                textToSpeech.say(toSay)

                if naoStanding:
                    motionProxy.setAngles(names, resting_standing, fractionMaxSpeed)
                else:
                    motionProxy.setAngles(names, resting_sitting, fractionMaxSpeed)

                #+++++++++++++++++++++++++++++++++++++++++++++++

                toSay = "Afin d'en dorer les deux côtés, elle envoyait"
                textToSpeech.say(toSay)
                motionProxy.setStiffnesses("Body", 1.0)

                if naoStanding:
                    angles = [-0.010779857635498047, 0.015298128128051758, 1.498676061630249, 0.05824995040893555, -1.1812219619750977, -0.7362780570983887, -0.004643917083740234, 0.2627999782562256, 4.1961669921875e-05, 4.1961669921875e-05, -0.4494199752807617, 0.7009961605072021, -0.35132789611816406, 4.1961669921875e-05, 4.1961669921875e-05, 4.1961669921875e-05, -0.44950389862060547, 0.7010800838470459, -0.3512439727783203, 4.1961669921875e-05, -1.0737581253051758, 0.17330002784729004, 1.6658821105957031, 0.4939899444580078, 0.1978440284729004, 0.7955999970436096]
                else:
                    angles = [-0.004643917083740234, 0.08432793617248535, 1.2823820114135742, -0.3141592741012573, -1.4880218505859375, -1.0216021537780762, 0.17636799812316895, 0.8276000022888184, -0.24846601486206055, -0.07665801048278809, -0.7316761016845703, 2.112546443939209, -1.1894419193267822, 0.07501578330993652, -0.24846601486206055, 0.07213997840881348, -0.7424979209899902, 2.112546443939209, -1.186300277709961, -0.07512402534484863, 1.282465934753418, 0.3141592741012573, 1.4587920904159546, 0.9925398826599121, -0.03225588798522949, 0.8387999534606934]

                motionProxy.post.setAngles(names, angles, fractionMaxSpeed)
                toSay = "ses crêpe en l'air pour les faire pivoter."
                textToSpeech.say(toSay)

                if naoStanding:
                    motionProxy.setAngles(names, resting_standing, fractionMaxSpeed)
                else:
                    motionProxy.setAngles(names, resting_sitting, fractionMaxSpeed)
                #+++++++++++++++++++++++++++++++++++++++++++++++

                toSay = "Quand soudain, par maladresse,"
                textToSpeech.say(toSay)
                motionProxy.setStiffnesses("Body", 1.0)

                if naoStanding:
                    angles = [-0.0061779022216796875, 0.02143406867980957, 0.6365680694580078, -0.3141592741012573, -0.49552392959594727, -0.7730941772460938, -1.07230806350708, 0.8023999929428101, 4.1961669921875e-05, 4.1961669921875e-05, -0.4494199752807617, 0.6979279518127441, -0.35132789611816406, 4.1961669921875e-05, 4.1961669921875e-05, 4.1961669921875e-05, -0.4510378837585449, 0.7010800838470459, -0.34970998764038086, 4.1961669921875e-05, 0.19792795181274414, -0.7946538925170898, 2.0856685638427734, 0.8238000869750977, -0.038392066955566406, 0.373199999332428]
                else:
                    angles = [-0.004643917083740234, 0.08432793617248535, 1.2823820114135742, -0.3141592741012573, -1.4880218505859375, -1.0216021537780762, 0.17636799812316895, 0.8276000022888184, -0.24846601486206055, -0.07665801048278809, -0.7316761016845703, 2.112546443939209, -1.1894419193267822, 0.07501578330993652, -0.24846601486206055, 0.07213997840881348, -0.7424979209899902, 2.112546443939209, -1.186300277709961, -0.07512402534484863, 1.282465934753418, 0.3141592741012573, 1.4587920904159546, 0.9925398826599121, -0.03225588798522949, 0.8387999534606934]

                motionProxy.post.setAngles(names, angles, fractionMaxSpeed)
                toSay = 'elle envoya une crêpe par la fenêtre.'
                textToSpeech.say(toSay)

                if naoStanding:
                    motionProxy.setAngles(names, resting_standing, fractionMaxSpeed)
                else:
                    motionProxy.setAngles(names, resting_sitting, fractionMaxSpeed)

                #+++++++++++++++++++++++++++++++++++++++++++++++

                toSay = "Le chat, qui passait par là, vit la crêpe dans son vol, ronde et dorée, et la pris pour la lune. Habilement il sauta,"
                textToSpeech.say(toSay)
                motionProxy.setStiffnesses("Body", 1.0)

                if naoStanding:
                    angles = [-0.007711887359619141, 0.02143406867980957, 0.13341593742370605, -0.2055978775024414, -1.6153440475463867, -1.4741320610046387, 1.7211060523986816, 0.8023999929428101, 4.1961669921875e-05, 4.1961669921875e-05, -0.45095396041870117, 0.6979279518127441, -0.35132789611816406, 4.1961669921875e-05, 4.1961669921875e-05, 4.1961669921875e-05, -0.4510378837585449, 0.6995458602905273, -0.3543119430541992, 4.1961669921875e-05, 0.1825878620147705, 0.23926210403442383, 1.630600094795227, 1.4880218505859375, -1.639887809753418, 0.6991999745368958]
                else:
                    angles = [-0.004643917083740234, 0.08432793617248535, 1.2823820114135742, -0.3141592741012573, -1.4880218505859375, -1.0216021537780762, 0.17636799812316895, 0.8276000022888184, -0.24846601486206055, -0.07665801048278809, -0.7316761016845703, 2.112546443939209, -1.1894419193267822, 0.07501578330993652, -0.24846601486206055, 0.07213997840881348, -0.7424979209899902, 2.112546443939209, -1.186300277709961, -0.07512402534484863, 1.282465934753418, 0.3141592741012573, 1.4587920904159546, 0.9925398826599121, -0.03225588798522949, 0.8387999534606934]

                motionProxy.post.setAngles(names, angles, fractionMaxSpeed)
                toSay = "et toutes griffes sorties,"
                textToSpeech.say(toSay)

                if naoStanding:
                    motionProxy.setAngles(names, resting_standing, fractionMaxSpeed)
                else:
                    motionProxy.setAngles(names, resting_sitting, fractionMaxSpeed)
                #+++++++++++++++++++++++++++++++++++++++++++++++

                toSay = "il l'attrapa. Enfin, la lune était à lui !"
                textToSpeech.say(toSay)
                motionProxy.setStiffnesses("Body", 1.0)

                if naoStanding:
                    angles = [-0.010779857635498047, 0.015298128128051758, 0.3128941059112549, -0.3141592741012573, -0.9173741340637207, -1.5446163415908813, -1.1766200065612793, 0.2627999782562256, 4.1961669921875e-05, 4.1961669921875e-05, -0.4494199752807617, 0.6994619369506836, -0.35132789611816406, 0.0015759468078613281, 4.1961669921875e-05, 4.1961669921875e-05, -0.44950389862060547, 0.6995458602905273, -0.34970998764038086, 4.1961669921875e-05, 1.4542741775512695, -0.23627805709838867, 0.7592880725860596, 0.9219760894775391, 0.9234261512756348, 0.795199990272522]
                else:
                    angles = [-0.004643917083740234, 0.08432793617248535, 1.2823820114135742, -0.3141592741012573, -1.4880218505859375, -1.0216021537780762, 0.17636799812316895, 0.8276000022888184, -0.24846601486206055, -0.07665801048278809, -0.7316761016845703, 2.112546443939209, -1.1894419193267822, 0.07501578330993652, -0.24846601486206055, 0.07213997840881348, -0.7424979209899902, 2.112546443939209, -1.186300277709961, -0.07512402534484863, 1.282465934753418, 0.3141592741012573, 1.4587920904159546, 0.9925398826599121, -0.03225588798522949, 0.8387999534606934]

                motionProxy.post.setAngles(names, angles, fractionMaxSpeed)
                toSay = "Goulument, il la dévora."
                textToSpeech.say(toSay)

                if naoStanding:
                    motionProxy.setAngles(names, resting_standing, fractionMaxSpeed)
                else:
                    motionProxy.setAngles(names, resting_sitting, fractionMaxSpeed)

                #+++++++++++++++++++++++++++++++++++++++++++++++

                toSay = 'Puis, un peu déçu, il se dit : Ma foi, je préfère les souris !'
                textToSpeech.say(toSay)
                motionProxy.setStiffnesses("Body", 1.0)

                if naoStanding:
                    motionProxy.setAngles(names, resting_standing, fractionMaxSpeed)
                else:
                    motionProxy.setAngles(names, resting_sitting, fractionMaxSpeed)

        tts.resetSpeed()

        # Stop tracker.
        tracker.stopTracker()
        tracker.unregisterAllTargets()

        rospy.sleep(2)
        # Switch to learning words
        msg = String()
        msg.data = "learning_words_nao"
        pub_activity.publish(msg)

        #textToSpeech.setLanguage("English")
        changeActivityReceived = " "
        nextState = "PAUSE_INTERACTION"
        infoForNextState = {'state_cameFrom': "DRAWING"}

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

    stateMachine = StateMachine()
    stateMachine.add_state("STARTING_INTERACTION", startInteraction)
    stateMachine.add_state("WAITING_FOR_ROBOT_TO_CONNECT", waitForRobotToConnect)
    stateMachine.add_state("PAUSE_INTERACTION", pauseInteraction)
    stateMachine.add_state("WAITING_FOR_TABLET_TO_CONNECT", waitForTabletToConnect)
    stateMachine.add_state("DRAWING", drawing)
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

    screenManager = ScreenManager(0.2, 0.1395)

    stateMachine.run(infoForStartState)

    rospy.spin()

    tabletWatchdog.stop()
