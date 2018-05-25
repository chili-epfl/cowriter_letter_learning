#!/usr/bin/env python2
# coding: utf-8"""

Created on Sun Mar 22 11:29:31 2015

@author: ferran
"""
from naoqi import ALBroker, ALProxy

class ConnexionToNao():
    @staticmethod
    def setConnexion(naoConnected, naoWriting, naoStanding, NAO_IP, LANGUAGE, effector):
        
        if naoConnected:
            port = 9559
            myBroker = ALBroker("myBroker", #I'm not sure that pyrobots doesn't already have one of these open called NAOqi?
                    "0.0.0.0",   # listen to anyone
                    0,           # find a free port and use it
                    NAO_IP,      # parent broker IP
                    port)        # parent broker port
            motionProxy = ALProxy("ALMotion", NAO_IP, port)
        
            postureProxy = ALProxy("ALRobotPosture", NAO_IP, port)
            textToSpeech = ALProxy("ALTextToSpeech", NAO_IP, port)   
            textToSpeech.setLanguage(LANGUAGE.capitalize())
            #textToSpeech.setVolume(1.0)
            if naoWriting:
                if naoStanding:
                    #postureProxy.goToPosture("StandInit",0.2)
                    motionProxy.wbEnableEffectorControl(effector, True) #turn whole body motion control on
                else:
                    motionProxy.rest()
                    motionProxy.setStiffnesses(["Head", "LArm", "RArm"], 0.5)
                    motionProxy.setStiffnesses(["LHipYawPitch", "LHipRoll", "LHipPitch", "RHipYawPitch", "RHipRoll", "RHipPitch"], 0.8)
                    motionProxy.wbEnableEffectorControl(effector, False) #turn whole body motion control off
                    
                armJoints_standInit = motionProxy.getAngles(effector,True)
                   
        return myBroker, postureProxy, motionProxy, textToSpeech, armJoints_standInit
