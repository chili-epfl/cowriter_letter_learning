#!/usr/bin/env python
# coding: utf-8

"""
Manages the settings used for the learning_words_nao.py node. 
"""
from enum import Enum 
import os.path

import numpy

from shape_learning.shape_learner import SettingsStruct

class LearningModes(Enum):
        startsGood = 0
        startsBad = 1
        startsRandom = 2
        
#define the learning mode for each shape we expect to see
learningModes = {'c': LearningModes.startsRandom,
                'e': LearningModes.startsRandom,
                'm': LearningModes.startsRandom,
                'n': LearningModes.startsRandom,
                'o': LearningModes.startsRandom,
                's': LearningModes.startsRandom,
                'u': LearningModes.startsRandom,
                'w': LearningModes.startsRandom}; 

global datasetDirectory
datasetDirectory = None
class InteractionSettings():
    @staticmethod
    def getTrajectoryTimings(naoWriting):
        if(naoWriting):
            t0 = 3;                 #Time allowed for the first point in traj (seconds) (@todo should be proportional to distance)
            dt = 0.25               #Seconds between points in traj
            delayBeforeExecuting = 3;#How far in future to request the traj be executed (to account for transmission delays and preparedness)
        else:
            t0 = 0.01
            dt = 0.1
            delayBeforeExecuting = 2.5
        return t0, dt, delayBeforeExecuting

    ###---------------------------------------------- NAO HEAD ANGLES FOR LOOKING
    @staticmethod
    def getHeadAngles():
        #["HeadYaw", "HeadPitch"] angles
        headAngles_lookAtTablet_down = [-0.01538, 0.512]
        headAngles_lookAtTablet_right = [-0.2, 0.08125996589660645]
        headAngles_lookAtTablet_left = [0.2, 0.08125996589660645]
        headAngles_lookAtPerson_front = [-0.0123, -0.05]
        headAngles_lookAtPerson_right = [-0.9639739513397217, 0.08125996589660645]
        headAngles_lookAtPerson_left = [0.9639739513397217, 0.08125996589660645]
        return  headAngles_lookAtTablet_down,headAngles_lookAtTablet_right, headAngles_lookAtTablet_left, headAngles_lookAtPerson_front ,headAngles_lookAtPerson_right, headAngles_lookAtPerson_left

    ###---------------------------------------------- WORD LEARNING PHRASES
    @staticmethod
    def getPhrases(language):
        if(language.lower()=='english'):
            introPhrase = "Hello. I'm Me me."
            testPhrase = "Ok, test time. I'll try my best."
            thankYouPhrase = 'I was a good training! Thank you... I hope I will see you again'
            introLearningWordsPhrase = "I want to practice my writing... Can you help me?"
            introDrawingPhrase = 'I feel tired... I would like to practice my french... Can I tell you a story in french.'
            
            againLearningWordsPhrase = 'I would like to play with the words again...'
            againDrawingPhrase = 'I know another good story. I will tell you...'
            introJokePhrase = 'All right... Can you also show me how to write?'
            againJokePhrase = 'I will tell you the joke again.'
            
            #The following phrases may have an optional string formatted into them
            demo_response_phrases = ["Okay, I'll try it like you", "So that's how you write %s", "That's a much better %s than mine", "I'll try to copy you","Let me try now","Thank you"]
            asking_phrases_after_feedback = ["Any better?", "How about now?", "Now what do you think?","Is there a difference?", "Is this one okay?", "Will you show me how?", "Did I improve?"]
            asking_phrases_after_word = ["Okay, what do you think?", "This is a hard word", "Is this how you write it?","Please help me"]
            word_response_phrases = ["Let's try the word %s", "%s seems like a good word to write", "Hopefully I can do well with this word", "Okay, %s now"]
            word_again_response_phrases = ["Let's try %s again.", "I like to practice this word"]
            
        elif(language.lower()=='french'):
            introPhrase = "Bonjour, je suis Mimi. Je voudrais apprendre à dessiner des chiffres." 
            testPhrase = "Ok, c'est l'heure du test. J'ai un peu peur."
            thankYouPhrase = "Merci pour ton aide, j'ai beaucoup appris grâce à toi... J'éspère qu'on se reverra !"
            introLearningWordsPhrase = "J'aimerai m'entrainer à l'écriture... Peux-tu m'aider ?"
            introDrawingPhrase = 'Je suis un peu fatigué... Je connais une histoire sympa. Je vais te la raconter... '
            
            againLearningWordsPhrase = 'Je voudrais apprendre à écrire encore un petit peu...'
            againDrawingPhrase = 'Je connais une autre histoire sympa. Je vais te la raconter...'
            introJokePhrase = 'All right... Can you also show me how to write?'
            againJokePhrase = 'I will tell you the joke again.'
            
            #The following phrases may have an optional string formatted into them
            demo_response_phrases = ["Ok", "D'accord, j'essaye comme ça", "Ah, c'est comme ça qu'on écrit %s", "Bon", "Ce %s est pas mal", "Bon, j'essaye comme toi", "D'accord", "Ok, à moi", "À mon tour", "Ok", "Merci, je vais essayer"]
            asking_phrases_after_feedback = ["C'est mieux ?", "Voilà", "Et comme ça ?", "Tu en penses quoi ?", "Alors ?", "Qu'est-ce que tu en penses ?", "Il y a une différence ?", "Ça va cette fois ?", "Je me suis amélioré ?", "Tu trouves que c'est mieux ?"]
            asking_phrases_after_word = ["Bon, qu'est ce que tu en penses ?", "Pas facile !", "Voilà", "C'est bien comme ça ?", "Bon", "Je crois que j'ai besoin d'aide.", "Et voilà !"]
            word_response_phrases = ["D'accord pour %s", "Ok, j'essaye %s", "Bon, je devrais y arriver", "D'accord", "%s ? ok"]
            word_again_response_phrases = ["Encore %s ? bon, d'accord.", "Je crois que j'ai déjà fait %s", "On dirait que tu aimes bien %s !", "Encore ?"]
            
        else:
            RuntimeError('Requested language ('+ language + ') not supported')
        return introPhrase, demo_response_phrases, asking_phrases_after_feedback, asking_phrases_after_word, word_response_phrases, word_again_response_phrases, testPhrase, thankYouPhrase, introLearningWordsPhrase, introDrawingPhrase, againLearningWordsPhrase, againDrawingPhrase, introJokePhrase, againJokePhrase
        
    @staticmethod
    def setDatasetDirectory(datasetDirectory_):
        global datasetDirectory
        datasetDirectory = datasetDirectory_
         
    ###---------------------------------------------- WORD LEARNING SETTINGS
    @staticmethod
    def generateSettings(shapeType):
        if(datasetDirectory is None):
            raise RuntimeError("Dataset directory has not been set yet with setDatasetDirectory()")

        paramsToVary = [3];            #Natural number between 1 and numPrincipleComponents, representing which principle component to vary from the template
        initialBounds_stdDevMultiples = numpy.array([[-6, 6]]) #Starting bounds for paramToVary, as multiples of the parameter's observed standard deviation in the dataset
        doGroupwiseComparison = True #instead of pairwise comparison with most recent two shapes
        initialParamValue = 0.0
        initialBounds = numpy.array([[numpy.NaN, numpy.NaN]])

        datasetFile = datasetDirectory + '/' + shapeType + '.dat'
        if not os.path.exists(datasetFile):
            raise RuntimeError("Dataset is not known for shape "+ shapeType)

        datasetParam = datasetDirectory + '/params.dat'
        if not os.path.exists(datasetParam):
            raise RuntimeError("parameters not found for this dataset ")
        else:
            with open(datasetParam, 'r') as f:
                initialParamValue = [0.0] * 20 #HACK: this (default) value should be independant from the number of params. Smthg like np.NaN would be better
                for line in f.readline():
                    if line[1:-2] == shapeType: # lines starting with [<letter>]\n
                        initialParamValue = [float(s) for s in f.readline().split(",")]
                        break

        settings = SettingsStruct(
                    shape_learning = shapeType,
                    paramsToVary = paramsToVary, 
                    doGroupwiseComparison = True, 
                    initDatasetFile = datasetFile, 
                    updateDatasetFiles = None, 
                    initialBounds = initialBounds, 
                    initialBounds_stdDevMultiples = initialBounds_stdDevMultiples,
                    initialParamValue = initialParamValue, 
                    paramFile = None, # used to store new updated params after demonstration
                    minParamDiff = 0.4)
        return settings

