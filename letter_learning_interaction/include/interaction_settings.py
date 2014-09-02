#!/usr/bin/env python
# coding: utf-8

"""
Manages the settings used for the learning_words_nao.py node. 
"""

from shape_learning.shape_learner import SettingsStruct
import numpy
from enum import Enum 
class LearningModes(Enum):
        startsGood = 0;
        startsBad = 1;
        startsRandom = 2;
        
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
datasetDirectory = None;
class InteractionSettings():
    @staticmethod
    def getTrajectoryTimings(naoWriting):
        if(naoWriting):
            t0 = 3;                 #Time allowed for the first point in traj (seconds) (@todo should be proportional to distance)
            dt = 0.25               #Seconds between points in traj
            delayBeforeExecuting = 3;#How far in future to request the traj be executed (to account for transmission delays and preparedness)
        else:
            t0 = 0.01;
            dt = 0.1;
            delayBeforeExecuting = 2.5;
        return t0, dt, delayBeforeExecuting

    ###---------------------------------------------- NAO HEAD ANGLES FOR LOOKING
    @staticmethod
    def getHeadAngles():
        #["HeadYaw", "HeadPitch"] angles
        headAngles_lookAtTablet_right = [-0.2, 0.08125996589660645];
        headAngles_lookAtTablet_left = [0.2, 0.08125996589660645];
        headAngles_lookAtPerson_right = [-0.9639739513397217, 0.08125996589660645];
        headAngles_lookAtPerson_left = [0.9639739513397217, 0.08125996589660645];
        return headAngles_lookAtTablet_right, headAngles_lookAtTablet_left, headAngles_lookAtPerson_right, headAngles_lookAtPerson_left

    ###---------------------------------------------- WORD LEARNING PHRASES
    @staticmethod
    def getPhrases(language):
        if(language.lower()=='english'):
            introPhrase = "Hello. I'm Nao. Please show me a word to practice.";
            demo_response_phrases = ["Okay, I'll try it like you", "So that's how you write %s", "That's a much better %s than mine", "I'll try to copy you","Let me try now","Thank you"];
            asking_phrases_after_feedback = ["Any better?", "How about now?", "Now what do you think?","Is there a difference?", "Is this one okay?", "Will you show me how?", "Did I improve?"];
            asking_phrases_after_word = ["Okay, what do you think?", "This is a hard word", "Is this how you write it?","Please help me"];
            word_response_phrases = ["%s, okay. ", "%s seems like a good word", "Hopefully I can do well with this word", "%s. Let's try", "Okay, %s now"];
            word_again_response_phrases = ["%s again, okay.", "I thought I already did %s", "You like to practice this word"];
            testPhrase = "Ok, test time. I'll try my best.";
            thankYouPhrase = 'Thank you for your help.';
        elif(language.lower()=='french'):
            introPhrase = "Bonjour, je m'appelle Nao. Peux-tu me montrer un mot ?";
            demo_response_phrases = ["D'accord, j'essaye comme ça", "Ah, c'est comme ça qu'on écrit %s", "Ce %s est pas mal", "Bon, j'essaye comme toi", "Ok, à moi", "À mon tour", "Merci, je vais essayer"];
            asking_phrases_after_feedback = ["C'est mieux ?", "Et comme ça ?", "Tu en penses quoi ?", "Qu'est-ce que tu en penses ?", "Il y a une différence ?", "Ça va cette fois ?", "Je me suis amélioré ?", "Tu trouves que c'est mieux ?"];
            asking_phrases_after_word = ["Bon, qu'est ce que tu en penses ?", "Pas facile !", "C'est bien comme ça ?", "Je crois que j'ai besoin d'aide.", "Et voilà !"];
            word_response_phrases = ["D'accord pour %s", "Ok, j'essaye %s", "Bon, je devrais y arriver", "D'accord", "%s ? ok"];
            word_again_response_phrases = ["Encore %s ? bon, d'accord.", "Je crois que j'ai déjà fait %s", "On dirait que tu aimes bien %s !", "Encore ?"];
            testPhrase = "Ok, c'est l'heure du test. J'ai un peu peur."
            thankYouPhrase = "Merci pour ton aide !";
        else:
            RuntimeError('Requested language ('+ langauge + ') not supported');
        return introPhrase, demo_response_phrases, asking_phrases_after_feedback, asking_phrases_after_word, word_response_phrases, word_again_response_phrases, testPhrase, thankYouPhrase
        
    @staticmethod
    def setDatasetDirectory(datasetDirectory_):
        global datasetDirectory
        datasetDirectory = datasetDirectory_;
         
    ###---------------------------------------------- WORD LEARNING SETTINGS
    @staticmethod
    def generateSettings(shapeType):
        if(datasetDirectory is None):
            raise RuntimeError("Dataset directory has not been set yet with setDatasetDirectory()");
    
        paramsToVary = [2];            #Natural number between 1 and numPrincipleComponents, representing which principle component to vary from the template
        initialBounds_stdDevMultiples = numpy.array([[-6, 6]]);  #Starting bounds for paramToVary, as multiples of the parameter's observed standard deviation in the dataset
        doGroupwiseComparison = True; #instead of pairwise comparison with most recent two shapes
        initialParamValue = numpy.NaN;
        initialBounds = numpy.array([[numpy.NaN, numpy.NaN]]);
        
        if shapeType == 'a':
            paramsToVary = [6];
            initialBounds_stdDevMultiples = numpy.array([[-3, 3]]);
            datasetFile = datasetDirectory + '/a_noHook_dataset.txt';
            initialParamValue = 0.8; 
        elif shapeType == 'c':
            paramToVary = 4;
            initialBounds_stdDevMultiples = numpy.array([[-10, 10]]);
            datasetFile = datasetDirectory + '/c_dataset.txt';
            
            if(learningModes[shapeType] == LearningModes.startsRandom):
                initialBounds_stdDevMultiples = numpy.array([[-10, 10]]);
            elif(learningModes[shapeType] == LearningModes.startsGood):
                initialBounds_stdDevMultiples = numpy.array([[-4, -3]]);
                initialParamValue = -0.5;
            elif(learningModes[shapeType] == LearningModes.startsBad):
                initialBounds_stdDevMultiples = numpy.array([[1.5, 10]]);
                initialParamValue = 0.5;
        elif shapeType == 'd':
            datasetFile = datasetDirectory + '/d_cursive_dataset.txt';
        elif shapeType == 'e':
            paramToVary = 3; 
            initialBounds_stdDevMultiples = numpy.array([[-6, 14]]);
            datasetFile = datasetDirectory + '/e_dataset.txt';
            #initialParamValue = 0.8;
        elif shapeType == 'm':
            paramToVary = 6; 
            initialBounds_stdDevMultiples = numpy.array([[-10, -6]]);
            datasetFile = datasetDirectory + '/m_dataset.txt';
            initialParamValue = -0.5;#0.0;
        elif shapeType == 'n':
            paramToVary = 7; 
            datasetFile = datasetDirectory + '/n_dataset.txt';
            initialParamValue = 0.0;
        elif shapeType == 'o':
            paramsToVary = [4];
            initialBounds_stdDevMultiples = numpy.array([[-3.5, 3]]);
            datasetFile = datasetDirectory + '/o_dataset.txt';
        elif shapeType == 'r':
            paramToVary = 1;
            datasetFile = datasetDirectory + '/r_print_dataset.txt';
        elif shapeType == 's':
            datasetFile = datasetDirectory + '/s_print_dataset.txt';
        elif shapeType == 'u':
            paramsToVary = [3];
            datasetFile = datasetDirectory + '/u_dataset.txt';
        elif shapeType == 'v':
            paramToVary = 6;
            datasetFile = datasetDirectory + '/v_dataset.txt';
        elif shapeType == 'w':
            datasetFile = datasetDirectory + '/w_dataset.txt';
        else:
            raise RuntimeError("Dataset is not known for shape "+ shapeType);
        settings = SettingsStruct(shape_learning = shapeType,
        paramsToVary = paramsToVary, doGroupwiseComparison = True, 
        datasetFile = datasetFile, initialBounds = initialBounds, 
        initialBounds_stdDevMultiples = initialBounds_stdDevMultiples,
        initialParamValue = initialParamValue, minParamDiff = 0.4);
        return settings

