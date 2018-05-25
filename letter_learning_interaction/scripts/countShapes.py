#!/usr/bin/env python
'''
Count things received from rosbags of nao_ros_cowriter experiment.
'''

import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Empty, String
SHAPES_TOPIC = "/user_shapes"
LETTER_TOPIC = 'write_traj'
WORDS_TOPIC = 'words_to_write';
TEST_TOPIC = 'test_learning';
STOP_TOPIC = 'stop_learning';

time_firstEvent = None;
userShapes = [];
strokes = []
numUserShapes=0;
numStrokes = 0;
numCorrectionsReceivedForThisWord = 0;
interactionFinished = False;
def on_user_traj(message):
    global numStrokes, numCorrectionsReceivedForThisWord,time_firstEvent, strokes, userShapes
    if(time_firstEvent is None):
        time_firstEvent = rospy.Time.now();

    if(len(message.poses)==0 and numStrokes > 0): #shape finished
        global numUserShapes
        numUserShapes+=1;

        numStrokes = 0;
        strokes = [];
        numCorrectionsReceivedForThisWord += 1;
        userShapes.append(strokes);
    elif(len(message.poses)>0 and not interactionFinished):
        stroke = [];
        for point in message.poses:
            stroke.extend([point.pose.position.x, point.pose.position.y]);

        strokeString = ','.join(map(str, stroke))
        writer_letters.writerow([str(numUserShapes) + ',' + str(numStrokes) + ',' +strokeString]);
        numStrokes += 1;
        writer.writerow(['Word','Number of corrections received', 'Number of corrections responded to', 'Number of corrections ignored']);

numRobotShapes=0
numCorrectionsRespondedToForThisWord = -1; #don't count the first drawing as a 'correction'
def on_robot_traj(message):
    global numRobotShapes, numCorrectionsRespondedToForThisWord,time_firstEvent
    if(time_firstEvent is None):
        time_firstEvent = rospy.Time.now();
    numRobotShapes+=1;
    numCorrectionsRespondedToForThisWord += 1;

numWords = 0;
numCorrectionsIgnored = 0;
prevWord = None;
def on_word(message):
    global numWords, numCorrectionsReceivedForThisWord, numCorrectionsRespondedToForThisWord, numCorrectionsIgnored, prevWord,time_firstEvent
    if(time_firstEvent is None):
        time_firstEvent = rospy.Time.now();
    if(message.data != 'end'):
        numWords += 1;
    #if(prevWord is not None):
    if(True):
        numCorrectionsIgnoredForThisWord = numCorrectionsReceivedForThisWord - numCorrectionsRespondedToForThisWord;
        print('Number of corrections received for previous word: ' + str(numCorrectionsReceivedForThisWord))
        print('Number of corrections responded to for previous word: '+str(numCorrectionsRespondedToForThisWord))
        print('Number of corrections ignored: ' + str(numCorrectionsIgnoredForThisWord))
        numCorrectionsIgnored += numCorrectionsIgnoredForThisWord;
        writer.writerow([prevWord,str(numCorrectionsReceivedForThisWord),str(numCorrectionsRespondedToForThisWord),str(numCorrectionsIgnoredForThisWord)]);
        numCorrectionsRespondedToForThisWord = -1;
        numCorrectionsReceivedForThisWord = 0;
    else:
        writer.writerow(['Word','Number of corrections received', 'Number of corrections responded to', 'Number of corrections ignored']);
    print('-------------------Number of word requests received: ' + str(numWords) + ' ('+message.data+')');
    prevWord = message.data;

numWordsBeforeTest = 0;
def onTestRequestReceived(message):
    global numWordsBeforeTest
    print('Number of words before test: '+str(numWords));
    numWordsBeforeTest = numWords;

def onStopRequestReceived(message):
    global interactionFinished
    interactionFinished = True;
    message = String()
    message.data = 'end'
    on_word(message);
    print('Total number of user-drawn shapes: '+str(numUserShapes));
    print('Total number of robot-drawn shape messages: '+str(numRobotShapes));
    print('Total number of corrections ignored: '+str(numCorrectionsIgnored));
    print('Total number of words: '+str(numWords));
    time_stop = rospy.Time.now();
    duration = time_stop - time_firstEvent;
    print('Total time: '+str(duration.to_sec()))


    #writer.writerow(['Total number of robot-drawn shape messages',str(numRobotShapes)]);
    writer.writerow(['Total number of words', str(numWords)]);
    writer.writerow(['Number of words before test', str(numWordsBeforeTest)]);
    writer.writerow(['Total number of robot-drawn letters',str(numRobotShapes+2*numWords)]); #three letter words are sent as one message\
    writer.writerow(['Total number of user-drawn shapes',str(numUserShapes)]);
    writer.writerow(['Total number of user-drawn shapes responded to',str(numUserShapes-numCorrectionsIgnored)]);
    writer.writerow(['Total number of corrections ignored', str(numCorrectionsIgnored)]);


args = None

if __name__ == "__main__":
    #parse arguments
    import argparse
    parser = argparse.ArgumentParser(description='Record results of rosbag');
    parser.add_argument('output', action="store",
                    help='a string containing the name of csv file to write to');
    parser.add_argument('output_letters', action="store",
                    help='a string containing the name of csv file to write letter strokes to');
    global args
    args = parser.parse_args();

    import csv
    csvfile = open(args.output,'wb')
    writer = csv.writer(csvfile, delimiter=' ',
                                #quotechar='|',
                                quoting=csv.QUOTE_MINIMAL)

    csvfile_letters = open(args.output_letters,'wb')
    writer_letters = csv.writer(csvfile_letters, delimiter=' ',
                                #quotechar='',
                                quoting=csv.QUOTE_MINIMAL)


    rospy.init_node('shape_counter')
    user_traj = rospy.Subscriber(SHAPES_TOPIC, Path, on_user_traj)
    robot_traj = rospy.Subscriber(LETTER_TOPIC, Path, on_robot_traj)
    words_subscriber = rospy.Subscriber(WORDS_TOPIC, String, on_word);
    test_subscriber = rospy.Subscriber(TEST_TOPIC, Empty, onTestRequestReceived);
    stop_subscriber = rospy.Subscriber(STOP_TOPIC, Empty, onStopRequestReceived);
    print('Waiting for rosbag to start')

    rospy.spin()
    csvfile.close()
    csvfile_letters.close()
