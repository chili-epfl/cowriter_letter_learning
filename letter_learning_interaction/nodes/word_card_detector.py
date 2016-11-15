#!/usr/bin/env python
'''
Publish word to write based on fiducial marker (chilitag) detected.
'''

import rospy
import tf
from std_msgs.msg import String, Empty
import operator

special_tags = {'tag_17':'test',
                'tag_302':'stop',
                'tag_300':'next',
                'tag_301':'prev',
                'tag_430':'help',
                'tag_341':'go'
                }

tags_words_mapping = {'tag_5':'cow',
                      'tag_6':'son',
                      'tag_7':'cue',
                      'tag_8':'new',
                      'tag_9':'use',
                      'tag_10':'cou',
                      'tag_11':'son',
                      'tag_12':'ces',
                      'tag_13':'une', # no tag_14: too many false positives!
                      'tag_15':'nos',
                      'tag_16':'ose',
                      'tag_18':'eau'
                      }

tags_letters_mapping = {}

#tags_words_mapping.update(special_tags)

# Add individual letters: tag IDs are the ASCII code of the letter
for char in range(ord('a'),ord('z') + 1):
    tags_letters_mapping["tag_%d" % char] = chr(char)

def cmp(l1, l2):
    _, x1 = l1
    _, x2 = l2
    return 1 if x1 < x2 else -1

def last_seen_since(frame):
    try:
        return (rospy.Time().now() - tf_listener.getLatestCommonTime(CAMERA_FRAME, frame)).to_sec()
    except tf.Exception:
         # card *never seen*
         return 10000
    
if __name__=="__main__":
    rospy.init_node("word_detector") 
    
    WORDS_TOPIC = rospy.get_param('~detected_words_topic','words_to_write');
    SPECIAL_TOPIC = rospy.get_param('~special_cards_topic','special_symbols');
    STOP_TOPIC = rospy.get_param('~stop_card_detected_topic','stop_learning');
    TEST_TOPIC = rospy.get_param('~test_card_detected_topic','test_learning');
    CAMERA_FRAME = rospy.get_param('~detector_frame_id','camera_frame');
    CAMERA_FRAME = 'v4l_frame'

    LANGUAGE = rospy.get_param('~language','english');

    pub_words = rospy.Publisher(WORDS_TOPIC, String, queue_size=10)
    pub_special = rospy.Publisher(SPECIAL_TOPIC, String, queue_size=10)
    pub_stop = rospy.Publisher(STOP_TOPIC, Empty, queue_size=10)
    pub_test = rospy.Publisher(TEST_TOPIC, Empty, queue_size=10)

    tf_listener = tf.TransformListener(True, rospy.Duration(0.5))
    rospy.sleep(0.5)
    rate = rospy.Rate(10)
    prevWord = ''
    wordToPublish = ''
    go_card_last_seen = 0. # in seconds

    while not rospy.is_shutdown():

        # Wait for the go card to appear
        while not rospy.is_shutdown():
            go_card_last_seen = last_seen_since("tag_341")       
            if go_card_last_seen < 0.1:
                break
        
        if rospy.is_shutdown():
           break

        #except tf.Exception: # likely a timeout
        #    print('...')
        #    continue

        rospy.loginfo("Got a 'GO' card! preparing a word to publish")

        lettersDetected = set()

        for tag, letter in tags_letters_mapping.items():

            try: 
                t = tf_listener.getLatestCommonTime(tag, CAMERA_FRAME)
                if (rospy.Time().now() - t).to_sec() < 0.3:

                    try:
                        trans, rot = tf_listener.lookupTransform(tag, CAMERA_FRAME, t)
                        if rot[2]-rot[3] > 0:
                            # the tag is not facing the camera
                            continue
                    except tf.ExtrapolationException:
                        continue
    
                    lettersDetected.add((letter, trans[0])) # trans[0] -> x coord in camera frame

            except tf.Exception:
                #this tag has not been seen yet
                continue
 
        if not lettersDetected:
            rospy.logwarn("Got a 'GO' card, but unable to find any letter!")
        else:

            sortedLetters = sorted(lettersDetected,cmp)
            wordToPublish = ''.join([l for l,_ in sortedLetters])

            if wordToPublish in prevWord:
                rospy.loginfo("I'm not publishing '%s' since it is still the same word (of part thereof)." % wordToPublish);
            else:
                rospy.loginfo('Publishing word: '+ wordToPublish);

                '''if tag in special_tags:
                    message = String()
                    message.data = wordToPublish
                    pub_special.publish(message)

                    if wordToPublish == 'stop':
                        message = Empty()
                        pub_stop.publish(message)
                    elif wordToPublish == 'test':
                        message = Empty()
                        pub_test.publish(message)
                else:'''
                message = String()
                message.data = wordToPublish
                pub_words.publish(message)
                
                tagsDetected = set();
                prevWord = wordToPublish
                
            #rospy.sleep(2)


        rospy.logdebug("Sleeping a bit to clear the tf cache...")
        rospy.sleep(1) #wait till the tag times out (in the use context
                        #it's not likely to get two words within 5s)
        rospy.logdebug("Ok, waiting for a new word")

    rate.sleep()
