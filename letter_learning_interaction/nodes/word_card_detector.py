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

#tags_words_mapping.update(special_tags)

# Add individual letters: tag IDs are the ASCII code of the letter
for char in range(ord('a'),ord('z') + 1):
    tags_words_mapping["tag_%d" % char] = chr(char)

def cmp(tag1, tag2):
    test = True
    while test:
        try:
            t = tf_listener.getLatestCommonTime(tag1, tag2)
            pos, rot = tf_listener.lookupTransform(tag1, tag2, t)
            test = False
        except tf.ExtrapolationException:
            pass
    dif = (int)(pos[0]<0)
    return 2*dif-1
    
    
if __name__=="__main__":
    rospy.init_node("word_detector") 
    
    WORDS_TOPIC = rospy.get_param('~detected_words_topic','words_to_write');
    SPECIAL_TOPIC = rospy.get_param('~special_cards_topic','special_symbols');
    STOP_TOPIC = rospy.get_param('~stop_card_detected_topic','stop_learning');
    TEST_TOPIC = rospy.get_param('~test_card_detected_topic','test_learning');
    CAMERA_FRAME = rospy.get_param('~detector_frame_id','CameraTop_frame');

    LANGUAGE = rospy.get_param('~language','english');

    pub_words = rospy.Publisher(WORDS_TOPIC, String, queue_size=10)
    pub_special = rospy.Publisher(SPECIAL_TOPIC, String, queue_size=10)
    pub_stop = rospy.Publisher(STOP_TOPIC, Empty, queue_size=10)
    pub_test = rospy.Publisher(TEST_TOPIC, Empty, queue_size=10)

    tf_listener = tf.TransformListener(True, rospy.Duration(0.5))
    rospy.sleep(0.5)
    rate = rospy.Rate(10)
    tagsDetected = set();
    prevWord = ''
    wordToPublish = ''
    groups = {}
    ok_go = False

    while not rospy.is_shutdown():
        for tag in tags_words_mapping:
            
            ok_go = tf_listener.frameExists('tag_341')
            
            tagDetected = tf_listener.frameExists(tag)
            if tagDetected:
                try:
                    t = tf_listener.getLatestCommonTime(tag, 'v4l_frame')
                    (trans,rot) = tf_listener.lookupTransform(tag, 'v4l_frame', t)
                    if rot[2]-rot[3]>0.2:
                        tagDetected = False
                except tf.ExtrapolationException:
                    tagDetected = False
            
            if not tagDetected: 
                if tag in tagsDetected:
                    tagsDetected.remove(tag)
                    groups = {}
            else:
                tagsDetected.add(tag)
                                
            if ok_go:
                word = sorted(tagsDetected,cmp)
                wordToPublish = ''
                for letter in word:
                    wordToPublish += tags_words_mapping[letter]
            
            
            if not wordToPublish in prevWord:
                rospy.loginfo('Publishing word: '+wordToPublish);

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
                groups = {}
                
                test = True
                while test:
                    test = tf_listener.frameExists('tag_341')
                    rospy.sleep(0.5)
                
                print('out of loop')
                ok_go = False

                tf_listener = tf.TransformListener(True, rospy.Duration(0.1))
                rospy.logdebug("Sleeping a bit to clear the tf cache...")
                rospy.sleep(5) #wait till the tag times out (in the use context
                                #it's not likely to get two words within 5s)
                rospy.logdebug("Ok, waiting for a new word")
                
                
                    
    rate.sleep()
    
    if rospy.is_shutdown():
        print('rospy is shutdown !')
