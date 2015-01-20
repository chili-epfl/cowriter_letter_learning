#!/usr/bin/env python
'''
Publish word to write based on fiducial marker (chilitag) detected.
'''

import rospy
import tf
from std_msgs.msg import String, Empty

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

tags_words_mapping.update(special_tags)

# Add individual letters: tag IDs are the ASCII code of the letter
for char in range(ord('a'),ord('z') + 1):
    tags_words_mapping["tag_%d" % char] = chr(char)

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
    prevWord = []
    wordToPublish = None
    groups = {}
    
    
    test = False

    while not rospy.is_shutdown():
        for tag in tags_words_mapping:
            
            tagDetected = tf_listener.frameExists(tag);  
            
            if not tagDetected: 
                if tag in tagsDetected:
                    tagsDetected.remove(tag)
            else:
                if not tagsDetected:
                    tagsDetected.add(tag)
                else:
                    
                    for prevTag in frozenset(tagsDetected):
                        if prevTag != tag:
                            
                            try:
                                if tf_listener.frameExists(tag) and tf_listener.frameExists(prevTag):
                                    t = tf_listener.getLatestCommonTime(tag, prevTag)
                                    pos, rot = tf_listener.lookupTransform(tag, prevTag, t)
                                    d = abs(pos[0]) + abs(pos[1]) + abs(pos[2])
                                    r = abs(rot[0]) + abs(rot[1]) + abs(rot[2])
                                    
                                    tagsDetected.add(tag)
                                    noCommonTime = False
                                    
                                    letter1 = tags_words_mapping[tag]
                                    letter2 = tags_words_mapping[prevTag]
                                    
                                    groups.setdefault(letter1,set([letter1]))
                                    groups.setdefault(letter2,set([letter2]))
                                    
                                    
                                    if d<0.2 and r<0.3:
                                        groups[letter1].add(letter2)
                                        groups[letter2].add(letter1)
                                    
                                    if len(groups[letter1])==3:
                                        #print('word detected with :')
                                        print(groups[letter1])
                                        groups = {}
                                    elif len(groups[letter2])==3:
                                        #print('word detected with :')
                                        print(groups[letter2])
                                        groups = {}
                                    
                            except tf.ExtrapolationException:
                                pass
            '''
            if wordToPublish:
                rospy.loginfo('Publishing word: '+wordToPublish);

                if tag in special_tags:
                    message = String()
                    message.data = wordToPublish
                    pub_special.publish(message)

                    if wordToPublish == 'stop':
                        message = Empty()
                        pub_stop.publish(message)
                    elif wordToPublish == 'test':
                        message = Empty()
                        pub_test.publish(message)
                else:
                    message = String()
                    message.data = wordToPublish
                    pub_words.publish(message)

                tf_listener = tf.TransformListener(True, rospy.Duration(0.1))
                rospy.logdebug("Sleeping a bit to clear the tf cache...")
                rospy.sleep(5) #wait till the tag times out (in the use context
                                #it's not likely to get two words within 5s)
                rospy.logdebug("Ok, waiting for a new word")
                
                tagsDetected = set();
                wordToPublish = None
                groups = {}'''
                    
    rate.sleep()
    
    if rospy.is_shutdown():
        print('rospy is shutdown !')
