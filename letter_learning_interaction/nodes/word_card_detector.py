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
    prevTagDetected = [];

    while not rospy.is_shutdown():
        for tag in tags_words_mapping:
            '''
            #this method will keep detecting tags even after they are removed because they're still in the buffer
            try:
                tf_listener.waitForTransform(CAMERA_FRAME, tag, rospy.Time.now(), rospy.Duration(0.5))
                t = tf_listener.getLatestCommonTime(CAMERA_FRAME, tag)
                (trans,rot) = tf_listener.lookupTransform(CAMERA_FRAME, tag, t)
                tagDetected = True;
                rospy.loginfo('found tag: '+tags_words_mapping[tag]);
            except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException),e:
                rospy.err(e);
                tagDetected = False;
            '''
            tagDetected = tf_listener.frameExists(tag);  
            
            #this method should prevent previous tags from being misdetected, by clearing the buffer
            if(tagDetected and not tag==prevTagDetected):
                prevTagDetected = tag;
                wordToPublish = tags_words_mapping[tag];

                rospy.loginfo('Publishing tag: '+wordToPublish);

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
    rate.sleep()
