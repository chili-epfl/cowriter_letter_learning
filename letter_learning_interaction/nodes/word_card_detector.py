#!/usr/bin/env python
'''
Publish word to write based on fiducial marker (chilitag) detected.
'''

import rospy
import tf
from std_msgs.msg import String, Empty

if __name__=="__main__":
    rospy.init_node("word_detector") 
    
    WORDS_TOPIC = rospy.get_param('~detected_words_topic','words_to_write');
    STOP_TOPIC = rospy.get_param('~stop_card_detected_topic','stop_learning');
    TEST_TOPIC = rospy.get_param('~test_card_detected_topic','test_learning');
    CAMERA_FRAME = rospy.get_param('~detector_frame_id','CameraTop_frame');

    LANGUAGE = rospy.get_param('~language','english');
    if(LANGUAGE.lower() == 'english'):
        tags_words_mapping = {'tag_5':'cow', 'tag_6':'son','tag_7':'cue','tag_8':'new','tag_9':'use', #english words
                      'tag_17':'test','tag_18':'stop' #special tags
                          };
    elif(LANGUAGE.lower() == 'french'):
        #warning: tag 14 is prone to false positives!!
        tags_words_mapping = {'tag_10':'cou','tag_11':'son','tag_12':'ces','tag_13':'une',
                      'tag_14':'sec','tag_15':'nos','tag_16':'ose',#'tag_17':'sac','tag_18':'eau', #french words
                      'tag_17':'test','tag_18':'stop' #special tags
                          };
    else:
        raise RuntimeError('Requested language ('+ LANGUAGE + ') not supported');
        
    pub_words = rospy.Publisher(WORDS_TOPIC, String)
    pub_stop = rospy.Publisher(STOP_TOPIC, Empty)
    pub_test = rospy.Publisher(TEST_TOPIC, Empty)
    
       
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
                print('found tag: '+tags_words_mapping[tag]);
            except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException),e:
                print e
                tagDetected = False;
            '''
            tagDetected = tf_listener.frameExists(tag);  
            
            #this method should prevent previous tags from being misdetected, by clearing the buffer
            if(tagDetected and not tag==prevTagDetected):
                prevTagDetected = tag;
                wordToPublish = tags_words_mapping[tag];
                                
                print('Publishing tag: '+wordToPublish);
                if(wordToPublish == 'stop'):
                    message = Empty();
                    pub_stop.publish(message);
                elif(wordToPublish == 'test'):
                    message = Empty();
                    pub_test.publish(message);
                else:
                    message = String();
                    message.data = wordToPublish;
                    pub_words.publish(message); 
                    
                tf_listener = tf.TransformListener(True, rospy.Duration(0.1))
                rospy.sleep(5) #wait till the tag times out (in the use context
                                #it's not likely to get two words within 5s)
    rate.sleep()
