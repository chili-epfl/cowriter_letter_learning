#!/usr/bin/env python

"""
Listens for gesture events on different topics and converts them into
appropriate feedback messages for shape_learning.
"""

import rospy
from std_msgs.msg import String, Empty
from geometry_msgs.msg import PointStamped

minTimeBetweenTouches = 0.1  #Seconds allowed between touches for the second one to be considered

def listenForAllGestures():
    global touch_subscriber, gesture_subscriber;
    #listen for touch events on the tablet
    touch_subscriber = rospy.Subscriber(TOUCH_TOPIC, PointStamped, touchInfoManager);
        
    #listen for touch events on the tablet
    gesture_subscriber = rospy.Subscriber(GESTURE_TOPIC, PointStamped, gestureManager); 

prevTouchTime = 0;   
def touchInfoManager(pointStamped):
    global touch_subscriber, gesture_subscriber, prevTouchTime
    touch_subscriber.unregister(); #unregister regardless of outcome, and re-subscribe if necessary
    gesture_subscriber.unregister(); 
    
    touchTime = pointStamped.header.stamp.to_sec();
    if((touchTime - prevTouchTime)>minTimeBetweenTouches):
        touchLocation = [pointStamped.point.x, pointStamped.point.y];
        
        #map touch location to closest shape drawn
        try:
            shape_at_location = rospy.ServiceProxy('shape_at_location', shapeAtLocation);
            request = shapeAtLocationRequest();
            request.location.x = touchLocation[0];
            request.location.y = touchLocation[1];
            response = shape_at_location(request);
            shapeType_code = response.shape_type_code;
            shapeID = response.shape_id;
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
        
        if(shapeType_code == -1):
            print('Touch not inside the display area');
            
        elif(shapeID == -1):
            print('Ignoring touch because not on valid shape');
        else:
            try:
                possible_to_display = rospy.ServiceProxy('possible_to_display_shape', isPossibleToDisplayNewShape);
                response = possible_to_display(shape_type_code = shapeType_code);
                ableToDisplay = response.is_possible.data;
            except rospy.ServiceException, e:
                ableToDisplay = False;
                print "Service call failed: %s"%e
            
            if(not ableToDisplay):#no more space
                print('Can\'t fit anymore letters on the screen');

            else:
                              
                print('Shape touched: '+str(shapeType_code)+'_'+str(shapeID))  
                feedbackMessage = String();
                feedbackMessage.data = str(shapeType_code) + '_' + str(shapeID);
                pub_feedback.publish(feedbackMessage);
                
        prevTouchTime = touchTime;
    else:
        print('Ignoring touch because it was too close to the one before');

    listenForAllGestures();
            
def gestureManager(pointStamped):
    global touch_subscriber, gesture_subscriber, prevTouchTime
    touch_subscriber.unregister(); #unregister regardless of outcome, and re-subscribe if necessary
    gesture_subscriber.unregister();   
    
    touchTime = pointStamped.header.stamp.to_sec(); 
    if((touchTime - prevTouchTime)>minTimeBetweenTouches):
        gestureLocation = [pointStamped.point.x, pointStamped.point.y];
        
        #map touch location to closest shape drawn
        try:
            shape_at_location = rospy.ServiceProxy('shape_at_location', shapeAtLocation);
            request = shapeAtLocationRequest();
            request.location.x = gestureLocation[0];
            request.location.y = gestureLocation[1];
            response = shape_at_location(request);
            shapeType_code = response.shape_type_code;
            shapeID = response.shape_id;
            #print( response.location);
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
                  
        if(shapeType_code == -1):
            print('Touch not inside the display area');
        elif(shapeID == -1):
            print('Ignoring touch because not on valid shape');
        else:
            try:
                possible_to_display = rospy.ServiceProxy('possible_to_display_shape', isPossibleToDisplayNewShape);
                response = possible_to_display(shape_type_code = shapeType_code);
                ableToDisplay = response.is_possible.data;
                #print( response.location);
            except rospy.ServiceException, e:
                ableToDisplay = False;
                print "Service call failed: %s"%e
            
            if(not ableToDisplay):#no more space
                print('Can\'t fit anymore letters on the screen');
            else:  
                
                print('Shape selected as best: '+str(shapeType_code)+'_'+str(shapeID))
                feedbackMessage = String();
                feedbackMessage.data = str(shapeType_code) + '_' + str(shapeID) + '_noNewShape';
                pub_feedback.publish(feedbackMessage);
                    
        prevTouchTime = touchTime;
    else:
        print('Ignoring touch because it was too close to the one before');
    
    listenForAllGestures();


if __name__ == "__main__":

    rospy.init_node("gesture_listener");
    #Topic for location of 'new shape like this one' gesture
    TOUCH_TOPIC = rospy.get_param('~touch_info_topic','touch_info');         
    #Topic for location of 'shape good enough' gesture
    GESTURE_TOPIC = rospy.get_param('~gesture_info_topic','gesture_info');  
    #Name of topic to publish feedback on
    FEEDBACK_TOPIC = rospy.get_param('~shape_feedback_topic','shape_feedback');  

    pub_feedback = rospy.Publisher(FEEDBACK_TOPIC, String);

    listenForAllGestures();
    
    #initialise display manager for shapes (manages positioning of shapes)
    from shape_learning_interaction.srv import *
    rospy.wait_for_service('shape_at_location'); 
    rospy.wait_for_service('possible_to_display_shape'); 

    rospy.spin();
