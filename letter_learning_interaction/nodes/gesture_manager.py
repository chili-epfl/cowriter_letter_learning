#!/usr/bin/env python

"""
Listens for gesture events on different topics and converts them into
appropriate feedback messages for shape_learning interaction nodes.
"""

import rospy
import numpy
from nav_msgs.msg import Path
from std_msgs.msg import String, Empty
from geometry_msgs.msg import PointStamped

from letter_learning_interaction.srv import *
from shape_learning.shape_learner_manager import ShapeLearnerManager, Shape
from shape_learning.shape_modeler import ShapeModeler

from letter_learning_interaction.msg import Shape as ShapeMsg

positionToShapeMappingMethod = 'basedOnClosestShapeToPosition';
shapePreprocessingMethod = "longestStroke";

def listenForAllGestures():
    global touch_subscriber, gesture_subscriber;
    #listen for touch events on the tablet
    touch_subscriber = rospy.Subscriber(TOUCH_TOPIC, PointStamped, touchInfoManager);
        
    #listen for touch events on the tablet
    gesture_subscriber = rospy.Subscriber(GESTURE_TOPIC, PointStamped, gestureManager); 


# ---------------------------------------------------- LISTENING FOR USER SHAPE
strokes=[];
def userShapePreprocessor(message):
    global strokes
    
    if(len(message.poses)==0): #a message with 0 poses signifies the shape has no more strokes
      
        if(len(strokes) > 0):                
            onUserDrawnShapeReceived(strokes, shapePreprocessingMethod, positionToShapeMappingMethod); 
        else:
            print('empty demonstration. ignoring')
            
        strokes = [];

    else: #new stroke in shape - add it
        print('Got stroke to write with '+str(len(message.poses))+' points');
        x_shape = [];
        y_shape = [];
        for poseStamped in message.poses:
            x_shape.append(poseStamped.pose.position.x);
            y_shape.append(-poseStamped.pose.position.y);
            
        numPointsInShape = len(x_shape); 

        #format as necessary for shape_modeler (x0, x1, x2, ..., y0, y1, y2, ...)'
        shape = [];
        shape[0:numPointsInShape] = x_shape;
        shape[numPointsInShape:] = y_shape;
        
        shape = numpy.reshape(shape, (-1, 1)); #explicitly make it 2D array with only one column
        strokes.append(shape);


# ------------------------------------------------------- PROCESSING USER SHAPE
def onUserDrawnShapeReceived(path, shapePreprocessingMethod, positionToShapeMappingMethod):
            
    #preprocess to turn multiple strokes into one path
    if(shapePreprocessingMethod == 'longestStroke'):
        path = processShape_longestStroke(strokes); 
    else:
        path = processShape_firstStroke(strokes);
            
            
    #identify type of shape which demonstration was for
    location = ShapeModeler.getShapeCentre(path);

    if(positionToShapeMappingMethod == 'basedOnColumnOfScreen'):
        shapeType_demoFor = getShapeCode_basedOnColumnOfScreen(location);

    elif(positionToShapeMappingMethod == 'basedOnRowOfScreen'):
        shapeType_demoFor = getShapeCode_basedOnColumnOfScreen(location);    

    elif(positionToShapeMappingMethod == 'basedOnShapeAtPosition'):
        shapeType_demoFor = getShapeCode_basedOnShapeAtPosition(location);
        
    else:
        shapeType_demoFor = getShapeCode_basedOnClosestShapeToPosition(location);
       
       
    #block the space from robot use
    try:
        display_shape_at_location = rospy.ServiceProxy('display_shape_at_location', displayShapeAtLocation);
        request = displayShapeAtLocationRequest();
        request.shape_type_code = shapeType_demoFor;
        request.location.x = location[0];
        request.location.y = location[1];
        #todo: allow for blocking different sized shapes
        response = display_shape_at_location(request);
        result = response.success;
        #todo: do something if unsuccessful
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    
    if(shapeType_demoFor == -1):# or response.shape_id == -1): 
        print("Ignoring demo because not for valid shape");
    else:
        #shapeType = wordManager.shapeAtIndexInCurrentCollection(shapeIndex_demoFor);
        
        demoShapeReceived = Shape(path=path, shapeType_code=shapeType_demoFor);
        #activeShapeForDemonstration_type = shapeType_demoFor;
        #send to learning_words_nao
        
        shapeMessage = makeShapeMessage(demoShapeReceived);
        pub_shapes.publish(shapeMessage);
        
# ---------------------------------------- FORMATTING SHAPE OBJECT INTO ROS MSG
# expects a ShapeLearnerManager.Shape as input
def makeShapeMessage(shape):
    shapeMessage = ShapeMsg();
    if(shape.path is not None):
        shapeMessage.path = shape.path;
    if(shape.shapeID is not None):
        shapeMessage.shapeID = shape.shapeID;
    if(shape.shapeType is not None):
        shapeMessage.shapeType = shape.shapeType;
    if(shape.shapeType_code is not None):
        shapeMessage.shapeType_code = shape.shapeType_code;
    if(shape.paramsToVary is not None):
        shapeMessage.paramsToVary = shape.paramsToVary;
    if(shape.paramValues is not None):
        shapeMessage.paramValues = shape.paramValues;

    return shapeMessage;    
        
# ------------------------------------------------- SHAPE PREPROCESSING METHODS
def processShape_longestStroke(strokes):
    length_longestStroke = 0;
    for stroke in strokes:
        strokeLength = stroke.shape[0]; #how many rows in array: number of points
        if(strokeLength > length_longestStroke):
            longestStroke = stroke;
            length_longestStroke = strokeLength;
    return longestStroke;
    
def processShape_firstStroke(strokes):
    return strokes[0];        
 
# --------------------------------------- INTENDED SHAPE INTERPRETATION METHODS
def getShapeCode_basedOnColumnOfScreen(location):
    #map to shapelearner based on third of the screen demo was in
    if(location[0]<.21/3):
        shapeType_demoFor = 0;
    elif(location[0]>.21/3*2):
        shapeType_demoFor = 2;
    else:
        shapeType_demoFor = 1;
    return shapeType_demoFor
    
def getShapeCode_basedOnRowOfScreen(location):
    try:
        index_of_location = rospy.ServiceProxy('index_of_location', indexOfLocation);
        request = indexOfLocationRequest();
        request.location.x = location[0];
        request.location.y = location[1];
        response = index_of_location(request);
        shapeIndex_demoFor = response.row;
        
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        shapeType_demoFor = -1;
        
    return shapeType_demoFor
    
def getShapeCode_basedOnShapeAtPosition(location):
    try:
        shape_at_location = rospy.ServiceProxy('shape_at_location', shapeAtLocation);
        request = shapeAtLocationRequest();
        request.location.x = location[0];
        request.location.y = location[1];
        response = shape_at_location(request);
        shapeType_demoFor = response.shape_type_code;
        
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        shapeType_demoFor = -1;
        
    return shapeType_demoFor
    
activeShapeForDemonstration_type = None;
def getShapeCode_basedOnClosestShapeToPosition(location):
    global demoShapeReceived, activeShapeForDemonstration_type
    try:
        closest_shapes_to_location = rospy.ServiceProxy('closest_shapes_to_location', closestShapesToLocation);
        request = closestShapesToLocationRequest();
        request.location.x = location[0];
        request.location.y = location[1];
        response = closest_shapes_to_location(request);
        closestShapes_type = response.shape_type_code;
        if(len(closestShapes_type)>1 and activeShapeForDemonstration_type is not None):
            try: #see if active shape is in list
                dummyIndex = closestShapes_type.index(activeShapeForDemonstration_type);
                shapeType_demoFor = activeShapeForDemonstration_type;
            except ValueError: #just use first in list otherwise
                shapeType_demoFor = closestShapes_type[0];
        else: #just use first in list
            shapeType_demoFor = closestShapes_type[0];
            
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        shapeType_demoFor = -1;

    return shapeType_demoFor
    
    
# ----------------------------------------------------- PROCESSING TOUCH EVENTS
prevTouchTime = 0;   
minTimeBetweenTouches = 0.1  #Seconds allowed between touches for the second one to be considered
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
            
            
# --------------------------------------------------- PROCESSING GESTURE EVENTS
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
    
    #Name of topic to get user drawn raw shapes on
    USER_DRAWN_SHAPES_TOPIC = rospy.get_param('~user_drawn_shapes_topic','user_drawn_shapes');
    #Name of topic to publish processed shapes on
    PROCESSED_USER_SHAPE_TOPIC = rospy.get_param('~processed_user_shape_topic','user_shapes_processed');

    
    #listen for user-drawn shapes
    shape_subscriber = rospy.Subscriber(USER_DRAWN_SHAPES_TOPIC, Path, userShapePreprocessor);
    
    pub_feedback = rospy.Publisher(FEEDBACK_TOPIC, String);
    pub_shapes = rospy.Publisher(PROCESSED_USER_SHAPE_TOPIC, ShapeMsg);

    listenForAllGestures();
    
    #initialise display manager for shapes (manages positioning of shapes)
    rospy.wait_for_service('shape_at_location'); 
    rospy.wait_for_service('possible_to_display_shape'); 
    rospy.wait_for_service('closest_shapes_to_location');
    rospy.wait_for_service('display_shape_at_location');
    rospy.wait_for_service('index_of_location');

    rospy.spin();
