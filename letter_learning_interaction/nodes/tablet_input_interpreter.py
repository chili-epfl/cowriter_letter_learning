#!/usr/bin/env python

"""
Listens for interaction events from the tablet and converts them into
appropriate messages for shape_learning interaction nodes.

Currently implemented:
- Receiving user-drawn shapes (demonstrations for learning alg.) as a series
of Path messages of strokes and processing the shape by keeping only the 
longest stroke and determining which shape being shown by the display_manager 
the demonstration was for.
- Receiving the location of a gesture on the tablet which represents which 
shape to give priority to if the demonstration was drawn next to multiple
shapes (if using the 'basedOnClosestShapeToPosition' method to map user demo to
intended shape).

Implemented but not in use: 
- Receiving touch and long-touch gestures and converting that to feedback for
the learning algorithm from when it was touch-feedback only.
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
shapePreprocessingMethod = "merge" #"longestStroke";


# ---------------------------------------------------- LISTENING FOR USER SHAPE
strokes=[];
def userShapePreprocessor(message):
    global strokes
    
    if(len(message.poses)==0): #a message with 0 poses signifies the shape has no more strokes
      
        if(len(strokes) > 0):                
            onUserDrawnShapeReceived(strokes, shapePreprocessingMethod, positionToShapeMappingMethod); 
        else:
            rospy.loginfo('empty demonstration. ignoring')
            
        strokes = [];

    else: #new stroke in shape - add it
        rospy.loginfo('Got stroke to write with '+str(len(message.poses))+' points');
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
    if(shapePreprocessingMethod == 'merge'):
        path = processShape_mergeStrokes(strokes); 
    elif(shapePreprocessingMethod == 'longestStroke'):
        path = processShape_longestStroke(strokes); 
    else:
        path = processShape_firstStroke(strokes);

    demoShapeReceived = Shape(path=path);
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

def processShape_mergeStrokes(strokes):
    x_shape = []
    y_shape = []
    for stroke in strokes:
        nbpts = stroke.shape[0] / 2
        x_shape.extend(stroke[:nbpts,0])
        y_shape.extend(stroke[nbpts:,0])

    return numpy.array(x_shape + y_shape)
    
def processShape_firstStroke(strokes):
    return strokes[0];        

# ----------------- PROCESS GESTURES FOR SETTING ACTIVE SHAPE FOR DEMONSTRATION
activeShapeForDemonstration_type = None;
def onSetActiveShapeGesture(message):
    global activeShapeForDemonstration_type
    
    gestureLocation = [message.point.x, message.point.y];
    #map gesture location to shape drawn
    try:
        shape_at_location = rospy.ServiceProxy('shape_at_location', shapeAtLocation);
        request = shapeAtLocationRequest();
        request.location.x = gestureLocation[0];
        request.location.y = gestureLocation[1];
        response = shape_at_location(request);
        shapeType_code = response.shape_type_code;
        shapeID = response.shape_id;
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s",e);
        
    if(shapeType_code != -1 and shapeID != -1):
        activeShapeForDemonstration_type = shapeType_code;
        rospy.loginfo('Setting active shape to shape ' + str(shapeType_code));

'''  
    
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
            rospy.logerr("Service call failed: %s",e);
        
        
        if(shapeType_code == -1):
            rospy.loginfo('Touch not inside the display area');
            
        elif(shapeID == -1):
            rospy.loginfo('Ignoring touch because not on valid shape');
        else:
            try:
                possible_to_display = rospy.ServiceProxy('possible_to_display_shape', isPossibleToDisplayNewShape);
                response = possible_to_display(shape_type_code = shapeType_code);
                ableToDisplay = response.is_possible.data;
            except rospy.ServiceException, e:
                ableToDisplay = False;
                rospy.logerr("Service call failed: %s",e);
            
            if(not ableToDisplay):#no more space
                rospy.loginfo('Can\'t fit anymore letters on the screen');

            else:
                              
                rospy.loginfo('Shape touched: '+str(shapeType_code)+'_'+str(shapeID))  
                feedbackMessage = String();
                feedbackMessage.data = str(shapeType_code) + '_' + str(shapeID);
                pub_feedback.publish(feedbackMessage);
                
        prevTouchTime = touchTime;
    else:
        rospy.loginfo('Ignoring touch because it was too close to the one before');

    listenForAllGestures();
            
            
# ------------------------------------------------ PROCESSING LONG-TOUCH EVENTS
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
            #rospy.loginfo( response.location);
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s",e);
                  
        if(shapeType_code == -1):
            rospy.loginfo('Touch not inside the display area');
        elif(shapeID == -1):
            rospy.loginfo('Ignoring touch because not on valid shape');
        else:
            try:
                possible_to_display = rospy.ServiceProxy('possible_to_display_shape', isPossibleToDisplayNewShape);
                response = possible_to_display(shape_type_code = shapeType_code);
                ableToDisplay = response.is_possible.data;
                #rospy.loginfo( response.location);
            except rospy.ServiceException, e:
                ableToDisplay = False;
                rospy.logerr("Service call failed: %s",e);
            
            if(not ableToDisplay):#no more space
                rospy.loginfo('Can\'t fit anymore letters on the screen');
            else:  
                
                rospy.loginfo('Shape selected as best: '+str(shapeType_code)+'_'+str(shapeID))
                feedbackMessage = String();
                feedbackMessage.data = str(shapeType_code) + '_' + str(shapeID) + '_noNewShape';
                pub_feedback.publish(feedbackMessage);
                    
        prevTouchTime = touchTime;
    else:
        rospy.loginfo('Ignoring touch because it was too close to the one before');
    
    listenForAllGestures();
    
def listenForAllGestures():
    global touch_subscriber, gesture_subscriber;
    #listen for touch events on the tablet
    touch_subscriber = rospy.Subscriber(TOUCH_TOPIC, PointStamped, touchInfoManager);
        
    #listen for touch events on the tablet
    gesture_subscriber = rospy.Subscriber(GESTURE_TOPIC, PointStamped, gestureManager); 

'''

if __name__ == "__main__":

    rospy.init_node("tablet_input_interpreter");
    '''
    #Topic for location of 'new shape like this one' gesture
    TOUCH_TOPIC = rospy.get_param('~touch_info_topic','touch_info');         
    #Topic for location of 'shape good enough' gesture
    GESTURE_TOPIC = rospy.get_param('~gesture_info_topic','gesture_info');  
    #Name of topic to publish feedback on
    FEEDBACK_TOPIC = rospy.get_param('~shape_feedback_topic','shape_feedback');
    pub_feedback = rospy.Publisher(FEEDBACK_TOPIC, String); 
    listenForAllGestures(); #start touch and long-touch subscribers
    '''
    
    #Name of topic to get gestures representing the active shape for demonstration
    GESTURE_TOPIC = rospy.get_param('~gesture_info_topic','gesture_info');
    #Name of topic to get user drawn raw shapes on
    USER_DRAWN_SHAPES_TOPIC = rospy.get_param('~user_drawn_shapes_topic','user_drawn_shapes');
    #Name of topic to publish processed shapes on
    PROCESSED_USER_SHAPE_TOPIC = rospy.get_param('~processed_user_shape_topic','user_shapes_processed');

    #listen for gesture representing active demo shape 
    gesture_subscriber = rospy.Subscriber(GESTURE_TOPIC, PointStamped, onSetActiveShapeGesture); 
    
    #listen for user-drawn shapes
    shape_subscriber = rospy.Subscriber(USER_DRAWN_SHAPES_TOPIC, Path, userShapePreprocessor);
    

    pub_shapes = rospy.Publisher(PROCESSED_USER_SHAPE_TOPIC, ShapeMsg, queue_size=10);
    
    #initialise display manager for shapes (manages positioning of shapes)
    rospy.wait_for_service('shape_at_location'); 
    rospy.wait_for_service('possible_to_display_shape'); 
    rospy.wait_for_service('closest_shapes_to_location');
    rospy.wait_for_service('display_shape_at_location');
    rospy.wait_for_service('index_of_location');

    rospy.spin();
