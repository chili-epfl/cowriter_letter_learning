#!/usr/bin/env python2
from letter_learning_interaction.srv import *
import rospy
from letter_learning_interaction.shape_display_manager import ShapeDisplayManager

def handle_clear_all_shapes(request):
    shapeDisplayManager.clearAllShapes();
    rospy.loginfo('Shapes cleared');
    response = clearAllShapesResponse();
    response.success.data = True; #probably not necessary
    return response;

def handle_display_new_shape(request):
    response = displayNewShapeResponse();

    location = shapeDisplayManager.displayNewShape(request.shape_type_code);
    response.location.x = location[0];
    response.location.y = location[1];

    rospy.loginfo('Shape added at '+str(location));
    return response;

def handle_index_of_location(request):
    response = indexOfLocationResponse();
    location = [request.location.x, request.location.y];
    [response.row, response.column] = shapeDisplayManager.indexOfLocation(location);
    rospy.loginfo('Index returned: ' + str(response.row) + ', ' + str(response.column));
    return response;

def handle_shape_at_location(request):
    response = shapeAtLocationResponse();
    location = [request.location.x, request.location.y];
    [response.shape_type_code, response.shape_id] = shapeDisplayManager.shapeAtLocation(location);
    rospy.loginfo('Shape at location returned: '+str(response.shape_type_code)+'_'+str(response.shape_id));
    return response;

def handle_closest_shapes_to_location(request):
    response = closestShapesToLocationResponse();
    location = [request.location.x, request.location.y];
    [response.shape_type_code, response.shape_id] = shapeDisplayManager.closestShapesToLocation(location);
    rospy.loginfo('Closest shape(s) to location returned: '+str(response.shape_type_code)+'_'+str(response.shape_id));
    return response;

def handle_possible_to_display(request):
    response = isPossibleToDisplayNewShapeResponse();
    response.is_possible.data = shapeDisplayManager.isPossibleToDisplayNewShape(request.shape_type_code);
    rospy.loginfo('If possible returned '+str(response.is_possible.data));
    return response;

def handle_display_shape_at_location(request):
    response = displayShapeAtLocationResponse();
    location = [request.location.x, request.location.y];
    response.success.data = shapeDisplayManager.displayShapeAtLocation(request.shape_type_code, location);
    rospy.loginfo('Shape added at :' +str(location))
    return response;

def display_manager_server():
    rospy.init_node('display_manager_server')
    clear_service = rospy.Service('clear_all_shapes', clearAllShapes, handle_clear_all_shapes)
    rospy.loginfo("Ready to clear all shapes.");

    display_shape_service = rospy.Service('display_new_shape', displayNewShape, handle_display_new_shape)
    rospy.loginfo("Ready to display new shapes.");

    index_of_location_service = rospy.Service('index_of_location', indexOfLocation, handle_index_of_location)
    rospy.loginfo("Ready to determine index of location.");

    shape_at_location_service = rospy.Service('shape_at_location', shapeAtLocation, handle_shape_at_location)
    rospy.loginfo("Ready to determine shape at location.");

    closest_shapes_to_location_service = rospy.Service('closest_shapes_to_location', closestShapesToLocation, handle_closest_shapes_to_location)
    rospy.loginfo("Ready to determine closest shape(s) to location.");

    display_shape_at_location_service = rospy.Service('display_shape_at_location', displayShapeAtLocation, handle_display_shape_at_location)
    rospy.loginfo("Ready to display new shapes at specific location.");

    possible_to_display_service = rospy.Service('possible_to_display_shape', isPossibleToDisplayNewShape, handle_possible_to_display)
    rospy.loginfo("Ready to determine is shape fits.");
    rospy.spin()

if __name__ == "__main__":
    shapeDisplayManager = ShapeDisplayManager();

    display_manager_server()
    rospy.loginfo('shut down');
