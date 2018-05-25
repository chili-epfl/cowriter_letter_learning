#!/usr/bin/env python2
# coding: utf-8

"""
Manages the positioning of shapes. Assigns new shapes to their 
appropriate location, and can convert the location of an event into the
shape which it was done around. 
"""
import numpy

shapeWidth = 0.04;
shapeHeight = 0.0465;
shapeSize = numpy.array([shapeWidth,shapeHeight]);
    
#list of preferred shape cells
positionList_shape0 = [[1,1],[0,0],[2,0],[1,0],[0,1],[2,1],[0,2],[2,2],[1,2],[0,3],[2,3],[1,3],[1,4],[0,4],[2,4]];
positionList_shape1 = [[1,2],[0,2],[2,2],[1,1],[1,3],[0,1],[0,3],[2,1],[2,3],[1,0],[0,0],[2,0],[1,4],[0,4],[2,4]];
positionList_shape2 = [[1,3],[0,4],[2,4],[1,4],[1,2],[0,3],[2,3],[0,2],[2,2],[0,1],[2,1],[1,1],[1,0],[0,0],[2,0]];
positionList = [positionList_shape0, positionList_shape1, positionList_shape2];

class ShapeDisplayManager: #TODO make implementation of abstract class/interface

    def __init__(self): #TODO allow for generic shape and dimensions of display grid
        self.shapesDrawn = numpy.ones((3,5,2))*numpy.NaN; #3rd dim: shapeType_code, ID

    def clearAllShapes(self):
        self.shapesDrawn = numpy.ones((3,5,2))*numpy.NaN; #3rd dim: shapeType_code, ID
        
    def displayNewShape(self, shapeType_code):
        if(shapeType_code > (len(positionList)-1)):
            print('I don\'t know how to position that shape');
            return [-1, -1];
        else:
            row = -1; col = -1;
            foundSpace = False;
            positionList_index = 0;
            while((not foundSpace) and (positionList_index < len(positionList[shapeType_code]))):
                #check next position in position list for this shape
                [row_test, col_test] = positionList[shapeType_code][positionList_index];
                if(numpy.isnan(self.shapesDrawn[row_test,col_test,0])):
                    #space is available
                    row = row_test;
                    col = col_test;
                    foundSpace = True;
                else:
                    #space is not available - keep looking
                    positionList_index += 1;
            
            if(foundSpace):
                shapeID = numpy.equal(self.shapesDrawn[:,:,0],shapeType_code).sum();
                self.shapesDrawn[row,col,0] = shapeType_code;
                self.shapesDrawn[row,col,1] = shapeID;   
            else:
                print('I cannot draw here.');
            numRows = self.shapesDrawn.shape[0];
            position = [(col+0.5)*shapeWidth,((numRows-1)-row+0.5)*shapeHeight];
            return position;
            
    def isPossibleToDisplayNewShape(self, shapeType_code):
        if(shapeType_code > (len(positionList)-1)):
            print('I don\'t know how to position that shape');
            foundSpace = False;
        else:
            row = -1; col = -1;
            foundSpace = False;
            positionList_index = 0;
            while((not foundSpace) and (positionList_index < len(positionList[shapeType_code]))):
                #check next position in position list for this shape
                [row_test, col_test] = positionList[shapeType_code][positionList_index];
                if(numpy.isnan(self.shapesDrawn[row_test,col_test,0])):
                    #space is available
                    foundSpace = True;
                else:
                    #space is not available - keep looking
                    positionList_index += 1;

        return foundSpace;

    def indexOfLocation(self, location):
        location = numpy.array(location);
        location_cell = (location - shapeSize/2)/shapeSize;
    
        numRows = self.shapesDrawn.shape[0];

        row = (numRows -1)- int(round(location_cell[1]));
        col = int(round(location_cell[0]));
        return row, col
        
    def shapeAtLocation(self, location):          
        #map a location to the shape drawn at that location. 
        #shapeType_code will be -1 if invalid location.
        #shapeID will be -1 if no shape present at location.
        [row, col] = self.indexOfLocation(location);
    
        numRows = self.shapesDrawn.shape[0];
        numCols = self.shapesDrawn.shape[1];
        
        if(row>(numRows-1) or row<0):
            print('Invalid row');
            shapeType_code = -1;
            shapeID = 0;
        elif(col>(numCols-1) or col<0):
            print('Invalid column');
            shapeType_code = -1;
            shapeID = 0;
        else:
            shapeType_code = self.shapesDrawn[row,col,0];
            shapeID = self.shapesDrawn[row,col,1];
        
            if(numpy.isnan(shapeID)): #nothing is there
                shapeID = -1;
                shapeType_code = 0;
            else:
                shapeType_code = int(shapeType_code);
                shapeID = int(shapeID);

        return shapeType_code, shapeID
        
    def closestShapesToLocation(self, location):
        '''map a location to the closest shape(s) drawn at that location. 
        If multiple shapes are adjacent to the location, all will be returned.
        shapeType_code will be -1 if invalid location.
        shapeID will be -1 if no shapes have been drawn.
        '''
        [row, col] = self.indexOfLocation(location);
    
        numRows = self.shapesDrawn.shape[0];
        numCols = self.shapesDrawn.shape[1];
        
        if(row>(numRows-1) or row<0):
            print('Invalid row');
            shapeType_code = [-1];
            shapeID = [0];
        elif(col>(numCols-1) or col<0):
            print('Invalid column');
            shapeType_code = [-1];
            shapeID = [0];
        elif(numpy.all(numpy.isnan(self.shapesDrawn[:,:,0]))):
            print('No shapes drawn yet');
            shapeID = [-1];
            shapeType_code = [0];
        else:
            #calculate distance of all drawn shapes
            shapeIndexes = numpy.argwhere(numpy.isfinite(self.shapesDrawn[:,:,0]));
            numShapesDrawn = shapeIndexes.shape[0];
            distsToLocation = numpy.zeros(numShapesDrawn);
            for i in range(numShapesDrawn):
                distsToLocation[i] = numpy.sqrt((row-shapeIndexes[i,0])**2 + (col-shapeIndexes[i,1])**2);
                
            closest_indexes = numpy.argsort(distsToLocation);
            
            i = 0;
            shapeType_code = [];
            shapeID = [];
            #return all of the shapes which are closest if there are multiple at the same distance
            while(i < len(closest_indexes) and
                  distsToLocation[closest_indexes[i]] == distsToLocation[closest_indexes[0]]):
                nextClosest_row = shapeIndexes[closest_indexes[i],0];
                nextClosest_col = shapeIndexes[closest_indexes[i],1];
                shapeType_code.append(self.shapesDrawn[nextClosest_row,nextClosest_col,0]);
                shapeID.append(self.shapesDrawn[nextClosest_row,nextClosest_col,1]);
                i+=1;
        
        return shapeType_code, shapeID
        
    def displayShapeAtLocation(self, shapeType_code, location):
        '''blocks the space at location from being used
        '''
        [row, col] = self.indexOfLocation(location);
        numRows = self.shapesDrawn.shape[0];
        numCols = self.shapesDrawn.shape[1];
        
        if(row>(numRows-1) or row<0):
            print('Invalid row');
            success = False;
        elif(col>(numCols-1) or col<0):
            print('Invalid column');
            success = False;
        else:
            shapeID = numpy.equal(self.shapesDrawn[:,:,0],shapeType_code).sum();
            self.shapesDrawn[row,col,0] = shapeType_code;
            self.shapesDrawn[row,col,1] = shapeID;  
            success = True;
            
        return success
