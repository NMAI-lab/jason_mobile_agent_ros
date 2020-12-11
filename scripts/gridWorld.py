#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import re

class GridMap:
    
    # Initialize the map
    def __init__(self):

        # |-------------------------------|
        # |   m   |   n   |   o   |   p   |
        # |       |       |       |   X   |
        # | [0,0] | [1,0] | [2,0] | [3,0] |
        # |-------------------------------|
        # |   i   |   j   |   k   |   l   |
        # |       |   X   |       |       |
        # | [0,1] | [1,1] | [2,1] | [3,1] |
        # |-------------------------------|
        # |   e   |   f   |   g   |   h   |
        # |       |   X   |       |       |
        # | [0,2] | [1,2] | [2,2] | [3,2] |
        # |-------------------------------|
        # |   a   |   b   |   c   |   d   |
        # |       |       |   X   |       |
        # | [0,3] | [1,3] | [2,3] | [3,3] |
        # |-------------------------------|
        self.gridNames = [['m','n','o','p'],
                          ['i','j','k','l'],
                          ['e','f','g','h'],
                          ['a','b','c','d']]


        # Agent location is an A and X is not accessible
        self.gridContent = [[' ',' ',' ','X'],
                            [' ','X',' ',' '],
                            [' ','X',' ',' '],
                            ['A',' ','X',' ']]
        
        self.agentLocation = (0,3) # X,Y, to match the perceptions
                                   # Note, the grid is addressed [row][column], meaning it is [Y][X]


    def getLocationName(self,x,y):
        return self.gridNames[y][x] # Locations are stored row, column, meaning Y,X
        
    def printMap(self):
        print()
        print("|-------------------------------|")
        y = 0
        for row in(self.gridNames):
            content = self.gridContent[y]
            print("|   " + row[0] + "   |   " + row[1]+ "   |   " + row[2] + "   |   " + row[3] + "   |")
            print("|   " + content[0] + "   |   " + content[1]+ "   |   " + content[2] + "   |   " + content[3] + "   |")
            print("| [0," + str(y) + "] | [1," + str(y) + "] | [2," + str(y) + "] | [3," + str(y) + "] |")
            print("|-------------------------------|")
            y+=1
        print()
        
    
    def perceive(self):
        perception = list()
        
        perception.append("position" + str(self.agentLocation))
        perception.extend(self.perceiveMap(self.agentLocation[0],self.agentLocation[1]))
        perception.extend(self.perceiveObstacles(self.agentLocation[0],self.agentLocation[1]))
        
        
        return perception 

    def onMap(self,x,y):
        if (x >= 0) and (x <= 3) and (y >= 0) and (y <= 3):
            return True
        else:
            return False
        
    def perceiveMap(self,x,y):
        locations = [(x+1,y),(x-1,y),(x,y+1),(x,y-1)]
        perception = list()
        for location in locations:
            if self.onMap(location[0],location[1]):
                perception.append("map(" + str(self.getDirection(location)) + ")") 
            else:
                perception.append("obstacle(" + str(self.getDirection(location)) + ")") 
        return perception
    
    def perceiveObstacles(self,x,y):
        locations = [(x+1,y),(x-1,y),(x,y+1),(x,y-1)]
        perception = list()
        for location in locations:
            if self.onMap(location[0],location[1]):
                if self.gridContent[location[1]][location[0]] == 'X':
                    perception.append("obstacle(" + str(self.getDirection(location)) + ")")       
        return perception
    
    def getDirection(self,position):
        (x,y) = self.agentLocation
        (x1,y1) = position
 
        # Not bulletbroof!
        if y1 < y:
            return "up"
        elif y1 > y:
            return "down"
        elif x1 < x:
            return "left"
        else:
            return "right"
            
    
    def move(self, direction):
        (x,y) = self.agentLocation
        if direction == "up":
            y -=1
        elif direction == "down":
            y += 1
        elif direction == "left":
            x -= 1
        elif direction == "right":
            x += 1
        else:
            return False
        
        if self.onMap(x, y):
            if self.gridContent[y][x] != 'X':
                (oldX,oldY) = self.agentLocation
                self.gridContent[oldY][oldX] = ' '
                self.agentLocation = (x,y)
                self.gridContent[y][x] = 'A'
                return True
        return False


def runPerceptions(myMap, publisher):
    rate = rospy.Rate(2)
    
    while not rospy.is_shutdown():
        perceptionList = myMap.perceive()
        perceptionString = ""
    
        for perception in perceptionList:
            perceptionString += str(perception) + " "

        # Publish the perception
        rospy.loginfo("Perceptions: " + str(perceptionString))
        publisher.publish(perceptionString)   
        
        rate.sleep()
           
def act(data, args):
    (myMap) = args
    action = data.data
    
    rospy.loginfo("Action: " + str(action))
    
    if "move" in action:
        # Extract the action parameter between the brackets
        direction = re.search('\((.*)\)', action).group(1)
        myMap.move(direction)
        myMap.printMap()
        

def rosMain():
    myMap = GridMap()
    rospy.init_node('gridWorld', anonymous=True)
    publisher = rospy.Publisher('perceptions', String, queue_size=10)
    rospy.Subscriber('actions', String, act, (myMap))
    runPerceptions(myMap, publisher)


if __name__ == '__main__':
    try:
        rosMain()
    except rospy.ROSInterruptException:
        pass
    