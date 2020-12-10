# -*- coding: utf-8 -*-
"""
Created on Wed Dec  9 21:34:19 2020

@author: Patrick
"""

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
        print("|-------------------------------|")
        i = 0
        for row in(self.gridNames):
            content = self.gridContent[i]
            i+=1
            print("|   " + row[0] + "   |   " + row[1]+ "   |   " + row[2] + "   |   " + row[3] + "   |")
            print("|   " + content[0] + "   |   " + content[1]+ "   |   " + content[2] + "   |   " + content[3] + "   |")
            print("|-------------------------------|")
        
    
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
                perception.append("map" + str(location))
            else:
                perception.append("obstacle" + str(location))
        return perception
    
    def perceiveObstacles(self,x,y):
        locations = [(x+1,y),(x-1,y),(x,y+1),(x,y-1)]
        perception = list()
        for location in locations:
            if self.onMap(location[0],location[1]):
                if self.gridContent[location[1]][location[0]] == 'X':
                    perception.append("obstacle" + str(location))         
        return perception
    
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
           

if __name__ == '__main__':
     myMap = GridMap()
     myMap.printMap()
     
     print(myMap.perceive())
     myMap.move("up")
     myMap.printMap()
     print(myMap.perceive())
     