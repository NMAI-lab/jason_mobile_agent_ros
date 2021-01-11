#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import re

def translateAction(data, args):
    (_, positionPublisher, destinationPublisher) = args
    action = data.data
    
    # Action format: getPath(Current,Destination)
    if "getPath" in action:
        parameterString = re.search('\((.*)\)', action).group(1)
        parameterString = parameterString.replace(" ","")
        parameterList = parameterString.split(",")
        
        current = parameterList[0]
        rospy.loginfo("Position: " + str(current))
        positionPublisher.publish(current)
        
        destination = parameterList[1]
        rospy.loginfo("Destination: " + str(destination))
        destinationPublisher.publish(destination)

    
def translatePath(data, args):
    (perceptionsPublisher, _, _) = args
    path = data.data
    perceptionString = "path(" + path + ")"
    perceptionString = perceptionString.replace(" ","")
    perceptionString = perceptionString.replace("'","")
    
    rospy.loginfo("Perceptions: " + str(perceptionString))
    perceptionsPublisher.publish(perceptionString) 


def rosMain():
    rospy.init_node('translator', anonymous=True)
    
    # Setup publishers
    perceptionsPublisher = rospy.Publisher('perceptions', String, queue_size=10)
    positionPublisher = rospy.Publisher('navigation/position', String, queue_size=10)    
    destinationPublisher = rospy.Publisher('navigation/destination', String, queue_size=10)
    publishers = (perceptionsPublisher, positionPublisher, destinationPublisher)
    
    # Subscribe to actions - monitor for actions relevant to the map
    rospy.Subscriber('actions', String, translateAction, publishers)
    
    # subscribe to the path topic
    rospy.Subscriber('nagivation/path', String, translatePath, publishers)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        rosMain()
    except rospy.ROSInterruptException:
        pass
    