#!/usr/bin/env python

import os
import sys
import rospy
import copy
from RvizGenerator import RvizGenerator
from displays import displays

def main():
    """Example run"""
    node_name = "example"
    rospy.init_node(node_name)
    rospy.loginfo("Initializing {} node...".format(node_name))

    generator = RvizGenerator()
    image_display = displays.Image(DISPLAY_NAME="kalimera", DISPLAY_TOPIC="kalinuxta")
    image_display.modify()
    generator.add_display(image_display)
    generator.add_display(image_display)
    generator.add_display(image_display)

    generator.launch_rviz()


if __name__ == "__main__":
    main()
    
