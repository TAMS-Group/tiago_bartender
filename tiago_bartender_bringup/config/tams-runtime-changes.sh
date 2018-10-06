#!/bin/sh

# these modules subscribe to the camera RGB image
# and have to be stopped to access the IR image
pal-stop ros_topic_monitor
pal-stop web_commander

amixer sset 'Master' 50
