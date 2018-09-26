#!/bin/sh

pal-stop ros_topic_monitor
pal-stop web_commander

amixer sset 'Master' 40
