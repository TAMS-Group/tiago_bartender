#!/bin/bash
rostopic pub person_detection geometry_msgs/PointStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'map'
point:
  x: -3.0
  y: 0.0
  z: 1.5"
