#!/bin/bash
rostopic pub person_detection geometry_msgs/PointStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'world'
point:
  x: -0.689974
  y: 0.358968
  z: 1.7"
