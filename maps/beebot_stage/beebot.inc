
define sick_laser ranger
(
  sensor(
    range [ 0 90 ]
    fov 180.0
    samples 181
  )
  size [ 0.14 0.14 0.1 ]
  color "blue"
  block(
    points 6
    point[0] [  0.0375  0.015 ]
    point[1] [  0.0225  0.03 ]
    point[2] [ -0.0375  0.03 ]
    point[3] [ -0.0375 -0.03 ]
    point[4] [  0.0225 -0.03 ]
    point[5] [  0.0375 -0.015 ]
    z [ 0 0.06 ]
  )
)

#define kinect ranger
#(
#  sensor
#  (
#    range_max 6.5
#    fov 58.0
#    samples 640
#  )
  # generic model properties
#  color "black"
#  size [ 0.06 0.15 0.03 ]
#)

define beebot position
(
  pose [ 0.0 0.0 0.0 0.0 ]

  odom_error [0.03 0.03 999999 999999 999999 0.02]

  size [ 0.7 0.7 0.375 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  drive "diff"
  color "yellow"
  
#  kinect(pose [ -0.1 0.0 -0.11 0.0 ])
  sick_laser( pose [0.42 0 -0.1675 0 ] )

  # [ xmin xmax ymin ymax zmin zmax amin amax ]
  velocity_bounds [-1000 1000 -1000 1000 -1000 1000 -3600 3600 ]
)
