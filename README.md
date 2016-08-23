# RRT-constrained
RRT planner 2D with geometrical constrains.
The generated trajectory consists of sequences of points that approximate a feasible trajectory for some class of robots (see paper TODO).
The sequence of points can be constrained to be in a min/max distance from the previous point and in a limited arc to approximate limited curvatures-dynamic models.

## Run
rosrun rrt_planner_2d rrt_planner_2d

## Load the map
roslaunch map_server unibo.launch (in the launch you can put a different .pgm files)

