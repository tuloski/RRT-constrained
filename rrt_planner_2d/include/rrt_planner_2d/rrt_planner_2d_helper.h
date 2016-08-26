/*
 * rrt_planenr_2d_helper.h
 *
 * 
 */


/* Include files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdint.h>
#include "nav_msgs/OccupancyGrid.h"
#include "voro++.hh"

/* Function Declarations */
extern bool intersection_segment_segment(const float x1,const float y1,const float x2,const float y2,const float x3,const float y3,const float x4,const float y4);
extern bool intersection_square_segment(const float x1,const float y1,const float x2,const float y2,const float x_center,const float y_center,const float resolution);
extern bool rrt_cones_2d(std::vector<std::vector<float> >& waypoints, int *number_waypoints, std::vector<std::vector<int> >& tree_connections, int *number_connections, std::vector<std::vector<float> >& tree_points, int *points_added_out, const float max_length,const float min_length,const int max_points,const float max_angle, const nav_msgs::OccupancyGrid::ConstPtr& grid, const geometry_msgs::Point start,const geometry_msgs::Point goal,const float max_error, const float clearance);
extern float angle_2_vectors_2d(float ax, float ay, float bx, float by);
extern float norm(float a, float b);
extern bool check_collision(const nav_msgs::OccupancyGrid::ConstPtr& grid, const float from_x, const float from_y, const float to_x, const float to_y, const float distance);

