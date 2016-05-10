#include "rrt_planner_2d_helper.h"
#include <limits>
#include <cmath>
#include "ros/ros.h"
//#include <stdlib.h>     /* srand, rand */
//#include "nav_msgs/OccupancyGrid.h"


template <typename T> int sgn(T val){
    return (T(0) < val)-(val < T(0));
}


bool intersection_segment_segment(const float x1,const float y1,const float x2,
		const float y2,const float x3,const float y3,const float x4,const float y4){

    bool intersection = false;   //return value-->true if collision
    float a1,b1,c1,r3,r4,a2,b2,c2,r1,r2;
    //From http://www.realtimerendering.com/resources/GraphicsGems/gemsii/xlines.c
    //Returns no point of intersection. Just for collision purpose
    // Compute a1, b1, c1, where line joining points 1 and 2
    // is "a1 x  +  b1 y  +  c1  =  0".

    a1 = y2 - y1;
    b1 = x1 - x2;
    c1 = x2 * y1 - x1 * y2;

    // Compute r3 and r4.

    r3 = a1 * x3 + b1 * y3 + c1;
    r4 = a1 * x4 + b1 * y4 + c1;

    //Check signs of r3 and r4.  If both point 3 and point 4 lie on
    //same side of line 1, the line segments do not intersect.

    if ( r3 != 0 && r4 != 0 && sgn(r3)==sgn(r4)){
        intersection = 0;
        return intersection;
    }

    //Compute a2, b2, c2

    a2 = y4 - y3;
    b2 = x3 - x4;
    c2 = x4 * y3 - x3 * y4;
    
    //Compute r1 and r2 */

    r1 = a2 * x1 + b2 * y1 + c2;
    r2 = a2 * x2 + b2 * y2 + c2;

    //Check signs of r1 and r2.  If both point 1 and point 2 lie
    //on same side of second line segment, the line segments do
    //not intersect.

    if ( r1 != 0 && r2 != 0 && sgn(r1)==sgn(r2)){
    	intersection = false;
		return intersection;
	}

    //Line segments intersect: compute intersection point TODO.
    intersection = true;
    return intersection;
}


bool intersection_square_segment(const float x1,const float y1,const float x2,const float y2,const float x_center,const float y_center,const float resolution){
	float x3,y3,x4,y4;
	bool intersection = false;

	for (int i=0; i<4; i++){
		switch (i){
			case 0:
				x3 = x_center-resolution/2;
				y3 = y_center+resolution/2;
				x4 = x_center+resolution/2;
				y4 = y_center+resolution/2;
				break;
			case 1:
				x3 = x_center+resolution/2;
				y3 = y_center+resolution/2;
				x4 = x_center+resolution/2;
				y4 = y_center-resolution/2;
				break;
			case 2:
				x3 = x_center-resolution/2;
				y3 = y_center-resolution/2;
				x4 = x_center+resolution/2;
				y4 = y_center-resolution/2;
				break;
			case 3:
				x3 = x_center-resolution/2;
				y3 = y_center+resolution/2;
				x4 = x_center-resolution/2;
				y4 = y_center-resolution/2;
				break;
		}
		if (intersection_segment_segment(x1,y1,x2,y2,x3,y3,x4,y4)){
			intersection = true;
			break;
		}
	}
	return intersection;
}

bool rrt_cones_2d(float **waypoints, int *number_waypoints, int **tree_connections, int *number_connections, float **tree_points, int *points_added_out, const float max_length,const float min_length,const int max_points,const float max_angle, const nav_msgs::OccupancyGrid::ConstPtr& grid, const geometry_msgs::Point start,const geometry_msgs::Point goal,const float max_error){
	bool found = false;  //set false unless path found
	*number_waypoints = 0;
	//float tree_points[max_points][2];
	float tree_branches_per_point[max_points];
	memset(tree_branches_per_point, 0, max_points*sizeof(float));
	//float tree_connections[max_points][2];
	float vector[2];
	float angle_2_goal;
	float distance_to_goal;
	float previous_point[2];
	float point_to_test[2];
	float versor[2];
	int points_added = 0;
	int waipoints_added = 0;
	int connection_added = 0;
	float random_angle;
	float random_number;
	float random_increment;
	int index = 0;
    int max_iterations = max_points*100;  //TODO check-remove
    int iterations = 0;

	//Fill start in the tree
	points_added++;
	tree_points[points_added-1][0] = start.x;
	tree_points[points_added-1][1] = start.y;

	while(points_added < max_points && !found && iterations < max_iterations){	//main loop of algorithm
		random_number = (float)rand() / (RAND_MAX + 1.0); //Random between 0 and 0.99999  TODO check this
		int random_point_in_tree = floor(random_number*points_added) + 1;
        //ROS_INFO("RRT: iteration");
		while (tree_branches_per_point[random_point_in_tree-1] > 3){   //to avoid points with more than 5 branches
			random_number = random_number = (float)rand() / (RAND_MAX + 1.0);
			random_point_in_tree = floor(random_number*points_added) + 1;
		}
		if (random_point_in_tree == 1){
		        vector[0] = goal.x-start.x;
				vector[1] = goal.y-start.y;
		} else {
			for (int j=0; j<connection_added; j++){
				if (tree_connections[j][1] == random_point_in_tree){
					int previous_point_index = tree_connections[j][0];
					previous_point[0] = tree_points[previous_point_index-1][0];
					previous_point[1] = tree_points[previous_point_index-1][1];
					break;
				}
			}
			vector[0] = tree_points[random_point_in_tree-1][0]-previous_point[0];
			vector[1] = tree_points[random_point_in_tree-1][1]-previous_point[1];
		}
		angle_2_goal = angle_2_vectors_2d(vector[0],vector[1],goal.x-tree_points[random_point_in_tree-1][0],goal.y-tree_points[random_point_in_tree-1][1]);
		distance_to_goal = norm(tree_points[random_point_in_tree][0]-goal.x, tree_points[random_point_in_tree][1]-goal.y);
		bool collision = true;
		if (angle_2_goal < max_angle && angle_2_goal > -max_angle && distance_to_goal > min_length && distance_to_goal < max_length){
		        point_to_test[0] = goal.x;
		        point_to_test[1] = goal.y;
		        collision = check_collision(grid, tree_points[random_point_in_tree-1][0],tree_points[random_point_in_tree-1][1], point_to_test[0], point_to_test[1], distance_to_goal);
		}
		if (collision){
			if (angle_2_goal > max_angle){
				angle_2_goal = max_angle;
			}
			if (angle_2_goal < -max_angle){
				angle_2_goal = -max_angle;
			}
			random_angle = random_number*max_angle*2-max_angle; //TODO generate a new random instead of using the old value
			random_increment = random_number*(max_length-min_length)+min_length;	//TODO generate a new random instead of using the old value
			random_angle = random_number/2*angle_2_goal + (1-random_number/2)*random_angle; //BIAS toward goal of angle
			versor[0] = vector[0]/(norm(vector[0],vector[1]));
			versor[1] = vector[1]/(norm(vector[0],vector[1]));
			point_to_test[0] = tree_points[random_point_in_tree-1][0] + random_increment*versor[0]*cos(random_angle) - random_increment*versor[1]*sin(random_angle);
			point_to_test[1] = tree_points[random_point_in_tree-1][1] + random_increment*versor[0]*sin(random_angle) + random_increment*versor[1]*cos(random_angle);
			collision = check_collision(grid, tree_points[random_point_in_tree-1][0],tree_points[random_point_in_tree-1][1], point_to_test[0], point_to_test[1], random_increment);
		}
		if (!collision){	//add point and connection in tree
			points_added++;
			tree_points[points_added-1][0] = point_to_test[0];
			tree_points[points_added-1][1] = point_to_test[1];
			tree_branches_per_point[random_point_in_tree-1]++;
			connection_added++;
			tree_connections[connection_added-1][0] = random_point_in_tree;
			tree_connections[connection_added-1][1] = points_added;
			if (norm(point_to_test[0]-goal.x,point_to_test[1]-goal.y) < max_error){	//found path
				found = true;
				//Build waypoints
				index = points_added;
				while (index != 1){
					waipoints_added++;
					waypoints[waipoints_added-1][0] = tree_points[index-1][0];
					waypoints[waipoints_added-1][1] = tree_points[index-1][1];
					for (int j=0; j<connection_added; j++){
						if (tree_connections[j][1] == index){
							index = tree_connections[j][0];
							break;
						}
					}
				}
                waipoints_added++;
				waypoints[waipoints_added-1][0] = start.x;
				waypoints[waipoints_added-1][1] = start.y;
				//Reverse waypoints TODO

				*number_waypoints = waipoints_added;
			}
		}
        iterations++;
	}
	*number_connections = connection_added;
	*points_added_out = points_added;
    ROS_INFO("RRT: FINISH");
	return found;
}

float angle_2_vectors_2d(float ax, float ay, float bx, float by){
	return atan2(ax*by-ay*bx, ax*bx+ay*by);
}

float norm(float a, float b){
	return sqrt(pow(a,2)+pow(b,2));
}

bool check_collision(const nav_msgs::OccupancyGrid::ConstPtr& grid, const float from_x, const float from_y, const float to_x, const float to_y, const float distance){
	//distance is the norm of the distance between "from" and "to" points
	//TODO better cells selection to check. Now looking for all the cells in the square around the radius "distance"
	int width = grid->info.width;
	int height = grid->info.height;
	float resolution = grid->info.resolution;
	float origin_x = grid->info.origin.position.x;
	float origin_y = grid->info.origin.position.y;
	//ROS_INFO("Map data: %d - %d - %f - %f - %f", width, height, resolution, origin_x, origin_y);
	int threshold = 10;	//TODO define better based on data
	int width_tile_index_from = floor((from_x-origin_x)/resolution); // index of the tile of initial point along x
	int height_tile_index_from = floor((from_y-origin_y)/resolution); // index of the tile of initial point along y
	int numbers_of_tiles_to_check_around = ceil(distance/resolution);
	//ROS_INFO("Collision data: %d - %d - %d", width_tile_index_from, height_tile_index_from, numbers_of_tiles_to_check_around);
	for (int i=width_tile_index_from-numbers_of_tiles_to_check_around; i< width_tile_index_from+numbers_of_tiles_to_check_around+1; i++){
		for (int j=height_tile_index_from-numbers_of_tiles_to_check_around; j< height_tile_index_from+numbers_of_tiles_to_check_around +1; j++){
			if (grid->data[j*width+i] > threshold){
				if (intersection_square_segment(from_x,from_y,to_x,to_y,i*resolution-resolution/2+origin_x,j*resolution-resolution/2+origin_y,resolution)){
					return true;
				}
			}
		}
	}
	return false;
}



/*void find_closest_2D(float **points, const float initial_point[2], float closest_point[2], int *index, int N_points){             //TESTED: WORKS
	*index = 0;
	float distance;
	float temp_distance;
	closest_point[0] = points[0][0];
	closest_point[1] = points[0][1];
	distance = sqrt(pow(points[0][0]-initial_point[0],2)+pow(points[0][1]-initial_point[1],2));
	
	for (int i=1; i<N_points; i++){
		temp_distance = sqrt(pow(points[i][0]-initial_point[0],2)+pow(points[i][1]-initial_point[1],2));	
		if (temp_distance < distance){
			distance = temp_distance;
			*index = i;
			closest_point[0] = points[i][0];
			closest_point[1] = points[i][1];
		}
	}
}


void WP_grid(float **vertex, int *N_vertex, const float initial_position[2], const float d, float **WP, bool *success, int *number_WP){
	//variables
	int index_WP = 0;
	int index_temp_WP = 0;
	float **temp_WP;
	temp_WP = new float *[2];
	for(int i = 0; i<2; i++){
    	temp_WP[i] = new float[2];
	}
	float closest_point [2];
	float initial_pos_internal [2];
	int index = 0;
	float m;
	float q;
	float q_inc;
	bool convex;
	bool positive_intersection = false;
	float s1[2];
	float s2[2];
	int is_intersecting;
	float intersection_point[2];
	bool collision = true;
	const int MAX_WP = 150;     //TODO try to not make it hardcoded
	//initialize
	for (int i=0; i<2; i++){
		for (int j=0; j<2; j++){
			temp_WP [i][j] = 0;
		}
	}
	*success = false;
	*number_WP = 0;
	
	//----START AGLORITHM-----
	is_convex(vertex, true, &convex, N_vertex);
	//is_convex(points, true, &convex, &N_vertex);

	if (!convex) return;     //Some error in convex. Probably less that 3 vertex

	find_closest_2D(vertex, initial_position, closest_point, &index, *N_vertex);	
	WP[index_WP][0] = closest_point[0];   //filling first waypoint
	WP[index_WP][1] = closest_point[1];
	index_WP++;
	//ROS_INFO("GRID: filling first WP");
	if (index == *N_vertex-1){
		WP[index_WP][0] = vertex[0][0];   //filling second waypoint
		WP[index_WP][1] = vertex[0][1];
		index_WP++;
		//ROS_INFO("GRID: filling second WP (1) Index: %d", index);
	} else {
		WP[index_WP][0] = vertex[index+1][0];   //filling second waypoint
		WP[index_WP][1] = vertex[index+1][1];
		index_WP++;
		//ROS_INFO("GRID: filling second WP (2). Index: %d", index);
	}
	
	if (WP[0][0] != WP[1][0]){     //first and second waypoint has different x
		m = (WP[0][1] - WP[1][1]) / (WP[0][0] - WP[1][0]);
		q = WP[0][1] - m * WP[0][0];
		q_inc = d / sin(M_PI/2-atan(m));
	} else {     //first and second waypoint has same x
		m = std::numeric_limits<float>::infinity();
		q = WP[0][0];
		q_inc = d;
	}
	//ROS_INFO("GRID: m: %f - q: %f - q_inc: %f", m, q, q_inc);

	for (int i=0; i<*N_vertex; i++){
		s1[0] = vertex[i][0];
		s1[1] = vertex[i][1];
		if (i<*N_vertex-1){       //circular problem
			s2[0] = vertex[i+1][0];
			s2[1] = vertex[i+1][1];
		} else {
			s2[0] = vertex[0][0];
			s2[1] = vertex[0][1];
		}
		intersection_line_segment(s1, s2, m, q+q_inc, &is_intersecting, intersection_point);
		//ROS_INFO("GRID: Positive intersection: s1: %f-%f - s2: %f-%f - m: %f - q: %f - int: %d - point: %f-%f",s1[0], s1[1], s2[0], s2[1], m, q, is_intersecting, intersection_point[0], intersection_point[1]);
		if (is_intersecting > 0){     //we have intersection with at least one segment. we can break the for
			positive_intersection = true;
			//ROS_INFO("GRID: Positive intersection");
			break;
		}
	}
	while (collision){
		//ROS_INFO("GRID: inside while");
		if (index_WP>=MAX_WP){          //too much WP
			*number_WP = MAX_WP;
			*success = false;
			//ROS_INFO("GRID: too much WP");
			goto end;                  //END
		}
		collision = false;
		if (positive_intersection){
			q += q_inc;
		} else {
			q -= q_inc;
		}
		for (int i=0; i<*N_vertex; i++){
			s1[0] = vertex[i][0];
			s1[1] = vertex[i][1];
			if (i<*N_vertex-1){       //circular problem
				s2[0] = vertex[i+1][0];
				s2[1] = vertex[i+1][1];
			} else {
				s2[0] = vertex[0][0];
				s2[1] = vertex[0][1];
			}
			//ROS_INFO("GRID: testing segment: %f-%f --> %f-%f", s1[0], s1[1], s2[0], s2[1]);
			intersection_line_segment(s1, s2, m, q, &is_intersecting, intersection_point);
			//ROS_INFO("GRID: intersection point: %f-%f", intersection_point[0], intersection_point[1]);
			if (is_intersecting == 1){  //one intersection
				collision = true;
				temp_WP[index_temp_WP][0] = intersection_point[0];
				temp_WP[index_temp_WP][1] = intersection_point[1];
				index_temp_WP++;
				//ROS_INFO("GRID: collision 1");
			} else if (is_intersecting == 2){        //collinear --> the two WP are the edges of the i-th segment (points s1 and s2)
				//ROS_INFO("GRID: collision 2");
				collision = true;
				index_temp_WP = 0;            //absolutely needed to reset the temp_WP found, since the two WP from collinearity already includes prevoius WP found, otherwise out of range
				temp_WP[index_temp_WP][0] = s1[0];
				temp_WP[index_temp_WP][1] = s1[1];
				index_temp_WP++;
				temp_WP[index_temp_WP][0] = s2[0];
				temp_WP[index_temp_WP][1] = s2[1];
				index_temp_WP++;
			} else {
				//ROS_INFO("GRID: no collision");	
			}
			if (index_temp_WP == 2){   //found two temp WP: time to fill real WP
				index_temp_WP = 0;
				initial_pos_internal[0] = WP[index_WP-1][0];  
				initial_pos_internal[1] = WP[index_WP-1][1];
				find_closest_2D(temp_WP, initial_pos_internal, closest_point, &index, 2);
				WP[index_WP][0] = closest_point[0];
				WP[index_WP][1] = closest_point[1];
				index_WP++;
				if (index == 0){
					WP[index_WP][0] = temp_WP[1][0];
					WP[index_WP][1] = temp_WP[1][1];
					index_WP++;
				} else {
					WP[index_WP][0] = temp_WP[0][0];
					WP[index_WP][1] = temp_WP[0][1];
					index_WP++;
				}
			}
		}
	}
	//ROS_INFO("GRID: END");     //if algorythm was succesfull
	*number_WP = index_WP;
	*success = true;
	end:
	delete[] temp_WP;
	//DO NOTHING. Algorythm ended (succesfully or not)
}*/




