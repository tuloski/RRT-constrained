#include "ros/ros.h"

#include <sys/time.h>   //For time benchmark
#include <ctime>	//For time benchmark
#include <visualization_msgs/Marker.h>
#include <cmath>


#include "rrt_planner_2d_helper.h"         //GRID
#include "geometry_msgs/PoseStamped.h"
//#include "reference/LeashingCommand.h"   //leashing
//#include "reference/LeashingStatus.h"    //leashing
//#include "geographic_msgs/GeoPose.h"	 //leashing
//#include <wgs84_ned_lib/wgs84_ned_lib.h>       

typedef long long int64; typedef unsigned long long uint64;	//For time benchmark


//TODO delete[] dynamic arrays on shutdown
//TODO use vectors instead of dynamic arrays
//TODO parameter server


uint64 GetTimeMs64()
{

	/* Linux */
	struct timeval tv;

	gettimeofday(&tv, NULL);

	uint64 ret = tv.tv_usec;
	/* Convert from micro seconds (10^-6) to milliseconds (10^-3) */
	ret /= 1000;

	/* Adds the seconds (10^0) after converting them to milliseconds (10^-3) */
	ret += (tv.tv_sec * 1000);

	return ret;
}

class RRT_planner_2d_NodeClass {
public:
	RRT_planner_2d_NodeClass(ros::NodeHandle& node){

		n_ = node;

		//subscribers
		//subFromPosition_ = n_.subscribe("/position_nav", 10, &RRT_planner_2d_NodeClass::readPositionMessage,this);
		subMap = n_.subscribe("/map", 10, &RRT_planner_2d_NodeClass::readMap,this);
        subGoal = n_.subscribe("/goal_rrt", 10, &RRT_planner_2d_NodeClass::readGoal,this);
        subStart = n_.subscribe("/start_rrt", 10, &RRT_planner_2d_NodeClass::readStart,this);
        subGoalRviz = n_.subscribe("/move_base_simple/goal", 10, &RRT_planner_2d_NodeClass::readGoalRviz,this);

		// publishers
		//pubToReference_ = n_.advertise<guidance_node_amsl::Reference>("/reference",10);
		marker_pub = n_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
		
		//Initializing

		rate = 1;  //TODO put real frequency or no frequency
		start.x = 0;
		start.y = -5;
		start.z = 0;

		goal.x = 6;
		goal.y = 0;
		goal.z = 0;

		N_WP = 0;      //OUTPUT
		number_connections = 0;
		number_points = 0;
		max_points = 3000;	//TODO take prom parameter server

		/*WP = new float *[150];        //OUTPUT    //TODO check hardcoded 150
		for(int i = 0; i<150; i++){
			WP[i] = new float[2];
		}
		tree_connections = new int *[max_points];
		for(int i = 0; i<max_points; i++){
			tree_connections[i] = new int[2];
		}
		tree_points = new float *[max_points];
		for(int i = 0; i<max_points; i++){
			tree_points[i] = new float[2];
		}*/
		max_length = 3;	//TODO take prom parameter server
		min_length = 0.2;
		max_angle = 0.55;
		max_error = 0.2;
        clearance = 0.2;
	}

	void readMap(const nav_msgs::OccupancyGrid::ConstPtr& grid){
        //TODO check only collision first to see if old path is good
		_grid = grid;
        
        //temp
        /*bool collision;
        geometry_msgs::Point p;
        collision = check_collision(_grid, 0, -5, 5, 0, 0.2);
        ROS_INFO("Collision: %s", collision ? "true" : "false");
        p.x = 0;
		p.y = -5;
		p.z = 0;
		line_list.points.push_back(p);
        p.x = 5;
		p.y = 0;
		p.z = 0;
		line_list.points.push_back(p);
        marker_pub.publish(line_list);*/
        //----------

        executeRRT();
	}
	
    void readGoal(const geometry_msgs::Point::ConstPtr& goal_in){
        goal = *goal_in;
        points.action = points.DELETE;
        marker_pub.publish(points);
        points.action = points.ADD;
        points.points.resize(0);
        points.points.push_back(start);
		points.points.push_back(goal);
		marker_pub.publish(points);
        executeRRT();
    }

    void readGoalRviz(const geometry_msgs::PoseStamped::ConstPtr& goal_in_rviz){
        goal = goal_in_rviz->pose.position;
        points.action = points.DELETE;
        marker_pub.publish(points);
        points.action = points.ADD;
        points.points.resize(0);
        points.points.push_back(start);
		points.points.push_back(goal);
		marker_pub.publish(points);
        executeRRT();
    }

    void readStart(const geometry_msgs::Point::ConstPtr& start_in){
        start = *start_in;
        points.action = points.DELETE;
        marker_pub.publish(points);
        points.action = points.ADD;
        points.points.resize(0);
        points.points.push_back(start);
		points.points.push_back(goal);
		marker_pub.publish(points);
        executeRRT();
    }

    void executeRRT(){
        bool collision;
		bool found;
		uint64 time;

		time = GetTimeMs64();
		//collision = check_collision(grid, 0, 5, 1, 0, norm(0-5,1-0));
		found = rrt_cones_2d(WP, &N_WP, tree_connections, &number_connections, tree_points, &number_points, max_length, min_length, max_points, max_angle, _grid, start, goal, max_error,clearance);
		time = GetTimeMs64() - time;
		ROS_INFO("Elapsed milliseconds: %llu", time);
		ROS_INFO("Found: %s", found ? "true" : "false");


        //---- PRINT TREE -----//
        ROS_INFO("Printing %d lines",number_connections);
        geometry_msgs::Point p;
		for (int i=0; i<number_connections;i++){
			//ROS_INFO("Point %d: %f - %f",i, tree_points[tree_connections[i][0]-1][0], tree_points[tree_connections[i][0]-1][1]);
			p.x = tree_points[tree_connections[i][0]-1][0];
			p.y = tree_points[tree_connections[i][0]-1][1];
			p.z = 0;
			line_list.points.push_back(p);
			p.x = tree_points[tree_connections[i][1]-1][0];
			p.y = tree_points[tree_connections[i][1]-1][1];
			p.z = 0;
			line_list.points.push_back(p);
		}
		marker_pub.publish(line_list);
        line_list.points.resize(0);

		if (found){
            ROS_INFO("[RRT]: found");
			geometry_msgs::Point p;
			for (int i=0; i<N_WP;i++){
				p.x = WP[i][0];
				p.y = WP[i][1];
                ROS_INFO("[RRT]: WP(%d): %f - %f",i,WP[i][0],WP[i][1]);
				p.z = 0;
				line_strip.points.push_back(p);
			}
			marker_pub.publish(line_strip);
            line_strip.points.resize(0);
		} else {
            ROS_INFO("[RRT]: not found");
		}

        WP.resize(0);
        tree_connections.resize(0);
        tree_points.resize(0);

		//delete[] tree_connections;
		//delete[] WP;
		//new_pos = true;
    }

	void run() {
		ros::Rate loop_rate(rate);

		while (ros::ok())
		{
			ROS_INFO_ONCE("REF: RUNNING");

			//Reference_Handle();
			ros::spinOnce();

			bool intersection;
			float p1 = -2;
			float p2 = -1;
			float p3 = 1;
			float p4 = 3;
			float center_x = 1;
			float center_y = 3;
			float resolution = 2;
			uint64 time;

			//-------------- LINES RVIZ -----------//
			points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/map";
			points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
			points.ns = line_strip.ns = line_list.ns = "points_and_lines";
			points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
			points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;

			points.id = 0;
			line_strip.id = 1;
			line_list.id = 2;

			points.type = visualization_msgs::Marker::POINTS;
			line_strip.type = visualization_msgs::Marker::LINE_STRIP;
			line_list.type = visualization_msgs::Marker::LINE_LIST;

			// POINTS markers use x and y scale for width/height respectively
			points.scale.x = 0.2;	//Points width
			points.scale.y = 0.2;	//Points height
			points.color.g = 1.0f;	//Points color green
			points.color.a = 1.0;

			// LINE_STRIP
			line_strip.scale.x = 0.05; //line width
			line_strip.color.r = 1.0;  //color red
			line_strip.color.a = 1.0;

			//LINE LIST
			line_list.scale.x = 0.05;
			line_list.color.b = 1.0;	//color blue
			line_list.color.a = 1.0;

			/*geometry_msgs::Point p;
			p.x = ;
			p.y = 5;
			p.z = 0;
			line_strip.points.push_back(p);
			p.x = 1;
			p.y = 0;
			p.z = 0;
			line_strip.points.push_back(p);
			marker_pub.publish(line_strip);*/


			//-----------------------------------------//



			//intersection = intersection_segment_segment(p1,p2,p3,p4,p5,p6,p7,p8);             //usage. Takes <1ms for 10k executions

			//intersection = intersection_square_segment(p1,p2,p3,p4,center_x,center_y,resolution); //35ms for 100k
			//ROS_INFO("Intersection: %s", intersection ? "true" : "false");



            /*time = GetTimeMs64();

            for (int i=0;i<100000;i++){
            	//intersection = intersection_square_segment(p1,p2,p3,p4,center_x,center_y,resolution);
            }

			time = GetTimeMs64() - time;
			ROS_INFO("Elapsed milliseconds: %llu", time);*/

			loop_rate.sleep();
		}
        //delete[] tree_connections;
		//delete[] WP;
        //delete[] tree_points;
	}

protected:
	/*state here*/
	ros::NodeHandle n_;

	//ros::Subscriber subFromPosition_;
	ros::Subscriber subMap;
    ros::Subscriber subGoal;
    ros::Subscriber subStart;
    ros::Subscriber subGoalRviz;

	//ros::Publisher pubToReference_;
	ros::Publisher marker_pub;

	visualization_msgs::Marker line_strip;
	visualization_msgs::Marker points;
	visualization_msgs::Marker line_list;
	//PUT HERE VARIABLES
	int rate;
	geometry_msgs::Point start;
	geometry_msgs::Point goal;
    nav_msgs::OccupancyGrid::ConstPtr _grid;

	//float **WP;    //OUTPUT
    std::vector<std::vector<float> > WP;
    std::vector<std::vector<int> > tree_connections;
    std::vector<std::vector<float> > tree_points;
	//int **tree_connections;    //OUTPUT
	//float **tree_points;
	int N_WP;      //OUTPUT
	int number_connections;
	int number_points;
	int max_points;
	float max_length;
	float min_length;
	float max_angle;
	float max_error;
    float clearance;

private:

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rrt_planner_2d");
	ros::NodeHandle node;

	RRT_planner_2d_NodeClass rrt_planner2dNode(node);

	rrt_planner2dNode.run();
	return 0;
}
