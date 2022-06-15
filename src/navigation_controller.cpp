#include <ros/ros.h>
#include <string>
#include <iostream>
#include <chrono>
// this line includes the Action Specification for move_base which is a ROS action that exposes 
// a high level interface to the navigation stack.
// The move_base action accepts goals from clients and attempts to move the robot to the
// specified position/orientation in the world.
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/LaserScan.h>
#include <actionlib_msgs/GoalID.h>
#include <rt1_third_assignment/Goal.h>
#include <rt1_third_assignment/Velocity_control.h>
#include <rt1_third_assignment/Interface.h>
#include <geometry_msgs/Twist.h>

using namespace std;

// this line creates a convenience typedef for a SimpleActionClient that will allow us to communicate
// with actions that adhere to the MoveBaseAction action interface

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

double timeout = 120000000;
int TeleopVel;
string id = ""; 

bool auto_mode = false;
bool manual = false;
bool assisted = false;
bool canc = false;

float robot_x;
float robot_y;

float dist_x = 100;
float dist_y = 100;

float goal_x = 0.0;
float goal_y = 0.0;
float goal_th = 0.3;

ros::Publisher pubVel;
ros::Publisher pubCanc;

chrono::high_resolution_clock::time_point t_start;  
chrono::high_resolution_clock::time_point t_end; 

geometry_msgs::Twist teleopVel;
geometry_msgs::Twist new_vel;

/*
ranges will contain all distances detected by the laser scan sensor 
(between 0 and 180 degrees),whereas right_side, left_side and front_side
are ranges subsection. 
front_side contains all frontal distances between 
80 and 100 degrees.
right_side contains all lateral distances between 0 and 80 degrees.
left_side contains all lateral distances between 100 and 180 degrees                                                           
*/
float ranges[720], right_side[320], left_side[320], front_side[80];

float th = 0.5; // threshold which let us to consider a goal reached

/*
This is the callback function of the topic Velocity_control where Teleop is remapped.
Teleop is used only in the manual and assisted drive, so it can not always publish
on the cmd_vel topic, for this teleop is remapped on the Velocity_control topic.
In this way if the user uses teleop during the auto mode, it doesn't make effect on 
robot, becouse it publish on the Velocity_control topic that publish on the cmd_vel
topic only in the manual and assisted mode.
*/
void velCallback(const geometry_msgs::Twist::ConstPtr & msg){

		if(manual == true){

			pubVel.publish(msg);

		}
		if(assisted == true){

			new_vel.linear.x = msg->linear.x;
    		new_vel.angular.z = msg->angular.z;

		}
		else{

			return;
			
		}

}

/*
This function is called when the custom server (server_interface) is called.
It takes from the user interface node the command, given by the user.
For the auto, manual and assisted mode it sets the value of 3 boolean variable that 
are then used in the velCallback function to say if teleop can publish on teleop or 
not.
Instead when the user wants to cancel the actual goal, it checks the id of the goal
and publish on the move_base/cancel topic to cancel it.
*/
bool interface(rt1_third_assignment::Interface::Request &req, rt1_third_assignment::Interface::Response &res){


	if(req.command == 'm'){

		manual = true;
		auto_mode = false;
		assisted = false;
		cout<<"manual mode"<<endl;

	}
	else if(req.command == 'a'){

		auto_mode = true;
		manual = false;
		assisted = false;
		cout<<"auto mode"<<endl;

	}
	else if(req.command == 'd'){
		assisted = true;
		manual = true;
		auto_mode = false;
		cout<<"assisted driving mode"<<endl;
	}
	else if(req.command == 'c'){

		actionlib_msgs::GoalID canc_goal;
		canc_goal.id = id;
        pubCanc.publish(canc_goal);
		cout<<"goal canceled"<<endl;

	}

	return true;

}

/*
this function is used to take the position of the robot during its motion 
using the move_base/feedback 
*/
void robotPosition(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg) {

    	// Take the current robot position

    	robot_x = msg->feedback.base_position.pose.position.x;
    	robot_y = msg->feedback.base_position.pose.position.y;
    	//cout<<"robot pos x"<<robot_x<<endl;

	// Compute the error from the actual position and the goal position
	dist_x = robot_x - goal_x;
	dist_y = robot_y - goal_y;

	if (id != msg->status.goal_id.id) {

        id = msg->status.goal_id.id;

    	}

	if(abs(dist_x) <= goal_th && abs(dist_y) <= goal_th){

		actionlib_msgs::GoalID canc_goal;
		canc_goal.id = id;
        	pubCanc.publish(canc_goal);
		cout<<"goal reached"<<endl;

	}

	
	t_end = std::chrono::high_resolution_clock::now();
	auto time = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start).count();
	if (time > timeout) {
		actionlib_msgs::GoalID canc_goal;
		printf("\nThe goal point can't be reached!\n");
		canc_goal.id = id;
		pubCanc.publish(canc_goal);
		printf("Goal cancelled.\n");
	}
	
    	
}
	
bool goalPosition(rt1_third_assignment::Goal::Request &req, rt1_third_assignment::Goal::Response &res){

		move_base_msgs::MoveBaseGoal goal;

		// this line constructs an action client that we'll use to communicate with the action named
		// "move_base" that adheres to the MoveBaseAction interface. It also tells the action client to
		// start a thread to call ros::spin() so that ROS callbacks will be processed by passing "true"
		// as the second argument of the MoveBaseClient constructor.

		//tell the action client that we want to spin a thread by default
	
		MoveBaseClient ac("move_base",true);

		// Here we create a goal to send to move_base using the move_base_msgs::MoveBaseGoal message
		// type which is included automatically with the MoveBaseAction.h header. We'll just tell the 
		// to move 1 meter forward in the "base_link" coordinate frame.
		// The call to ac.sendGoal with actually push the goal out over the wire to the move_base node 
		// for processing

		//we will send a goal to the robot to move 1 meter forward
		goal.target_pose.header.frame_id = "map";
		//goal.target_pose.header.stamp = ros::Time::now();
	
		goal.target_pose.pose.orientation.w = 1.0;
		goal.target_pose.pose.position.x = req.x;
		goal.target_pose.pose.position.y = req.y;
		goal_x = goal.target_pose.pose.position.x;
		goal_y = goal.target_pose.pose.position.y;

		// this line wait for the action server to report that it has come up and is ready to begin
		// processing goals

		// wait for the action server to come up
		while(!ac.waitForServer(ros::Duration(5.0))){
			ROS_INFO("Waiting for the move_base action server to come up");
		}
		
		ROS_INFO("Sending goal");
		ac.sendGoal(goal);
		
		t_start = std::chrono::high_resolution_clock::now();
		/* THIS PART DOESN'T WORK

		// the only thing left to do now is to wait for the goal to finish using the ac.waitForGoalToFinish 
		// call which will block until the move_base action is done processing the goal we sent it. 
		// After it finishes, we can check if the goal succeded or failed and output a message to the user 
		// accordingly.
		if(ac.waitForResult(ros::Duration(timeout))){
			cout<<"ciaoo"<<endl;
			if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
				ROS_INFO("The goal is reached");
				res.feedback = 0; // the User Interface node will receive "okay" as feedback
			}else{
				ROS_INFO("The goal is not reachable");
				res.feedback = 1; // the User Interface node will receive "error" as feedback
			}
		}else{
			ROS_INFO("Action timed out.");
			res.feedback = 1; // the User Interface node will receive "error" as feedback
			ac.cancelGoal();
		}
		
		*/
		

		return true;
	}

float min_right_side(float * r){
	float min_r = 100;
	int j = 0;
	for(int i = 0; i < 320; i++){
		right_side[j] = r[i];
		if(right_side[j] < min_r){
			min_r = right_side[j];
		}
		j++;
	}
	return min_r;
}

// min_left_side is a function that takes as input parameter the ranges array
// ,computes the minimum value among the left lateral distances contained in 
// the left_side array and returns the minimum value as output.
float min_left_side(float * r){
	float min_l = 100;
	int j = 0;
	for(int i = 400; i < 720; i++){
		left_side[j] = r[i];
		if(left_side[j] < min_l){
			min_l = left_side[j];
		}
		j++;
	}
	return min_l;
}

// min_front_side is a function that takes as input parameter the ranges array
// ,computes the minimum value among the frontal distances contained in 
// the front_side array and returns the minimum value as output.
float min_front_side(float * r){
	float min_f = 100;
	int j=0;
	for(int i = 320; i < 400; i++){
		front_side[j] = r[i];
		if(front_side[j] <  min_f){
			min_f = front_side[j];
		}
		j++;
	}
	return min_f;
}



// avoid_walls is a function used to avoid that the robot crashes on the walls
// of the circuit. It takes as input parameters the front, left and right minimum
// values, computed by the min_left_side, min_right_side amd min_front_side
// functions and using this values it checks if the robot crosses the threshold
// ,for instance, firstly it checks if the front min value is less than th, 
// if it isn't ,the robot go on ,otherwise it checks if the left min
// is less than the right min and vice versa. If the left min is less than the right min 
// it means that the nearest wall is on the left and so on.
void avoid_walls(float front, float left, float right){

	float speed_d = left + right;

	if(new_vel.linear.x > 0 && new_vel.angular.z == 0){ //you are going forward

		if(front < th){

			new_vel.linear.x = 0;
			pubVel.publish(new_vel);
			cout<<"There is an obstacle in front of you"<<endl;

		}
	}

	else if(new_vel.linear.x >0 && new_vel.angular.z < 0){

		if(right < th){

			new_vel.linear.x = 0;
			new_vel.angular.z = 0;
			pubVel.publish(new_vel);
			cout<<"There is an obstacle on the right"<<endl;

		}
	}

	else if(new_vel.linear.x >0 && new_vel.angular.z > 0){

		if(left < th){

			new_vel.linear.x = 0;
			new_vel.angular.z = 0;
			pubVel.publish(new_vel);
			cout<<"There is an obstacle on the left"<<endl;

		}

	}

}


void drivingAssistance(const sensor_msgs::LaserScan::ConstPtr& msg){

	for(int i = 0; i < 720 ; i++){
		ranges[i] = msg -> ranges[i]; 
	}
	
	float f = min_front_side(ranges);
	float l = min_left_side(ranges);
	float r = min_right_side(ranges);
	
	avoid_walls(f, l, r);

}

int main(int argc, char** argv){

	ros::init(argc, argv, "controller");
		
	ros::NodeHandle n;
	
	ros::Subscriber subTeleVel = n.subscribe("/Velocity_control", 1000, velCallback); // teleop remapping

	ros::Subscriber subPos = n.subscribe("/move_base/feedback", 1000, robotPosition);

	ros::Subscriber sub_laser = n.subscribe("/scan", 1000, drivingAssistance); // Laser scanner
	
	pubVel = n.advertise<geometry_msgs::Twist>("/cmd_vel",1000);

	pubCanc = n.advertise<actionlib_msgs::GoalID>("/move_base/cancel",1000);
	
	ros::ServiceServer service_goal = n.advertiseService("/goal_position", goalPosition);

	ros::ServiceServer service_interface = n.advertiseService("/commands", interface);

	// it blocks the main thread from exiting until ROS invokes a shutdown.

	ros::spin();
	
	
	return 0;
}
