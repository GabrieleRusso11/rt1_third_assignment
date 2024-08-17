/**
* \file navigation_controller.cpp
* \brief the node that implements all the drive modalities of the robot (and more)
* \author Gabriele Russo
* \date 06/03/2022
*
* \details
*
* Subscribes to:<BR>
*
* °/Velocity_control the remapped topic used to control the robot in the manual/assisted driving mode 
*
* °/move_base/feedback topic which contains the current robot position
*
* °/scan topic which contains the output of the robot laser scan
*
*
* Publishes to:<BR>
*
* °/cmd_vel in order to modify the robot velocity in the assisted driving mode 
*
* °/move_base/cancel in order to cancel the move base goal
*
*
* Custom Services:<BR>
*
* °/goal_position custom server which receives the robot desired goal position from the user interface node
*
* °/commands custom server which receives the user commands from the user interface node
*
*
* Description:
* 
* This node receives from the user interface node the drive mode chosen by the user.
* There are three drive mode:
*
* Auto drive mode: in this case the user insert a desired robot position, and the robot
* will try to reach the goal position (autonomously) using the move base action server
*
* Manual drive mode: in this case the user will be able to control the robot using the teleop
* twist keyboard. Hence, the user will can drive manually the robot towards a desired position
*
* Assisted drive mode: in this case the user can drive manually the robot, as in the manual 
* drive mode, but in this case there will be an assisted drive mode that will avoid the walls
* (and the obstacle) autonomously, when the user drives the robot towards a wall.
*
**/

#include <ros/ros.h>
#include <string>
#include <iostream>
#include <chrono>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/LaserScan.h>
#include <actionlib_msgs/GoalID.h>
#include <rt1_third_assignment/Goal.h>
#include <rt1_third_assignment/Interface.h>
#include <geometry_msgs/Twist.h>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient; ///< this line creates a convenience typedef for a SimpleActionClient that will allow us to communicate with actions that adhere to the MoveBaseAction action interface

double timeout = 120000000; ///< timeout of 2 minutes
string id = ""; ///< goal identifier

bool auto_mode = false; ///< auto drive mode flag
bool manual = false; ///< manual drive mode flag
bool assisted = false; ///< assisted drive mode flag
bool canc = false; ///< flag for cancelling the goal

float robot_x; ///< the x coordinate of the robot position
float robot_y; ///< the y coordinate of the robot position

float e_x = 100; ///< the initialization of the x coordinate error
float e_y = 100; ///< the initialization of the y coordinate error

float goal_x = 0.0; ///< the initialization of the goal position x coordinate 
float goal_y = 0.0; ///< the initialization of the goal position y coordinate
float goal_th = 0.3; ///< allowable position goal error threshold 

ros::Publisher pubVel; ///< the robot velocity publisher
ros::Publisher pubCanc; ///< the goal cancellation publisher 

chrono::high_resolution_clock::time_point t_start; ///< the starting time instant
chrono::high_resolution_clock::time_point t_end; ///< the ending time instant

geometry_msgs::Twist new_vel; ///< new robot velocity to publish

float ranges[720]; ///< will contain all distances detected by the laser scan sensor (between 0 and 180 degrees).
float right_side[320]; ///< is a 'ranges' subsection, which contains all lateral distances between 0 and 80 degrees.
float left_side[320]; ///< is a 'ranges' subsection, which contains all lateral distances between 100 and 180 degrees.
float front_side[80]; ///< is a 'ranges' subsection, which contains all frontal distances between 80 and 100 degrees.

float th = 0.5; ///< threshold which let us to consider a goal reached

/**
*
* \brief interface function, used to compute the user commands
*
* \param req client reequest, the user commands
* \param res server response, no response
*
* \return true
*
* This function is called when the custom server (server_interface) is called.
* It takes from the user interface node the command, given by the user.
* For the auto, manual and assisted mode it sets the value of 3 boolean variable that 
* are then used in the velCallback function to say if teleop can publish on teleop or 
* not.
* Instead when the user wants to cancel the actual goal, it checks the id of the goal
* and publish on the move_base/cancel topic to cancel it.
*
**/
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
		cout<<"goal canceled by the user"<<endl;

	}

	return true;

}

/**
*
* \brief velCallback function, executed each time a new robot velocity is published from teleop
*
* \param msg robot velocity
*
* \return
*
* This is the callback function of the topic Velocity_control where Teleop is remapped.
* Teleop is used only in the manual and assisted drive, so it can not always publish
* on the cmd_vel topic, for this teleop is remapped on the Velocity_control topic.
* In this way if the user uses teleop during the auto mode, it doesn't make effect on 
* robot, becouse it publish on the Velocity_control topic that publish on the cmd_vel
* topic only in the manual and assisted mode.
*
**/
void velCallback(const geometry_msgs::Twist::ConstPtr & msg){

	if(manual == true){

		pubVel.publish(msg);

	}
	if(assisted == true){

		new_vel.linear.x = msg->linear.x;
		new_vel.angular.z = msg->angular.z;

	}
	if(auto_mode == true){

		return;
		
	}
}

/**
*
* \brief robotPosition function, used to obtain the updated robot position 
* \param msg move base goal informations
*
* \return
*
* This function is used to take the position of the robot during its motion 
* using the move_base/feedback topic.
* So it takes the x and y coordinate of robot and compute the error distance 
* between the actual position of the robot and the goal position.
* If the error distance is acceptable (i.e. lower than a goal threshold) then
* it takes the id of the goal, that is continuously updated, and cancel the 
* the goal itself, using the topic /move_base/cancel, in order to say (in this 
* case) that the goal is reached.
* Instead if the timeout is elapsed than the goal is cancelled beacause it is
* unreachable.
*
**/
void robotPosition(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg) {

    // Take the current robot position
	robot_x = msg->feedback.base_position.pose.position.x;
	robot_y = msg->feedback.base_position.pose.position.y;

	// Compute the error from the actual position and the goal position
	e_x = robot_x - goal_x;
	e_y = robot_y - goal_y;

	if (id != msg->status.goal_id.id) {

        id = msg->status.goal_id.id;

    }

	if(abs(e_x) <= goal_th && abs(e_y) <= goal_th){

		actionlib_msgs::GoalID canc_goal;
		canc_goal.id = id;
        pubCanc.publish(canc_goal);
		//cout<<"goal reached"<<endl;
		
	}

	
	t_end = std::chrono::high_resolution_clock::now();
	auto time = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start).count();

	if (time > timeout) {

		actionlib_msgs::GoalID canc_goal;
		//cout<<"TIMEOUT ELAPSED : The goal point can't be reached!"<<endl;
		canc_goal.id = id;
		pubCanc.publish(canc_goal);
		//cout<<"Goal cancelled."<<endl;

	}
    	
}

/**
*
* \brief goalPosition function, used to obtain the goal coordinates from the user interface
*
* \param req client request, the goal coordinates
* \param res server response, no response
*
* \return true
*
* This function is called when the custom server (server_goal) is called.
* It takes from the user interface node the coordinates of the desired 
* goal position (choosen by the user) and using an action client it sends
* this coordinates to the MoveBase action server that will move the robot
* towards the desired goal position.
*
**/
bool goalPosition(rt1_third_assignment::Goal::Request &req, rt1_third_assignment::Goal::Response &res){

	move_base_msgs::MoveBaseGoal goal;

	// this line constructs an action client that we'll use to communicate with the action named
	// "move_base" that adheres to the MoveBaseAction interface. 
	MoveBaseClient ac("move_base",true);

	// Here we create a goal to send to move_base using the move_base_msgs::MoveBaseGoal message
	// type which is included automatically with the MoveBaseAction.h header.
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.pose.orientation.w = 1.0;
	goal.target_pose.pose.position.x = req.x;
	goal.target_pose.pose.position.y = req.y;
	goal_x = goal.target_pose.pose.position.x;
	goal_y = goal.target_pose.pose.position.y;

	// this line wait for the action server to report that it has come up and is ready to begin
	// processing goals
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}
	
	// The call to ac.sendGoal with actually push the goal out over the wire to the move_base node 
	// for processing
	ROS_INFO("Sending goal");
	ac.sendGoal(goal);
	
	t_start = std::chrono::high_resolution_clock::now();

	// the only thing left to do now is to wait for the goal to finish using the ac.waitForGoalToFinish 
	// call which will block until the move_base action is done processing the goal we sent it. 
	// After it finishes, we can check if the goal succeded or failed and output a message to the user 
	// accordingly.
	if(ac.waitForResult(ros::Duration(timeout))){
		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
			ROS_INFO("The goal is reached");
			res.goal_feedback = "Goal Reached"; // the User Interface node will receive "okay" as feedback
		}else{
			ROS_INFO("The goal is not reachable");
			res.goal_feedback = "Goal Not Reachable"; // the User Interface node will receive "error" as feedback
		}
	}else{
		ROS_INFO("Action timed out.");
		res.goal_feedback = "Goal not reachable"; // the User Interface node will receive "error" as feedback
		ac.cancelGoal();
	}
	
	return true;

}

/**
*
* \brief min_right_side function, used to obtain the minimum value of the right side ranges
*
* \param r pointer to the ranges array
*
* \return min_r the minimum value of the right side ranges
*
* min_right_side is a function that takes as input parameter the ranges array
* ,computes the minimum value among the right lateral distances contained in 
* the right_side array and returns the minimum value as output.
*
**/
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

/**
*
* \brief min_left_side function, used to obtain the minimum value of the left side ranges
*
* \param r pointer to the ranges array
*
* \return min_l the minimum value of the left side ranges
*
* min_left_side is a function that takes as input parameter the ranges array
* ,computes the minimum value among the left lateral distances contained in 
* the left_side array and returns the minimum value as output.
*
**/
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

/**
*
* \brief min_front_side function, used to obtain the minimum value of the front side ranges
*
* \param r pointer to the ranges array
*
* \return min_f the minimum value of the front side ranges
*
* min_front_side is a function that takes as input parameter the ranges array
* ,computes the minimum value among the frontal distances contained in 
* the front_side array and returns the minimum value as output.
*
**/
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

/**
*
* \brief avoid_walls function, used to avoid that the robot crashes on the walls of the environment
*
* \param front the minimum value of the front side ranges
* \param left the minimum value of the left side ranges
* \param right the minimum value of the right side ranges
*
* \return
*
* avoid_walls is a function used to avoid that the robot crashes on the walls
* of the environment. It takes as input parameters the front, left and right minimum
* values, computed by the min_left_side, min_right_side amd min_front_side
* functions and using this values it checks if the robot crosses the threshold
* ,for instance, firstly it checks if the front min value is less than th, 
* if it isn't ,the robot go on ,otherwise it checks if the left min
* is less than the right min and vice versa. If the left min is less than the right min 
* it means that the nearest wall is on the left and so on.
*
**/
void avoid_walls(float front, float left, float right){
	
	//you are going forward
	if(front < th){

		new_vel.linear.x = -0.05;
		pubVel.publish(new_vel);
		cout<<"There is an obstacle in front of you"<<endl;

	}

	else if(right < th){

		new_vel.linear.x = -0.05;
		new_vel.angular.z = 0;
		pubVel.publish(new_vel);
		cout<<"There is an obstacle on the right"<<endl;

	}

	else if(left < th){

		new_vel.linear.x = -0.05;
		new_vel.angular.z = 0;
		pubVel.publish(new_vel);
		cout<<"There is an obstacle on the left"<<endl;

	}
}

/**
*
* \brief drivingAssistance function, used to implement the driving assistance when the robot is in the assisted drive mode
*
* \param msg laser scan data
*
* \return
*
* This function split the ranges data taken by the laser scan topic
* in three parts (front, left and right) and call the function 
* avoid_walls passing it these three data parts.
*
**/
void drivingAssistance(const sensor_msgs::LaserScan::ConstPtr& msg){

	if(assisted){
		for(int i = 0; i < 720 ; i++){
			ranges[i] = msg -> ranges[i]; 
		}
		
		float f = min_front_side(ranges);
		float l = min_left_side(ranges);
		float r = min_right_side(ranges);
		
		avoid_walls(f, l, r);
	}
}

/**
*
* @brief Main function
*
**/
int main(int argc, char** argv){

	ros::init(argc, argv, "controller");
		
	ros::NodeHandle n;
	
	ros::Subscriber subTeleVel = n.subscribe("/Velocity_control", 1000, velCallback); // teleop remapping

	//ros::Subscriber subPos = n.subscribe("/move_base/feedback", 1000, robotPosition);

	ros::Subscriber sub_laser = n.subscribe("/scan", 1000, drivingAssistance); // Laser scanner
	
	pubVel = n.advertise<geometry_msgs::Twist>("/cmd_vel",1000);

	pubCanc = n.advertise<actionlib_msgs::GoalID>("/move_base/cancel",1000);
	
	ros::ServiceServer service_goal = n.advertiseService("/goal_position", goalPosition);

	ros::ServiceServer service_interface = n.advertiseService("/commands", interface);

	// it blocks the main thread from exiting until ROS invokes a shutdown.

	ros::spin();
	
	
	return 0;
}
