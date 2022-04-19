#include <ros/ros.h>

// this line includes the Action Specification for move_base which is a ROS action that exposes 
// a high level interface to the navigation stack.
// The move_base action accepts goals from clients and attempts to move the robot to the
// specified position/orientation in the world.
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <rt1_third_assignment/Goal.h>
#include <rt1_third_assignment/Velocity_control.h>
#include <rt1_third_assignment/Interface.h>
#include <geometry_msgs/Twist.h>

using namespace std;

// this line creates a convenience typedef for a SimpleActionClient that will allow us to communicate
// with actions that adhere to the MoveBaseAction action interface

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

double timeout = 120.0;
int TeleopVel;
bool manual = false;

void velCallback(const rt1_third_assignment::Velocity_control::ConstPtr & msg){

		TeleopVel = msg->vel;

}

bool interface(rt1_third_assignment::Interface::Request &req, rt1_third_assignment::Interface::Response &res){

	if(req.command == 'm'){
		manual = true;
	}

	return true;
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
		goal.target_pose.header.stamp = ros::Time::now();
	
		goal.target_pose.pose.orientation.w = 1.0;
		goal.target_pose.pose.position.x = req.x;
		goal.target_pose.pose.position.y = req.y;

		// this line wait for the action server to report that it has come up and is ready to begin
		// processing goals

		// wait for the action server to come up
		while(!ac.waitForServer(ros::Duration(5.0))){
			ROS_INFO("Waiting for the move_base action server to come up");
		}
	

		
		ROS_INFO("Sending goal");
		ac.sendGoal(goal);
		
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
		return true;
	}

int main(int argc, char** argv){

	ros::init(argc, argv, "controller");
		
	ros::NodeHandle n;
	
	ros::Subscriber subTeleVel = n.subscribe("rt1_third_assignment/Velocity_control", 1000, velCallback);

	ros::Publisher pubVel = n.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
	// server of the /velocity_control custom service
	ros::ServiceServer service_goal = n.advertiseService("/goal_position", goalPosition);

	ros::ServiceServer service_interface = n.advertiseService("/commands", interface);

	while(ros::ok()){

		if(manual == true){

      		//pubVel.publish(TeleopVel);

		}

	}
	// it blocks the main thread from exiting until ROS invokes a shutdown.
	ros::spin();

	

	
	
	return 0;
}
