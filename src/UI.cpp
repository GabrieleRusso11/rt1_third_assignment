#include <ros/ros.h>
#include <RT1_third_assignment/Goal.h>
#include <RT1_third_assignment/Interface.h>

using namespace std;

int main(int argc, char** argv){

	ros::init(argc, argv, "user_interface");

    ros::NodeHandle n;

    ros::ServiceClient client_goal = n.serviceClient<RT1_third_assignment::Goal>("/goal_position");

    ros::ServiceClient client_interface = n.serviceClient<RT1_third_assignment::Interface>("/commands");

    RT1_third_assignment::Goal pos;

    RT1_third_assignment::Interface com;

    while(ros::ok()){
        int command = 0;
        cout<<"choose the modality : "<<endl;
        cout<<" * auotonomus drive -> press 1 "<<endl;
        cout<<" * manual drive -> press 2 "<<endl;
        cin>>command;

        if(command == 1){
            
            cout<<"Insert the x position"<<endl;
            cin>>pos.request.x;

            cout<<"Insert the y position"<<endl;
            cin>>pos.request.y;

            client_goal.waitForExistence();
            client_goal.call(pos);

            if(pos.response.feedback = 1){
                cout<<"ERROR (elapsed Timeout) : check if the coordinate are reachable and retry with the feasible ones."<<endl;
            }
            else{
                cout<<"OKAY : The Goal is Achieved."<<endl;
            }
        }
        else if(command == 2 ){
            
            cout<<"Now You are in the manual drive mode, You can use teleop to conntrol the robot."<<endl;
            com.request.command = 'm';

        }
    }
    
    ros::spin();

    return 0;

}