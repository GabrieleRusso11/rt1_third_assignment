#include <ros/ros.h>
#include <iostream>
#include <rt1_third_assignment/Goal.h>
#include <rt1_third_assignment/Interface.h>

using namespace std;

int main(int argc, char** argv){

	ros::init(argc, argv, "user_interface");

    ros::NodeHandle n;

    ros::ServiceClient client_goal = n.serviceClient<rt1_third_assignment::Goal>("/goal_position");

    ros::ServiceClient client_interface = n.serviceClient<rt1_third_assignment::Interface>("/commands");

    rt1_third_assignment::Goal pos;

    rt1_third_assignment::Interface com;

    while(ros::ok()){

        int command = 0;
        cout<<"************************************************"<<endl;
        cout<<endl<<"choose the modality : "<<endl;
        cout<<endl<<" * auotonomus drive -> press 1 "<<endl;
        cout<<endl<<" * manual drive -> press 2 "<<endl;
        cout<<endl<<" * driving assistance -> press 3"<<endl;
        cout<<endl<<" * cancel the goal -> press 4"<<endl;
        cout<<endl<<" * Exit -> press 0 "<<endl;
        cout<<endl<<"************************************************"<<endl;
        cin>>command;

        if(command == 1){  // the user has chosen the auto drive mode
            
            com.request.command = 'a';
            client_interface.waitForExistence();
            client_interface.call(com);
            cout<<"Insert the x position"<<endl;
            cin>>pos.request.x;

            cout<<"Insert the y position"<<endl;
            cin>>pos.request.y;

            client_goal.waitForExistence();
            client_goal.call(pos);

            cout<<"the goal position is sent to the controller"<<endl;

        }
        else if(command == 2 ){ // the user has chosen the manual drive mode
            
            cout<<"Now You are in the manual drive mode, You can use teleop to conntrol the robot."<<endl;
            com.request.command = 'm';
            client_interface.waitForExistence();
            client_interface.call(com);
             

        }
        else if(command == 3){ // the user has chosen the assisted drive mode

            cout<<"Now You are in the assisted drive mode"<<endl;
            com.request.command = 'd';
            client_interface.waitForExistence();
            client_interface.call(com);
             

        }
        else if(command == 4){ // the user has chosen to cancel its desired goal given in the auto drive mode

            com.request.command = 'c';
            client_interface.waitForExistence();
            client_interface.call(com);

        }
        else if(command == 0){ // the user has chosen to exit from the user interface

            exit(0);

        }
        else{

            cout<<endl<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
            cout<<"You have pressed the wrong button, please be careful."<<endl;
            cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;

        }

        
    }
    
    ros::spin();

    return 0;

}