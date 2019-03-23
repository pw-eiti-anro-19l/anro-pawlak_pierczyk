#include "geometry_msgs/Twist.h"
#include "ros/ros.h"

#include <iostream>
#include <termios.h>
#include <stdlib.h>


/* Simple non-blocking console mechanism */
void SetKeyboardNonBlock(struct termios *initial_settings)
{
    struct termios new_settings;
    tcgetattr(0,initial_settings);

    new_settings = *initial_settings;
    new_settings.c_lflag &= ~ICANON;
    new_settings.c_lflag &= ~ECHO;
    new_settings.c_lflag &= ~ISIG;
    new_settings.c_cc[VMIN] = 0;
    new_settings.c_cc[VTIME] = 0;

    tcsetattr(0, TCSANOW, &new_settings);
}

/* Restoring a console to the initial state */
void RestoreKeyboardBlocking(struct termios *initial_settings)
{
	tcsetattr(0, TCSANOW, initial_settings);
}


int main(int argc, char **argv){
    ros::init(argc, argv, "turtle_controller"); 
    ros::NodeHandle handler;

    /* Parameters names */
    std::string forward_param = "go_forward_key";
    std::string backward_param = "go_backward_key";
    std::string right_param = "turn_right_key";
    std::string left_param = "turn_left_key";

    /* Default parameters values */
    std::string forward("w");
    std::string backward("s");
    std::string right("d");
    std::string left("a");
    
    /* Initializing parameters */
    handler.setParam(forward_param, forward);
    handler.setParam(backward_param, backward);
    handler.setParam(right_param, right);
    handler.setParam(left_param, left);

    /* Topic publisher initialization */
    ros::Publisher controll_pub =
        handler.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);

    //A structure enabling setting non-blocking console input
    struct termios term_settings;


    /* Quick manual displayed */
    std::cout << "Reading from keyboard" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    std::cout << std::endl;
    std::cout << "Use WSAD keys to move the turtle." << std::endl;
    std::cout << "By changing values of four parameters:" << std::endl;
    std::cout << "  -go_forward_key" << std::endl;
    std::cout << "  -go_backward_key" << std::endl;
    std::cout << "  -turn_right_key" << std::endl;
    std::cout << "  -turn_left_key" << std::endl;
    std::cout << "you are able to change control keys." << std::endl;
    std::cout << std::endl;
    std::cout << "----------------------------------------" << std::endl;


    /* Hiding user's input in the console */
    termios oldt;
    tcgetattr(STDIN_FILENO, &oldt);
    termios newt = oldt;
    newt.c_lflag &= ~ECHO;
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    //Reffresing freq to 20Hz
    ros::Rate loop_rate(20); 
    while(ros::ok()){

        /* Setting non-blocking input */
        SetKeyboardNonBlock(&term_settings);

        //Input key
        std::string key = std::string(1, (char)getchar());
        
        /* Setting blocking input */
        RestoreKeyboardBlocking(&term_settings);

        /* Looking for params changes */
        if(handler.hasParam(forward_param))
            handler.getParam(forward_param, forward);
        if(handler.hasParam(backward_param))
            handler.getParam(backward_param, backward);
        if(handler.hasParam(right_param))
            handler.getParam(right_param, right);
        if(handler.hasParam(left_param))
            handler.getParam(left_param, left);

        //Potential message
        geometry_msgs::Twist msg;

        /* Preparing an appropriate message. */
        if(key == (forward)){
            msg.linear.x = 2;
            controll_pub.publish(msg);
        }
        else if(key == backward){
            msg.linear.x = -2;
            controll_pub.publish(msg);
        }
        else if(key == right){
            msg.angular.z = -2;
            controll_pub.publish(msg);
        }
        else if(key == left){
            msg.angular.z = 2;
            controll_pub.publish(msg);
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    /* Restoring user input dislay */
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    /* Cleaning params */
    handler.deleteParam(forward_param);
    handler.deleteParam(backward_param);
    handler.deleteParam(right_param);
    handler.deleteParam(left_param);
}
