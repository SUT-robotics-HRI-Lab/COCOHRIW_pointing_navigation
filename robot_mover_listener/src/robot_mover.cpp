#include "ros/ros.h"
#include "robot_mover_listener/AngleDistance.h" // Replace 'your_package_name' with your package name

//Create veci:
//#include "Serial.h"
#include "iRobot_Create/Roomba.h"
//#include "Tserial_event.h"

#include <boost/bind.hpp>



// Callback function for when a new message arrives

//void angleDistanceCallback(const robot_mover_listener::AngleDistance::ConstPtr& msg, Roomba& Roomba_)
void angleDistanceCallback(const robot_mover_listener::AngleDistance::ConstPtr& msg, Roomba& iRobot) {
    ROS_INFO("Received message - Angle1: %f deg, Distance: %f mm, Angle2: %f deg", msg->angle1, msg->distance, msg->angle2);
    ROS_INFO("Driving!!!");

    iRobot.rotateAngle(-msg->angle1);
    iRobot.driveDistance(300,300,msg->distance);
    iRobot.rotateAngle(-msg->angle2);

}

int main(int argc, char **argv) {


          //inicializacie seriaku
    ROS_INFO("Received message");
    Roomba iRobot;
    //ROS_INFO("Received message2");
    std::cout<<"helou"<<std::endl;
		    iRobot.initRoomba("/dev/ttyUSB0");


  // Initialize the ROS node
    ros::init(argc, argv, "robot_mover");

    //param test
    /*
    while(1)
    {
        ros::param::set("/robot_drive_lock", true);
        ROS_INFO("ROBOT DRIVE LOCK: LOCKED");
        sleep(5);
        ros::param::set("/robot_drive_lock", false);
        ROS_INFO("ROBOT DRIVE LOCK: OPENED");
        sleep(5);
    }*/

    // Create a node handle
    ros::NodeHandle nh;

    // Subscribe to the 'angle_distance_topic' topic with a queue size of 1000
    //ros::Subscriber sub = nh.subscribe("/angle_distance_topic", 1000, angleDistanceCallback);

    ros::Subscriber sub = nh.subscribe<robot_mover_listener::AngleDistance>(
         "/angle_distance_topic",
         1000,
         boost::bind(&angleDistanceCallback, _1, boost::ref(iRobot)) // Binding message and Roomba_ reference
     );

    // Spin to keep the node running and process incoming messages
    ros::spin();

    return 0;
}
