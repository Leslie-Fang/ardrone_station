//*******receiver.cpp**********

#include "receiver.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <math.h>
#include "std_msgs/Float64.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/TwistStamped.h"
#include "ros/time.h"
#include <sstream>
#include <math.h>
#include <ros/console.h>
#include <iostream>

using namespace std;

extern MavrosMessage message;

bool f_equal(float x, float y);

void MavrosMessage::run()
{
    //initialize value


    //接收以下话题
    ros::NodeHandle n;
   
    //ros::Subscriber sub17 = n.subscribe("/mavros/pump_status/pump_status", 200,chatterCallback_Pump_Status);

    //Publish Topic
    //ros::Publisher offboard_setpoint_pub = n.advertise<mavros_extras::OffboardRoutePoints>("offboard_route_points", 30);
  
    ros::Rate check_loop_rate(4);

    while(ros::ok())
    {

        ros::spinOnce();
        check_loop_rate.sleep();
    }

}


bool f_equal(float x, float y)
{
  if(fabs(x-y)<0.01) return true;
  else return false;
}


