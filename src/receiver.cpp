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
#include "geometry_msgs/Pose2D.h"
#include "ros/time.h"
#include <sstream>
#include <math.h>
#include <ros/console.h>
#include <iostream>
#include "camera.h"
#include "camera2.h"

using namespace std;

extern MavrosMessage message;
extern Camera camera_video;
extern Camera2 camera2_video;

geometry_msgs::Pose2D position_msg;

bool f_equal(float x, float y);

void MavrosMessage::run()
{
    //initialize value

    //接收以下话题
    ros::NodeHandle n;
   
    //ros::Subscriber sub17 = n.subscribe("/mavros/pump_status/pump_status", 200,chatterCallback_Pump_Status);

    //Publish Topic
    ros::Publisher position_pub = n.advertise<geometry_msgs::Pose2D>("robot_position", 30);
  
    ros::Rate send_loop_rate(30);

    while(ros::ok())
    {
        if(camera_video.robot_position_updated)
        {
            if(camera_video.camera_enemy_side && camera_video.robot_real_p[0] > -0.3)
            {
                position_msg.x = camera_video.robot_real_p[0];
                position_msg.y = camera_video.robot_real_p[1];
                position_pub.publish(position_msg);
                camera_video.robot_position_updated = false;
            }
            else if(!camera_video.camera_enemy_side && camera_video.robot_real_p[0] <= -0.3)
            {
                position_msg.x = camera_video.robot_real_p[0];
                position_msg.y = camera_video.robot_real_p[1];
                position_pub.publish(position_msg);
                camera_video.robot_position_updated = false;
            }
            else ;
        }

        if(camera2_video.robot_position_updated)
        {
            if(camera2_video.camera_enemy_side && camera2_video.robot_real_p[0] > -0.3)
            {
                position_msg.x = camera2_video.robot_real_p[0];
                position_msg.y = camera2_video.robot_real_p[1];
                position_pub.publish(position_msg);
                camera2_video.robot_position_updated = false;
            }
            else if(!camera2_video.camera_enemy_side && camera2_video.robot_real_p[0] <= -0.3)
            {
                position_msg.x = camera2_video.robot_real_p[0];
                position_msg.y = camera2_video.robot_real_p[1];
                position_pub.publish(position_msg);
                camera2_video.robot_position_updated = false;
            }
            else ;
        }


        ros::spinOnce();
        send_loop_rate.sleep();
    }

}


bool f_equal(float x, float y)
{
  if(fabs(x-y)<0.01) return true;
  else return false;
}


