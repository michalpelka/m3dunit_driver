// This is low-integrity driver.
// This driver will publish only transform from 3dunit_link to 3d_unit_rot_laser
// Another node will agragate everything to pointclound.
// High integirty driver containts build-in cpp code laser driver. This Driver can be used only with SICK range finders.
// High integrate driver is meant for porting driver from ROS to another systems.

#include <iostream>
#include "driverLib.hpp"
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float32.h>
driver_m3d m3d;
void positionCallbac(const std_msgs::Float32ConstPtr& msg)
{
    m3d.setPosition(msg->data,25,1);
}

void velocityCallback(const std_msgs::Float32ConstPtr& msg)
{
    m3d.setSpeed(msg->data);
}
int main(int argc, char** argv)
{


    ros::init(argc, argv, "m3d_driver");
    ros::NodeHandle node ("~");
	tf::TransformBroadcaster m3dRot;
    ros::Publisher angPub = node.advertise<std_msgs::Float32>("ang", 10);
    ros::Subscriber s1 =node.subscribe("velocity", 1, velocityCallback);
    ros::Subscriber s2 =node.subscribe("position", 1, positionCallbac);

            //encoderSocket e;

    std::string ip = "192.168.0.150";
    double encoderOffset = 0;

    node.getParam("encoderOffset", encoderOffset);


    if (!node.getParam("ip", ip))
    {
        ROS_FATAL("what is m3d_unit ip? set ~/ip!!");

    }

	ROS_INFO_ONCE("connecting");


    m3d.connect_to_m3d(ip);
	ROS_INFO_ONCE("connected");

    tf::Transform front_laserLink;

    front_laserLink.setIdentity();

    front_laserLink.setOrigin(tf::Vector3(0, 0.0285, 0.04));



	while (node.ok())
	{
		tf::Transform transform;
        transform.setOrigin(tf::Vector3(0, -0.0835, 0.1835));
		tf::Quaternion q;
		//int ang = e.readActualAng();

		if (true)
		{
			ROS_INFO_ONCE("Started to get data");
            int _value;

            float ang = m3d.getAngle();

            ROS_DEBUG("ang %f",ang);
            q.setRPY(0,-M_PI_2,ang+encoderOffset);
            q.normalize();
            std_msgs::Float32 f;
            f.data= ang;
            angPub.publish(f);
            transform.setRotation(q);
            m3dRot.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "m3d_link", "m3d_rot_laser_link"));
            m3dRot.sendTransform(tf::StampedTransform(front_laserLink, ros::Time::now(), "m3d_link", "m3d_front_laser_link"));

            ros::spinOnce();
        }
		
	}
    m3d.setPosition(0,20,1);
}
