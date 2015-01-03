// This is low-integrity driver. 
// This driver will publish only transform from 3dunit_link to 3d_unit_rot_laser
// Another node will agragate everything to pointclound.
// High integirty driver containts build-in cpp code laser driver. This Driver can be used only with SICK range finders.
// High integrate driver is meant for porting driver from ROS to another systems.

#include <iostream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <angles/angles.h>
#include <std_msgs/Float32.h>
sensor_msgs::PointCloud cloud;

tf::TransformListener* tflistener;
bool creatingPointCloud;

float distance;
tf::Quaternion actual;
tf::Quaternion begin;
tf::Quaternion old;

int profileCount =0;
 std::string rootTransform;
void scanCallback(const sensor_msgs::LaserScanPtr& scan )
{
   ;

    ROS_DEBUG("Recived scan, frame : %s", scan->header.frame_id.c_str());
    tf::StampedTransform transform;
    try{

      tflistener->waitForTransform(rootTransform, scan->header.frame_id,scan->header.stamp, ros::Duration(0.2));
      tflistener->lookupTransform(rootTransform, scan->header.frame_id ,scan->header.stamp, transform);
    }
    catch (tf::TransformException ex){
         ROS_ERROR("%s",ex.what());
       }
    actual = transform.getRotation();

    distance=distance + actual.angleShortestPath(old) ;
    for (int i=0; i < scan->ranges.size(); i++)
    {
        tf::Vector3 pointOut;
        float ang = scan->angle_min+i*scan->angle_increment;
        float dist = scan->ranges[i];
        tf::Vector3 pointIn (cos(ang)*dist,
                             sin(ang)*dist,
                             0);
        pointOut = transform*pointIn;
        {
            geometry_msgs::Point32 p;
            p.x=pointOut.x();
            p.y=pointOut.y();
            p.z=pointOut.z();
            cloud.points.push_back(p);
        }
        if (profileCount>20)
        {
            int indexP = cloud.points.size()-10*scan->ranges.size();
            for (int j =0; j <scan->ranges.size(); j++)
            {

            }
        }

    }
    old = actual;
    profileCount ++;
}

void requestCallback(const std_msgs::BoolPtr& req)
{

    if (req->data == false) return;
    if (creatingPointCloud == true) return;
    ROS_INFO("Recived request");
    begin = actual;
    old = actual;
    distance = 0;
    creatingPointCloud = true;
    cloud.points.clear();
    profileCount=0;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "m3d_agg");
  sleep(2);
  ros::NodeHandle n("~");
  rootTransform = "m3d_link";
  tf::TransformListener tflistener_;
  tflistener = &tflistener_;
  ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud>("cloud", 1);
  ros::Publisher progress_pub = n.advertise<std_msgs::Float32>("progress", 1);
  ros::Publisher done_pub = n.advertise<std_msgs::Bool>("done", 1);

  ros::Subscriber cloud_req = n.subscribe("rqst", 1, requestCallback);
  ros::Subscriber scanner_las = n.subscribe("rotLaserScan", 1000, scanCallback);

  n.getParam("root_frame", rootTransform);


  int count = 0;

  creatingPointCloud=false;
  ros::Rate rate(10.0);
  double maxDist= 1.0*M_PI;
  while(n.ok()){
	  
	  
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "m3d_link";

    //we'll also add an intensity channel to the cloud
    //cloud.channels.resize(1);
    //cloud.channels[0].name = "intensities";
    //cloud.channels[0].values.resize(num_points);


    ROS_DEBUG("distance :%f, %d",distance,cloud.points.size());

    std_msgs::Bool doneMsg;
    std_msgs::Float32 progress;
    progress.data= 100.0*distance /(1.1*M_PI);

    if (creatingPointCloud  && progress.data > 100.0 )
    {
        doneMsg.data = true;
        done_pub.publish(doneMsg);
        ROS_INFO("Publish pointcloud");
        creatingPointCloud = false;
        cloud_pub.publish(cloud);
        cloud.points.clear();
        begin = actual;
        std_msgs::Float32 progress;
        progress.data= 100.0*distance /maxDist;
        progress_pub.publish(progress);
        distance = 0;
        profileCount =0;
    }
    if (creatingPointCloud  && progress.data < 100.0)
    {

        progress_pub.publish(progress);
        doneMsg.data = false;
        done_pub.publish(doneMsg);
    }
    ++count;
    ros::spinOnce();
    rate.sleep();
  }
}
