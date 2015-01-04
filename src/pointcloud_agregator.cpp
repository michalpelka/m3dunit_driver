// This is low-integrity driver. 
// This driver will publish only transform from 3dunit_link to 3d_unit_rot_laser
// Another node will agragate everything to pointclound.
// High integirty driver containts build-in cpp code laser driver. This Driver can be used only with SICK range finders.
// High integrate driver is meant for porting driver from ROS to another systems.

#include <iostream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <angles/angles.h>
#include <std_msgs/Float32.h>
sensor_msgs::PointCloud2 cloud;

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

    ROS_DEBUG("Recived scan, frame : %s", scan->header.frame_id.c_str());
    tf::StampedTransform transform;
    try{

      tflistener->waitForTransform(rootTransform, scan->header.frame_id,scan->header.stamp, ros::Duration(0.5));
      tflistener->lookupTransform(rootTransform, scan->header.frame_id ,scan->header.stamp, transform);
    }
    catch (tf::TransformException ex){
         ROS_ERROR("%s",ex.what());
       }
    actual = transform.getRotation();
    float dd = actual.angleShortestPath(old);
    if (!isnan(dd))distance=distance + dd ;
    for (int i=0; i < scan->ranges.size(); i++)
    {
        tf::Vector3 pointOut;
        float ang = scan->angle_min+i*scan->angle_increment;
        float dist = scan->ranges[i];
        float intensity = 0;
        if (i <scan->intensities.size())
        {
            intensity = 0.01* scan->intensities[i];
        }
        tf::Vector3 pointIn (cos(ang)*dist,
                             sin(ang)*dist,
                             0);
        pointOut = transform*pointIn;

        {
            //geometry_msgs::Point32 p;
            //p.x=pointOut.x();
            //p.y=pointOut.y();
            //p.z=pointOut.z();
            //cloud.points.push_back(p);
            //cloud.channels[0].values.push_back(intensity);
            float _x = pointOut.x();
            float _y = pointOut.y();
            float _z = pointOut.z();

            int offset = cloud.data.size();
            cloud.data.resize(cloud.data.size()+cloud.point_step);
            memcpy (&cloud.data[offset],&_x, sizeof(float));
            memcpy (&cloud.data[offset+sizeof(float)],&_y, sizeof(float));
            memcpy (&cloud.data[offset+2*sizeof(float)],&_z, sizeof(float));
            memcpy (&cloud.data[offset+3*sizeof(float)],&intensity, sizeof(float));

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
    cloud.data.clear();
    profileCount=0;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "m3d_agg");
  sleep(2);
  ros::NodeHandle n("~");
  rootTransform = "m3d_link";
  tf::TransformListener tflistener_;
  tflistener = &tflistener_;
  ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud2>("cloud", 1);
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

    // set fields names
    cloud.fields.resize(4);
    cloud.fields[0].name="x";
    cloud.fields[1].name="y";
    cloud.fields[2].name="z";
    cloud.fields[3].name="intesity";
    //set fieldTypes
    cloud.fields[0].offset =0;
    cloud.fields[1].offset =4;
    cloud.fields[2].offset =8;
    cloud.fields[3].offset =12;

    cloud.point_step=16;

    cloud.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    cloud.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    cloud.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    cloud.fields[3].datatype = sensor_msgs::PointField::FLOAT32;

    cloud.fields[0].count  = 1;
    cloud.fields[1].count  = 1;
    cloud.fields[2].count  = 1;
    cloud.fields[3].count  = 1;
    cloud.is_bigendian = false;
    cloud.is_dense =false;




    ROS_DEBUG("distance :%f, pointcloud size: %d",distance,cloud.data.size());

    std_msgs::Bool doneMsg;
    std_msgs::Float32 progress;
    progress.data= 100.0*distance /(1.1*M_PI);

    if (creatingPointCloud  && progress.data > 100.0 )
    {
        doneMsg.data = true;
        done_pub.publish(doneMsg);
        ROS_INFO("Publish pointcloud");
        creatingPointCloud = false;
        cloud.height =1;
        cloud.width = cloud.data.size()/cloud.point_step;
        cloud_pub.publish(cloud);
        cloud.data.clear();
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
