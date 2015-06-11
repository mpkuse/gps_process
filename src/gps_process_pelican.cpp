#include <iostream>
#include <string.h>
#include <ros/ros.h>
#include <armadillo>
#include <tf/tf.h>
#include <GeographicLib/LocalCartesian.hpp>
#include <sensor_msgs/Imu.h>
#include <rtkgps/PosVel.h>
#include <nav_msgs/Odometry.h>
#include <pose_utils.h>

using namespace std;

ros::NodeHandle *node;
ros::Publisher odomPub;

// stddev that can be set by user
double stdYprYaw   = 0;
double stdYprPitch = 0;
double stdYprRoll  = 0;

// Message storage
bool imuSet    = false;
bool posVelSet = false;
bool odomSet   = false;
bool posGPSSet = false;
sensor_msgs::Imu    imuMsg;
rtkgps::PosVel   posVelMsg;
nav_msgs::Odometry odomMsg;
GeographicLib::LocalCartesian posGPS;

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  imuSet = true;
  imuMsg = *msg;
}

void pos_vel_callback(const rtkgps::PosVel::ConstPtr& msg)
{
  posVelSet = true;
  posVelMsg = *msg;
}

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  odomSet = true;
  odomMsg = *msg;
}

void process_data()
{
  if (imuSet && posVelSet && odomSet)
  {
    imuSet = posVelSet = odomSet = false;

    // Position
    double _x = 0.0;
    double _y = 0.0; 
    double _z = 0.0, _h = 0.0;
    double lat = posVelMsg.latitude;
    double lon = posVelMsg.longitude;
    if (!posGPSSet)
    {
      posGPSSet = true;
      posGPS.Reset(lat, lon, _h);
    }
    posGPS.Forward(lat, lon, _h, _x, _y, _z);
    colvec xyz(3);
    xyz(0) = -_y;
    xyz(1) =  _x;
    xyz(2) = odomMsg.pose.pose.position.z;
    // Velocity 
    colvec dxyz(3);
    dxyz(0) = -posVelMsg.velocity.x;
    dxyz(1) =  posVelMsg.velocity.y;
    dxyz(2) =  odomMsg.twist.twist.linear.z;
    // Orientation
    colvec q(4);
    q(0) = imuMsg.orientation.w;
    q(1) = imuMsg.orientation.x;
    q(2) = imuMsg.orientation.y;
    q(3) = imuMsg.orientation.z;
    colvec ypr = R_to_ypr(quaternion_to_R(q));
    colvec mag(3);
    mag(0) = imuMsg.orientation_covariance[3];
    mag(1) = imuMsg.orientation_covariance[4];
    mag(2) = imuMsg.orientation_covariance[5];
    ypr(0) = atan2(mag(1), mag(0));
    q = R_to_quaternion(ypr_to_R(ypr));

    // Assemble odometry message, ignore angular velocity
    nav_msgs::Odometry odom;
    odom.header.stamp = posVelMsg.header.stamp;
    odom.pose.pose.position.x = xyz(0);
    odom.pose.pose.position.y = xyz(1);
    odom.pose.pose.position.z = xyz(2);
    odom.pose.covariance[0]   = posVelMsg.position_std[0] * posVelMsg.position_std[0] / 4.0;
    odom.pose.covariance[7]   = posVelMsg.position_std[1] * posVelMsg.position_std[1] / 4.0;
    odom.pose.covariance[14]  = odomMsg.pose.covariance[14];
    odom.pose.pose.orientation.w = q(0);
    odom.pose.pose.orientation.x = q(1);
    odom.pose.pose.orientation.y = q(2);
    odom.pose.pose.orientation.z = q(3);
    odom.pose.covariance[21]     = stdYprYaw*stdYprYaw;
    odom.pose.covariance[28]     = stdYprPitch*stdYprPitch;
    odom.pose.covariance[35]     = stdYprRoll*stdYprRoll;
    odom.twist.twist.linear.x = dxyz(0);
    odom.twist.twist.linear.y = dxyz(1);
    odom.twist.twist.linear.z = dxyz(2);
    odom.twist.covariance[0]  = posVelMsg.velocity_std[0] * posVelMsg.velocity_std[0] / 4.0;
    odom.twist.covariance[7]  = posVelMsg.velocity_std[1] * posVelMsg.velocity_std[1] / 4.0;
    odom.twist.covariance[14] = odomMsg.twist.covariance[14];
    // Determin whther GPS localization is valid or not
    if (sqrt(odom.pose.covariance[0]) < 1.0)
      odom.header.frame_id = string("/gps");    
    else
      odom.header.frame_id = string("/gps_fail");
    // Publish
    odomPub.publish(odom);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gps_process_pelican");
  ros::NodeHandle n("~");

  n.param("noise_std/orientation/yaw",   stdYprYaw,   20.0*PI/180);
  n.param("noise_std/orientation/pitch", stdYprPitch, 4.0*PI/180);
  n.param("noise_std/orientation/roll",  stdYprRoll,  4.0*PI/180);

  ros::Subscriber sub1 = n.subscribe("imu",         10, imu_callback);
  ros::Subscriber sub2 = n.subscribe("gps_pos_vel", 10, pos_vel_callback);
  ros::Subscriber sub3 = n.subscribe("odom_slam",   10, odom_callback);
  odomPub = n.advertise<nav_msgs::Odometry>("odom", 10);

  ros::Rate r(1000.0);
  while(n.ok())
  {
    ros::spinOnce();
    process_data();
    r.sleep();
  }

  return 0;
}
