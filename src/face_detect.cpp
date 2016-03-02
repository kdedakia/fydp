#include "ros/ros.h"
#include "people_msgs/PositionMeasurementArray.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"

/*  required packages
	git clone https://github.com/DLu/wu_ros_tools --branch hydro 
	git clone https://github.com/wg-perception/people --branch indigo-devel 
*/

ros::Publisher facepos_pub;
ros::Subscriber facepos_sub;
ros::Publisher facecloud_pub;
ros::Subscriber facecloud_sub;

double dist(geometry_msgs::Point pt)
{
	return sqrt(pow(pt.x,2) + pow(pt.y,2) + pow(pt.z,2));
}

void faceCallback(const people_msgs::PositionMeasurementArray& msg)
{	
	int len = sizeof(msg.people)/sizeof(people_msgs::PositionMeasurement);
	geometry_msgs::Point pt;
	double currFace, closestFace = -1;
	for (int person = 0; person < len; person++)
	{
		currFace = dist(msg.people[person].pos);
		if ((currFace < closestFace) || (closestFace == -1))
		{
			closestFace = currFace;
			pt = msg.people[person].pos; 
		}
	} 
	
	geometry_msgs::Quaternion qn = tf::createQuaternionMsgFromYaw(atan2(pt.y, pt.x));
	geometry_msgs::Pose pt_pose; pt_pose.position = pt;	pt_pose.orientation = qn;
	geometry_msgs::PoseStamped ps; ps.pose = pt_pose; 
	facepos_pub.publish(ps);
}

void facecloudCallback(const sensor_msgs::PointCloud& msg)
{
	facecloud_pub.publish(msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "face_detect_listener");
  
  ros::NodeHandle n;
  
  // face position
  facepos_pub = n.advertise<geometry_msgs::PoseStamped>("/face_detect_listener/face_positions", 1000);
  facepos_sub = n.subscribe("/face_detector/people_tracker_measurements_array", 1000, faceCallback);
  
  // face point clouds
  facecloud_pub = n.advertise<sensor_msgs::PointCloud>("/face_detect_listener/faces_cloud", 1000);
  facecloud_sub = n.subscribe("/face_detector/faces_cloud", 1000, facecloudCallback);
  
  ros::Rate loop_rate(10);
  
  while (ros::ok())
  {
  	ros::spinOnce();
  	loop_rate.sleep(); 
  }
  return 0; 
}
