#include "ros/ros.h"
#include "people_msgs/PositionMeasurementArray.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point.h"

/*  required packages
	git clone https://github.com/DLu/wu_ros_tools --branch hydro 
	git clone https://github.com/wg-perception/people --branch indigo-devel 
*/

ros::Publisher facepos_pub;
ros::Subscriber facepos_sub;
ros::Publisher facecloud_pub;
ros::Subscriber facecloud_sub;

void faceCallback(const people_msgs::PositionMeasurementArray& msg)
{	
	int len = sizeof(msg.people)/sizeof(people_msgs::PositionMeasurement);
	for (int person = 0; person < len; person++)
	{
		geometry_msgs::Point pt = msg.people[person].pos; 
		facepos_pub.publish(pt);
	} 
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
  facepos_pub = n.advertise<geometry_msgs::Point>("/face_detect_listener/face_positions", 1000);
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
