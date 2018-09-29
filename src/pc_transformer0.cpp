/*
 *	pc_transformer.cpp
 */

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

sensor_msgs::PointCloud2 pc2_out;
tf::TransformListener tflistener;

void callback_pc(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	// std::cout << "- CLOUD CALLBACK -" << std::endl;
	std::cout << "frame_id = " << msg->header.frame_id << std::endl;
	sensor_msgs::PointCloud tmp_pc;
	sensor_msgs::convertPointCloud2ToPointCloud(*msg, tmp_pc);
	std::cout << "tmp_pc.header.stamp = " << tmp_pc.header.stamp << std::endl;
	try{
		tflistener.waitForTransform(msg->header.frame_id, "/localmap", msg->header.stamp, ros::Duration(1.0));
		tflistener.transformPointCloud(msg->header.frame_id, msg->header.stamp, tmp_pc, "/localmap", tmp_pc);
		
		// tflistener.waitForTransform(msg->header.frame_id, "/odom", ros::Time(0), ros::Duration(1.0));
		// tflistener.transformPointCloud(msg->header.frame_id, ros::Time(0), tmp_pc, "/odom", tmp_pc);
	}
	catch(tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}
	sensor_msgs::convertPointCloudToPointCloud2(tmp_pc, pc2_out);
	std::cout << "pc2_out.frame_id = " << pc2_out.header.frame_id << std::endl;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pc_transformer");
	/*nodehundle*/
	ros::NodeHandle nh;
	/*pub*/
	ros::Publisher pub_pc_transformed = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points/transformed", 1);
	/*sub*/
	ros::Subscriber sub_pc = nh.subscribe("/velodyne_points", 1, callback_pc);
	/*loop*/
	ros::Rate loop_rate(20);
	while(ros::ok()){
		ros::spinOnce();
		if(!pc2_out.data.empty())	pub_pc_transformed.publish(pc2_out);
		loop_rate.sleep();
	}
	// ros::spin();
}
