/*
 * hector_affw.cpp
 *
 *  Created on: 16.05.2016
 *      Author: NicolaiO
 */

#include <affw_msgs/ActionRequest.h>
#include <affw_msgs/State.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/service_client.h>
#include <ros/subscriber.h>
#include <rosconsole/macros_generated.h>
#include <std_msgs/Header.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <string>
#include <vector>

ros::Publisher pub_set_vel;
ros::Publisher pub_fdbk_state;
ros::ServiceClient srv_action;
ros::Time lastSetVelTime;

tf::TransformListener* listener = NULL;

nav_msgs::Odometry lastOdom;
bool lastOdomReceived = false;

bool usePose = false;

//void setVelCallback(const geometry_msgs::TwistStamped::ConstPtr& vel) {
void setVelCallback(const geometry_msgs::Twist::ConstPtr& vel) {

	lastSetVelTime = ros::Time::now();

	affw_msgs::ActionRequest srv;
  //srv.request.state.header = vel->header;
  srv.request.state.header.frame_id = "base_link";
  srv.request.state.header.stamp = ros::Time::now();
  srv.request.state.vel.push_back(vel->linear.x);
  srv.request.state.vel.push_back(vel->angular.z);

	if(usePose)
	{
    ROS_INFO("usePose");
		if(!lastOdomReceived)
			return;
		tf::Quaternion q;
		tf::quaternionMsgToTF(lastOdom.pose.pose.orientation, q);
		tf::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);

		srv.request.state.custom.push_back(roll);
		srv.request.state.custom.push_back(pitch);
	}

	if (srv_action.call(srv)) {
		geometry_msgs::Twist outVel;
		outVel.linear.x = srv.response.outVel[0];
		outVel.angular.z = srv.response.outVel[1];

    ROS_INFO("lin: outvel: %f, target: %f, diff: %f", outVel.linear.x, vel->linear.x, vel->linear.x - outVel.linear.x);
    ROS_INFO("ang: outvel: %f, target: %f, diff: %f", outVel.angular.z, vel->angular.z, vel->angular.z - outVel.angular.z);

		pub_set_vel.publish(outVel);
		ros::spinOnce();
	} else {
		ROS_ERROR_THROTTLE(1, "Failed to get action from affw");
	}
}

void feedbackVelCallback(const nav_msgs::Odometry::ConstPtr& odom) {

  geometry_msgs::Vector3Stamped velIn_lin;
  geometry_msgs::Vector3Stamped velIn_ang;
  velIn_lin.header = odom->header;
  velIn_lin.vector = odom->twist.twist.linear;
  velIn_ang.header = odom->header;
  velIn_ang.vector = odom->twist.twist.angular;

  try{
    ROS_INFO("Wait: %s", odom->header.frame_id.c_str());
    listener->waitForTransform("base_link", odom->header.frame_id, odom->header.stamp, ros::Duration(3.0));
    //listener.waitForTransform("world", "base_link", odom->header.stamp, ros::Duration(3.0));
    listener->transformVector("base_link", velIn_lin, velIn_lin);
    listener->transformVector("base_link", velIn_ang, velIn_ang);
    ROS_INFO("END_WAIT");
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }

//	geometry_msgs::Vector3 velIn = odom->twist.twist.linear;
	affw_msgs::State state;
	state.header = odom->header;
	state.header.frame_id = "base_link";
  state.vel.push_back(velIn_lin.vector.x);
  state.vel.push_back(velIn_ang.vector.z);

	pub_fdbk_state.publish(state);
	ros::spinOnce();
}

void feedbackOdomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
	lastOdom = *odom;
	lastOdomReceived = true;
}

void timerCallback(const ros::TimerEvent&)
{
	ros::Duration diff = ros::Time::now() - lastSetVelTime;
	if(diff.toSec() > 0.2 && diff.toSec() < 0.4)
	{
		geometry_msgs::Twist outVel;
		pub_set_vel.publish(outVel);
		ros::spinOnce();
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "hector_affw");
	ros::NodeHandle n;

  listener = new(tf::TransformListener);

  ROS_INFO("CHECK");
	lastSetVelTime = ros::Time::now();

	std::string cmd_vel_topic = "/cmd_vel_raw";
  std::string state_topic = "/odom";
	std::string odom_topic = "/odom";
	ros::param::get("cmd_vel_topic", cmd_vel_topic);
	ros::param::get("state_topic", state_topic);
	ros::param::get("odom_topic", odom_topic);

	ros::param::get("usePose", usePose);

	// unreliable transport
	ros::TransportHints th;
	th.unreliable();

	// send velocity to robot
	pub_set_vel = n.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);

	// send robot state to affw
	pub_fdbk_state = n.advertise<affw_msgs::State>("/affw_ctrl/state", 1);

	// receive velocity cmd from external source
  ros::Subscriber sub_set_vel = n.subscribe("/affw_ctrl/target_vel", 1,
			setVelCallback, th);

	// receive robot state from robot
	ros::Subscriber sub_fdbk_vel = n.subscribe(state_topic, 1,
			feedbackVelCallback, th);
	ros::Subscriber sub_fdbk_odom = n.subscribe(odom_topic, 1,
			feedbackOdomCallback, th);

	srv_action = n.serviceClient<affw_msgs::ActionRequest>("/affw_ctrl/action");

	ros::Timer timer = n.createTimer(ros::Duration(0.1), timerCallback);

	ros::spin();

	return 0;
}
