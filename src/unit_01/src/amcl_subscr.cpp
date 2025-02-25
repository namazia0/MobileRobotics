// Includes to talk with ROS and all the other nodes
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>


// AMCL: Adaptive Monte Carlo Localization for finding the position of the robot (orientation and position of the robot)


// Class definition
class AMCL {
public:
	amcl();
	void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& m_estimate_data);
	void mainLoop();

protected:

	// Nodehandle for amcl robot
	ros::NodeHandle m_nodeHandle;

	// Subscriber and membervariables for our pose estimator
	ros::Subscriber m_poseSubscriber;
	geometry_msgs::Pose m_poseEstimate;

};

// constructor
amcl::amcl() {

	// Node will be instantiated in root-namespace
	ros::NodeHandle m_nodeHandle("/");

	// Initializing the node handle
	m_poseSubscriber  = m_nodeHandle.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 20, &amcl::poseCallback, this);

}// end of amcl constructor



// callback for getting pose estimates
void amcl::poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& estimateData) {
    m_poseEstimate = estimateData->pose.pose;
	
	double x,y,z,w,i,j,k = 0.0;

	x=m_poseEstimate.position.x;
	y=m_poseEstimate.position.y;
	z=m_poseEstimate.position.z;

	w=m_poseEstimate.orientation.w;
	i=m_poseEstimate.orientation.x;
	j=m_poseEstimate.orientation.y;
	k=m_poseEstimate.orientation.z;
	  
	ROS_INFO(" Position:=[%f, %f, %f], Orientation:=[%f, %f, %f, %f] \n", x, y, z, w, i, j, k ); 

}// end of poseCallback

void amcl::mainLoop(){
	// determines the number of loops per second
	ros::Rate loop_rate(20);

	// als long as all is O.K : run
	// terminate if the node get a kill signal
	while (m_nodeHandle.ok()){
		// spinOnce, just make the loop happen once
		ros::spinOnce();
		// and sleep, to be aligned with the 50ms loop rate
		loop_rate.sleep();

	}// end of if nodehandle O.K.

}// end of mainLoop


int main(int argc, char** argv){

	// initialize
	ros::init(argc, argv, "amcl");

	// get an object of type amcl and call it robbi
	AMCL robbi;

	// make robbi run
	// main loop
	robbi.mainLoop();

	return 0;
}