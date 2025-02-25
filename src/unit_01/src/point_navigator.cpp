// Includes to talk with ROS and all the other nodes
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

// Class definition
class Just_move {
public:
	Just_move();
	void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& m_estimate_data);
	void emergencyStop();
	void calculateCommand();
	void mainLoop();

protected:

	// Nodehandle for Just_move robot
	ros::NodeHandle m_nodeHandle;

	// Subscriber and membervariables for our pose estimator
	ros::Subscriber m_poseSubscriber;
	geometry_msgs::Pose m_poseEstimate;

	// Publisher and membervariables for the driving commands
	ros::Publisher m_commandPublisher;
	geometry_msgs::Twist m_roombaCommand;

};

// constructor
Just_move::Just_move() {

	// Node will be instantiated in root-namespace
	ros::NodeHandle m_nodeHandle("/");

	// Initializing the node handle
	m_poseSubscriber  = m_nodeHandle.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 20, &Just_move::poseCallback, this);
	m_commandPublisher = m_nodeHandle.advertise<geometry_msgs::Twist> ("cmd_vel", 20);

}// end of Just_move constructor

// robot shall stop, in case anything is closer than ... 
void Just_move::emergencyStop() {

        // see if we have laser data
 	if( (&m_laserscan)->ranges.size() > 0) 
	{
		for(unsigned int i=0; i < (&m_laserscan)->ranges.size(); i++)
		{
			if( m_laserscan.ranges[i] <= 0.30) {
				m_roombaCommand.linear.x = 0.0;
				m_roombaCommand.angular.z = 0.0;
			}// end of if too close
		}// end of for all laser beams
	} // end of if we have laser data
}// end of emergencyStop

// callback for getting pose estimates
void Just_move::poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& estimateData) {
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

void Just_move::calculateCommand(double goalX, double goalY) {

	// set the roomba velocities
	// the linear velocity (front direction is x axis) is measured in m/sec
	// the angular velocity (around z axis, yaw) is measured in rad/sec
	m_roombaCommand.linear.x  = 0.0;
	m_roombaCommand.angular.z = 0.0;

	double xCurr = m_poseEstimate.position.x;
	double yCurr = m_poseEstimate.position.y;
	double aCurr = m_poseEstimate.orientation.w;

	double deltaX = goalX-xCurr;
	double deltaY = goalY-yCurr;

	double angleGoal = atan2(deltaY,deltaX)
	double deltaA = angleGoal-aCurr;

	if (deltaA>0.1){
		m_roombaCommand.angular.z = 0.2*deltaA;
	}else{
		m_roombaCommand.linear.x = 0.2;
	}
} // end of calculateCommands


//
void Just_move::mainLoop() {
	// determines the number of loops per second
	ros::Rate loop_rate(20);

	// als long as all is O.K : run
	// terminate if the node get a kill signal
	while (m_nodeHandle.ok()){

	  /* Model */
		// still not necessary to model anything for this job 

	  /* Plan  */
		// Whatever the task is, here is the place to plan the actions
		calculateCommand();
	
		//ROS_INFO(" robo_3_0 movement commands:forward speed=%f[m/sec] and turn speed =%f[rad/sec]", m_roombaCommand.linear.x, m_roombaCommand.angular.z);


	  /* Act   */
		// once everything is planned, let's make it happen

		// last chance to stop the robot
		emergencyStop();

		// send the command to the roomrider for execution
		m_commandPublisher.publish(m_roombaCommand);

		// spinOnce, just make the loop happen once
		ros::spinOnce();
		// and sleep, to be aligned with the 50ms loop rate
		loop_rate.sleep();

	}// end of if nodehandle O.K.

}// end of mainLoop


int main(int argc, char** argv) {

	// initialize
	ros::init(argc, argv, "Just_move");

	// get an object of type Just_move and call it robbi
	Just_move robbi;

	// make robbi run
	// main loop
	robbi.mainLoop();

	return 0;
}