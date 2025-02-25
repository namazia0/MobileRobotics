//
// LAB: Mobile Robotics
//
// unit_02 
// robo_X
//
// A simple robot control named: Wall follower
//

// Includes to talk with ROS and all the other nodes
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

// Class definition
class Wall_follower {
public:
	Wall_follower();
	void laserCallback(const sensor_msgs::LaserScanConstPtr& m_scan_data);
	void emergencyStop();
	void calculateCommand();
	void readLaser();
	void mainLoop();

protected:

	// Nodehandle for Wall_follower robot
	ros::NodeHandle m_nodeHandle;

	// Subscriber and membervariables for our laserscanner
	ros::Subscriber m_laserSubscriber;
	sensor_msgs::LaserScan m_laserscan;

	// Publisher and membervariables for the driving commands
	ros::Publisher m_commandPublisher;
	geometry_msgs::Twist m_roombaCommand;

};

// constructor
Wall_follower::Wall_follower() {

	// Node will be instantiated in root-namespace
	ros::NodeHandle m_nodeHandle("/");

	// Initializing the node handle
	m_laserSubscriber  = m_nodeHandle.subscribe<sensor_msgs::LaserScan>("laserscan", 20, &Wall_follower::laserCallback, this);
	m_commandPublisher = m_nodeHandle.advertise<geometry_msgs::Twist> ("cmd_vel", 20);

}// end of Wall_follower constructor



// callback for getting laser values 
void Wall_follower::laserCallback(const sensor_msgs::LaserScanConstPtr& scanData) {
    
	if( (&m_laserscan)->ranges.size() < scanData->ranges.size() ) {
		m_laserscan.ranges.resize((size_t)scanData->ranges.size()); 
    	}

	for(unsigned int i = 0; i < scanData->ranges.size(); i++)
	{
		m_laserscan.ranges[i] = scanData->ranges[i];
	}
}// end of laserCallback



// have a look into the laser measurement 
void Wall_follower::readLaser() {
	double sum = 0.0 ;

        // see if we have laser data
 	if( (&m_laserscan)->ranges.size() > 0) 
	{
	   // go through all laser beams
   	   for(unsigned int i=0; i < (&m_laserscan)->ranges.size(); i++)
	     {
		sum = sum + m_laserscan.ranges[i] ;
 	     }// end of for all laser beams
	  
	} // end of if we have laser data
}// end of readLaser 


// robot shall stop, in case anything is closer than ... 
void Wall_follower::emergencyStop() {

	int danger = 0 ;
        // see if we have laser data
 	if( (&m_laserscan)->ranges.size() > 0) 
	   for(unsigned int i=0; i < (&m_laserscan)->ranges.size(); i++)
	      if( m_laserscan.ranges[i] <= 0.40) 
		 danger = 1 ;		
		
	if(1==danger)
	  {
	    m_roombaCommand.linear.x = 0.0;
 	    m_roombaCommand.angular.z = 0.0;
	    ROS_INFO(" Robot halted by emergency stop" ) ;
	  } // end of if danger
	
}// end of emergencyStop
 


// here we go
// this is the place where we will generate the commands for the robot
void Wall_follower::calculateCommand() {
	// set the roomba velocities
	// the linear velocity (front direction is x axis) is measured in m/sec
	// the angular velocity (around z axis, yaw) is measured in rad/sec
	m_roombaCommand.linear.x  = 0.2;
	m_roombaCommand.angular.z = 0.0;
    
    double v = 0.0;
    
    double ang = 0.0;
    
    double r = 0.0;
    
    double l = 0.0;
    
    double alpha = 1.0;
    
    double beta = 1.0;
    
    double gamma = 0.0;

    int danger = 0 ;
        // see if we have laser data
 	if( (&m_laserscan)->ranges.size() > 0)
        for( int i=260;i<=280;i++)
                   if (m_laserscan.ranges[i] <= 1.5)
                   {
                       //for (int j=-1; j
                       ROS_INFO(" test" ) ;
                       danger=1;
                   }
        
//	   for(unsigned int i=0; i < (&m_laserscan)->ranges.size(); i++)
//	      if( m_laserscan.ranges[i] <= 1.50) 
//		 danger = 1 ;		
		
	if(1==danger)
	  {
	    m_roombaCommand.linear.x = v;
 	    m_roombaCommand.angular.z = 0.0;
	  } // end of if danger

} // end of calculateCommands


void Wall_follower::mainLoop() {
	// determines the number of loops per second
	ros::Rate loop_rate(20);

	// als long as all is O.K : run
	// terminate if the node get a kill signal
	while (m_nodeHandle.ok())
	{
	   /*  Sense */
		// get all the sensor value that might be useful for controlling the robot
		readLaser(); 

	   /*  Model */
		// still not necessary to model anything for this job 

	   /* Plan  */
		// Whatever the task is, here is the place to plan the actions
		calculateCommand();
	
		ROS_INFO(" robo_X commands:forward speed=%f[m/sec] and turn speed =%f[rad/sec]", 
			m_roombaCommand.linear.x, m_roombaCommand.angular.z);


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
	ros::init(argc, argv, "Wall_follower");

	// get an object of type Wall_follower and call it robbi
	Wall_follower robbi;

	// make robbi run
	// main loop
	robbi.mainLoop();

	return 0;
}