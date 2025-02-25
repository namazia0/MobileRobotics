//
// LAB: Mobile Robotics
//
// unit_03 
// robo_3_0
//
// A simple robot control named: A-star search algorithm
//

// Includes to talk with ROS and all the other nodes
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <math.h>
#include <tf/tf.h>
#include <iostream>
#include <fstream>
double poseAMCLx, poseAMCLy, poseAMCLa;

// Class definition
class A_star {
public:

	A_star();
	void laserCallback(const sensor_msgs::LaserScanConstPtr& m_scan_data);
    void poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& m_msgAMCL);
	void emergencyStop();
	void calculateCommand();
	void readLaser();
	void mainLoop();
    double zx = 2;
    double zy = -11;
    int count = 0;

protected:

	// Nodehandle for A_star robot
	ros::NodeHandle m_nodeHandle;

    // Subscriber and membervariables for our amcl
	ros::Subscriber m_amclSubscriber;
	geometry_msgs::Pose m_amcl_pose;
    
	// Subscriber and membervariables for our laserscanner
	ros::Subscriber m_laserSubscriber;
	sensor_msgs::LaserScan m_laserscan;

	// Publisher and membervariables for the driving commands
	ros::Publisher m_commandPublisher;
	geometry_msgs::Twist m_roombaCommand;
    
    double roll,pitch,yaw;

};

// constructor
A_star::A_star() {

	// Node will be instantiated in root-namespace
	ros::NodeHandle m_nodeHandle("/");

	// Initializing the node handle
	m_laserSubscriber  = m_nodeHandle.subscribe<sensor_msgs::LaserScan>("laserscan", 20, &A_star::laserCallback, this);
    m_amclSubscriber  = m_nodeHandle.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 20, &A_star::poseAMCLCallback, this);
 //   m_amclSubscriber  = m_nodeHandle.subscribe("amcl_pose", 20, &A_star::poseAMCLCallback, this);
    m_commandPublisher = m_nodeHandle.advertise<geometry_msgs::Twist> ("cmd_vel", 20);

}// end of A_star constructor



//callback for getting amcl_pose

void A_star::poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amclData)
{
    m_amcl_pose = amclData->pose.pose;
    double x,y,z,w,i,j,k = 0.0;
    
//     poseAMCLx = msgAMCL->pose.pose.position.x;
//     poseAMCLy = msgAMCL->pose.pose.position.y;
//     poseAMCLa = msgAMCL->pose.pose.orientation.w;
    x=m_amcl_pose.position.x;
    y=m_amcl_pose.position.y;
    z=m_amcl_pose.position.z;
    i=m_amcl_pose.orientation.x;
    j=m_amcl_pose.orientation.y;
    k=m_amcl_pose.orientation.z;
    w=m_amcl_pose.orientation.w;
    
    tf::Quaternion quaternion(i,j,k,w);
    tf::Matrix3x3 m (quaternion);
    m.getRPY(roll,pitch,yaw);
}

// callback for getting laser values 
void A_star::laserCallback(const sensor_msgs::LaserScanConstPtr& scanData) {
    
	if( (&m_laserscan)->ranges.size() < scanData->ranges.size() ) {
		m_laserscan.ranges.resize((size_t)scanData->ranges.size()); 
    	}

	for(unsigned int i = 0; i < scanData->ranges.size(); i++)
	{
		m_laserscan.ranges[i] = scanData->ranges[i];
	}
}// end of laserCallback



// have a look into the laser measurement 
void A_star::readLaser() {
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
void A_star::emergencyStop() {

	int danger = 0 ;
        // see if we have laser data
 	if( (&m_laserscan)->ranges.size() > 0) 
	   for(unsigned int i=0; i < (&m_laserscan)->ranges.size(); i++)
	      if( m_laserscan.ranges[i] <= 0.40) 
		 danger = 1 ;		
		
	if(1==danger)
	  {
	    m_roombaCommand.linear.x = 0.0;count+=1;
 	    m_roombaCommand.angular.z = 0.0;
	    ROS_INFO(" Robot halted by emergency stop" ) ;
	  } // end of if danger
	
}// end of emergencyStop
 


// here we go
// this is the place where we will generate the commands for the robot
void A_star::calculateCommand() {
    double position_list [4][2]{
        {0.0,-13.0},
        {1.0,-11.0},
        {3.0,-9.0},
        {4.0,-8.0}
    };

    double angularz = 0.0;
    double linearx = 0.2;
    

        double dx= position_list[count][0]-m_amcl_pose.position.x;
        double dy= position_list[count][1]-m_amcl_pose.position.y;
    
    
        double alpha = atan2 (dy,dx);
    
        angularz= 0.1*(alpha-yaw);
        
        double diffalpha = alpha - yaw;
        
        ROS_INFO(" alpha = %f, angularz= %f, dx=%f, dy=%f, diffAlpha=%f, count=%i \n", alpha, angularz, dx,dy, diffalpha, count  ) ;

    
        if (sqrt(dx*dx+dy*dy) < 0.5 ){
            ROS_INFO("YOU HAVE ARRIVED AT YOUR DESTINATION!!!!!" ) ;
            count+=1;
        }
    
        if (count >= 4){
            linearx=0.0;
            angularz=0.0;
            ROS_INFO("YOU HAVE ARRIVED AT YOUR DESTINATION!!!!!" ) ;
        }
    
        m_roombaCommand.linear.x  = linearx ;
        m_roombaCommand.angular.z= angularz;
        

    
} //end of calculateCommands


//
void A_star::mainLoop() {
	// determines the number of loops per second
	ros::Rate loop_rate(50);

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
	
		ROS_INFO(" robo_3_0 commands:forward speed=%f[m/sec] and turn speed =%f[rad/sec] and amcl posx=%f, posy=%f, yaw=%f", 
			m_roombaCommand.linear.x, m_roombaCommand.angular.z, m_amcl_pose.position.x, m_amcl_pose.position.y, yaw);


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
	ros::init(argc, argv, "A_star");

	// get an object of type A_star and call it robbi
	A_star robbi;
    

    int kantenArray[98][3];
    double knotenArray[60][3];
    std::ifstream file("/home/user/ropra/PG_home/WS22_ws/src/unit_03/kanten_prakt_03.txt");
    if(file.is_open())
    {
        
        for(int i = 0; i < 98; i++) {
            for (int j=0; j < 3; j++)
        {
            file >> kantenArray[i][j];
        }
        }
            file.close();
        file.clear();
    }

    std::ifstream file2("/home/user/ropra/PG_home/WS22_ws/src/unit_03/knoten_prakt_03.txt");
    if(file2.is_open())
    {

        for(int i = 0; i < 60; i++) {
            for (int j=0; j < 3; j++)
        {
            file2 >> knotenArray[i][j];
            
        }
        }
            file2.close();
        file2.clear();
    }

    
    std::cout << knotenArray[57][1] <<"\t";
   std::cout << kantenArray[57][1] <<"\t";

        
        

	// make robbi run
	// main loop
	robbi.mainLoop();

	return 0;
}

// end of file: robo_3_0.cpp
 
