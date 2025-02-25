//
// LAB: Mobile Robotics
//
// unit_03 
// robo_3_1
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
    double zx = -4;
    double zy = -14.5;

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
	    m_roombaCommand.linear.x = 0.0;
 	    m_roombaCommand.angular.z = 0.0;
	    ROS_INFO(" Robot halted by emergency stop" ) ;
	  } // end of if danger
	
}// end of emergencyStop
 


// here we go
// this is the place where we will generate the commands for the robot
void A_star::calculateCommand() {

	// please find out what this part of the robot controller is doing
	// watch the robot 
	// look into the source file 
	// and try to deduce the underlying idea

    double angularz = 0.0;
    double linearx = 0.15;
    
    double dx= zx-m_amcl_pose.position.x;
    double dy= zy-m_amcl_pose.position.y;
    
    
    double alpha = atan2 (dy,dx);
    
    angularz= 0.2*(alpha-yaw);

    
    
    //check if angularz is too large
//     if (angularz*angularz>0.01){
//         linearx = 0.05;
//         if (angularz>0.0){
//             angularz=0.1;
//         }
//         else{
//             angularz=-0.1;
//         }
//     }
    
    //ROS_INFO(" alpha = %f, angularz= %f, dx=%f, dy=%f \n", alpha, angularz, dx,dy ) ;
    
    if (sqrt(dx*dx+dy*dy) < 0.5 ){
        linearx  = 0.0 ;
        angularz= 0.0;
        ROS_INFO("YOU HAVE ARRIVED AT YOUR DESTINATION!!!!!" ) ;
    }
    
    
    m_roombaCommand.linear.x  = linearx ;
    m_roombaCommand.angular.z= angularz;
    
    /*
    if (abs(angularz)>0.2){
        m_roombaCommand.linear.x  = 0.1*linearx ;
        if (angularz<0.0){
        
            m_roombaCommand.angular.z= -0.2;
        }
        else{
        
            m_roombaCommand.angular.z= 0.2;
        }
    }
    else{
        m_roombaCommand.linear.x  = linearx ;
        m_roombaCommand.angular.z= angularz;
        
    }
    /*
    
    
    
        // see if we have laser data available
	if( (&m_laserscan)->ranges.size() > 0)
          {
		double qs = 0.2 ;
                double qt = 0.0 ; 
                int c_r = 0 ;
                int c_l = 0 ;

                // first part
                for( int i=45;i<=270;i++)
                   if (m_laserscan.ranges[i] < 0.5)
                      c_r++; 

                // second part 
                for( int i=270;i<=490;i++)
                   if (m_laserscan.ranges[i] < 0.7)
                      c_l++ ;


                // third part 
                if (c_l > 5)
                   {
                        qs = 0.1, qt  = -0.25 ;
                   } // end of if

                // fourth part 
                if (c_r > 5)
                   {
                        qs = 0.1 ;
                        qt = +0.3 ;
                   } // end of if


                // set the roomba velocities
                // the linear velocity (front direction is x axis) is measured in m/sec
                // the angular velocity (around z axis, yaw) is measured in rad/sec
                m_roombaCommand.linear.x  = qs ;
                m_roombaCommand.angular.z = qt ;

          } // end of if we have laser data


    */
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
	
		ROS_INFO(" robo_3_1 commands:forward speed=%f[m/sec] and turn speed =%f[rad/sec] and amcl posx=%f, posy=%f, yaw=%f", 
			m_roombaCommand.linear.x, m_roombaCommand.angular.z, poseAMCLx, poseAMCLy, poseAMCLa);


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

	// make robbi run
	// main loop
	robbi.mainLoop();

	return 0;
}

// end of file: robo_3_1.cpp
 
