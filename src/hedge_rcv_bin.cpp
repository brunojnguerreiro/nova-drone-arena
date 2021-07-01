#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <fcntl.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include "marvelmind_nav/hedge_pos.h"
#include "marvelmind_nav/hedge_pos_a.h"
#include "marvelmind_nav/hedge_pos_ang.h"
#include "marvelmind_nav/beacon_pos_a.h"
#include "marvelmind_nav/hedge_imu_raw.h"
#include "marvelmind_nav/hedge_imu_fusion.h"
#include "marvelmind_nav/beacon_distance.h"
#include "marvelmind_nav/hedge_telemetry.h"
#include "marvelmind_nav/hedge_quality.h"
#include "marvelmind_nav/marvelmind_waypoint.h"
extern "C" 
{
#include "marvelmind_nav/marvelmind_hedge.h"
}

#include <sstream>

#define ROS_NODE_NAME "hedge_rcv_bin"

struct MarvelmindHedge * hedge= NULL;

geometry_msgs::PoseStamped pose;
static sem_t *sem;
struct timespec ts;

////////////////////////////////////////////////////////////////////////

void semCallback()
{
	sem_post(sem);
}

static int hedgeReceivePrepare(int argc, char **argv)
{
	 // get port name from command line arguments (if specified)
    const char * ttyFileName;
    uint32_t baudRate;
    if (argc>=2) ttyFileName=argv[1];
      else ttyFileName=DEFAULT_TTY_FILENAME;
    if (argc>=3) baudRate= atoi(argv[2]);
      else baudRate=DEFAULT_TTY_BAUDRATE;
    
    // Init
    hedge=createMarvelmindHedge ();
    if (hedge==NULL)
    {
        ROS_INFO ("Error: Unable to create MarvelmindHedge");
        return -1;
    }
    hedge->ttyFileName=ttyFileName;
    hedge->baudRate= baudRate;
    hedge->verbose=true; // show errors and warnings
    hedge->anyInputPacketCallback= semCallback;
    startMarvelmindHedge (hedge);
}

static bool hedgeReceiveCheck(void)
{
        struct PositionValue position;
        getPositionFromMarvelmindHedge (hedge, &position);

        srand((unsigned) time(0));
        float randomNumber = ((double)rand()) / ((double)10*RAND_MAX);

        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "fcu"; //optional. Works fine without frame_id

        pose.pose.position.x= position.x/1000.0; 
        pose.pose.position.y= position.y/1000.0; 
        pose.pose.position.z= - (position.z/1000.0);
        pose.pose.orientation.x = 0 + randomNumber;
        pose.pose.orientation.y = 0 + randomNumber;
        pose.pose.orientation.z = 0 + randomNumber;
        pose.pose.orientation.w = 1 + randomNumber;

        return true;
}

/**
 * Node for Marvelmind hedgehog binary streaming data processing
 */
int main(int argc, char **argv)
{
  // initialize ROS node
  ros::init(argc, argv, ROS_NODE_NAME);
  ROS_INFO("Initializing ROS node...");

  sem = sem_open(DATA_INPUT_SEMAPHORE, O_CREAT, 0777, 0);
  
  // prepare hedgehog data receiver module
  hedgeReceivePrepare(argc, argv);

  ros::NodeHandle n;
  ros::Publisher pose_pub             = n.advertise<geometry_msgs::PoseStamped>
            ("/mavros/vision_pose/pose", 10);
  ros::Rate loop_rate(200);

  while (ros::ok()) 
  {
    if (hedge->terminationRequired)
      {
		  break;
      }	
       
    if (clock_gettime(CLOCK_REALTIME, &ts) == -1)
     {
        ROS_INFO("clock_gettime");
        return -1;
	 }
    ts.tv_sec += 2;
    sem_timedwait(sem,&ts);  
	  
    if (hedgeReceiveCheck())
     {// hedgehog data received
        if(!(pose.pose.position.x == 0 && pose.pose.position.x == 0 && pose.pose.position.z == 0))
        {
        pose_pub.publish(pose);
        }
        else{
          ROS_INFO("Position zero");
        }
     }   
     else
     {
       ROS_INFO("RIP");
     }
     
     
    ros::spinOnce();

    loop_rate.sleep();
  }

  // Exit
  if (hedge != NULL) 
    {
      stopMarvelmindHedge (hedge);
      destroyMarvelmindHedge (hedge);
    }
    
   sem_close(sem);

  return 0;
}