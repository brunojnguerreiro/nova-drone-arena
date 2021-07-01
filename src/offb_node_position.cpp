/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Thrust.h>

/*std::vector<geometry_msgs::PoseStamped::ConstPtr> pose;*/
geometry_msgs::PoseStamped pose;
mavros_msgs::Thrust thrust;
mavros_msgs::State current_state;
geometry_msgs::Point hedge_pos;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void tf_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    
    pose.pose.position.x = msg->pose.position.x;
    pose.pose.position.y = msg->pose.position.y;
    pose.pose.position.z = msg->pose.position.z;
    pose.pose.orientation.x = msg->pose.orientation.x;
    pose.pose.orientation.y = msg->pose.orientation.y;
    pose.pose.orientation.z = msg->pose.orientation.z;
    pose.pose.orientation.w = msg->pose.orientation.w;
}

void random_thrust(const std_msgs::Float32::ConstPtr& msg) {
    
    thrust.thrust = msg->data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub           = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::ServiceClient arming_client    = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client  = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
            
    ros::Publisher local_pos_pub        = nh.advertise<geometry_msgs::PoseStamped>
             ("mavros/setpoint_position/local", 10);
    
    ros::Subscriber pos_sub          = nh.subscribe("/sim_position", 1000, tf_callback);
    ros::Publisher attitude_pub         = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_attitude/attitude", 10);
            
    ROS_INFO("Starting...");   

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    
    // send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(1.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(1.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
