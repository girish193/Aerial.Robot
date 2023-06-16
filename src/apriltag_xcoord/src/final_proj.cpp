

#include <ros/ros.h>
#include <ros/console.h>
#include <stdio.h>
#include <stdlib.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Time.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/CommandTOL.h> 

geometry_msgs::PoseStamped tag_pose;
geometry_msgs::PoseStamped tag_pose_inertial;
geometry_msgs::PoseStamped start_pose;
geometry_msgs::PoseStamped drone_pose;
mavros_msgs::PositionTarget pose_vel;

ros::ServiceClient land_client; 

bool tag_detected = false;

static float zpos = 0.75; 

int start_seq = 0;
int c = 0;

mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
 
    current_state = *msg;
}

void tag_inertial_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){

tag_pose_inertial = *msg; 

}


void tag_found_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){

    ROS_INFO("Tag Found"); 
     system("echo 0 > /sys/class/leds/red/brightness");

     
            if(zpos <= 0.3 && current_state.mode == "OFFBOARD")
            { 
                ROS_INFO("Checking for Landing"); 
                mavros_msgs::CommandTOL land_cmd; // Set all the descend parameters to Zero
                land_cmd.request.yaw = 0;
                land_cmd.request.latitude = 0;
                land_cmd.request.longitude = 0;
                land_cmd.request.altitude = 0;

                // When it lands, everything goes to zero
                if (!(land_client.call(land_cmd) && land_cmd.response.success))
                {   
                    // Publish landing
                    ROS_INFO("Landing");                  
                    
                    ros::shutdown(); // Shutdown node
                }
            
            }
            else if(zpos > 0.3 && current_state.mode == "OFFBOARD")
            {
                ROS_INFO("Inside Callback Drone Z position %f Zpos %f", drone_pose.pose.position.z, zpos); 
                 //drone_pose.pose.position.z -= 0.05; 
                zpos -= 0.001; 
                //pose_vel.velocity.z = -0.02;
            
            }
}
void tag_detection_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    tag_pose = *msg;

    if (c == 0) {
        start_seq = tag_pose.header.seq;
        start_pose = tag_pose;
        c++;
    }

    // When the tag pose stays constant for 5 iterations, the tag is not detected
    if ((tag_pose.header.seq - start_seq) > 5) {
        if (start_pose.pose.position.x == tag_pose.pose.position.x && start_pose.pose.position.y == tag_pose.pose.position.y) {
            tag_detected = false;
        } else {
            tag_detected = true;
        }
        start_seq = tag_pose.header.seq;
        start_pose = tag_pose;
    }
     
  
            
        
}

void altitude_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    drone_pose = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "enae788m_hw4");
    ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);

    land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");

    // ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::Publisher body_vel_pub_mavros = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 5);
    
    ros::Subscriber tag_pose_body = nh.subscribe<geometry_msgs::PoseStamped>("/tag_detections/tagpose_body", 10, tag_detection_cb);
    ros::Subscriber altitude_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, altitude_cb);
    ros::Subscriber tag_pose_detect = nh.subscribe<geometry_msgs::PoseStamped>("/tag_detections/tagpose", 10, tag_found_cb);
    ros::Subscriber target_lpp_pub = nh.subscribe<geometry_msgs::PoseStamped>("/tag_detections/tagpose_inertial", 10, tag_inertial_cb);
    
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    /* while(ros::ok() && !current_state.connected){
       ros::spinOnce();
       rate.sleep();
     }*/

    
    pose_vel.coordinate_frame = pose_vel.FRAME_BODY_NED;//pose_vel.FRAME_LOCAL_NED;
    pose_vel.coordinate_frame = 1;
    pose_vel.type_mask =  pose_vel.IGNORE_AFX | pose_vel.IGNORE_AFY | pose_vel.IGNORE_AFZ | pose_vel.FORCE | pose_vel.IGNORE_YAW ; //| pose_vel.IGNORE_PX | pose_vel.IGNORE_PY | pose_vel.IGNORE_PZ;


    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    ros::Time init_time;


   
     pose_vel.position.x = 0;
     pose_vel.position.y = 0;
     pose_vel.position.z = 0.75;
     //pose_vel.yaw = 0;
     for(int i = 100; ros::ok() && i > 0; --i){
        body_vel_pub_mavros.publish(pose_vel);
        ros::spinOnce();
        rate.sleep();
    }
   
    while(ros::ok() && !current_state.armed){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                    
                }
                last_request = ros::Time::now();
                break; 
            }
        }

       body_vel_pub_mavros.publish(pose_vel);

        ros::spinOnce();
        rate.sleep();
    }
    while(ros::ok()){

                    
        ROS_INFO_STREAM("\nVx: " << pose_vel.velocity.x << "Tx: " << tag_pose.pose.position.x);
        ROS_INFO_STREAM("\nVy: " << pose_vel.velocity.y << "Ty: " << tag_pose.pose.position.y);
        ROS_INFO_STREAM("\nVz: " << pose_vel.velocity.z << "Tz: " << tag_pose.pose.position.z);

    if (!tag_detected) {
            ROS_WARN_STREAM("Oops! Tag Not Detected!");
            system("echo 127 > /sys/class/leds/red/brightness");
           
            // go to the first waypoint
        pose_vel.position.x = 0;
        pose_vel.position.y = 0;
        pose_vel.position.z = 0.75;

        ROS_INFO("going to the first way point");
        for(int i = 0; ros::ok() && i < 10*20; ++i){
            body_vel_pub_mavros.publish(pose_vel);
            ros::spinOnce();
            rate.sleep();
        }
        ROS_INFO("first way point finished!");

  
        // go to the second waypoint
        pose_vel.position.x = 0;
        pose_vel.position.y = 1;
        pose_vel.position.z = 0.75;

         //send setpoints for 10 seconds
        ROS_INFO("going to second way point");
        for(int i = 0; ros::ok() && i < 10*20; ++i){

            body_vel_pub_mavros.publish(pose_vel);
            ros::spinOnce();
            rate.sleep();
        }
        ROS_INFO("second way point finished!");
     
    
        
        // go to the third waypoint
        pose_vel.position.x = 1;
        pose_vel.position.y = 1;
        pose_vel.position.z = 0.75;
    
        //send setpoints for 10 seconds
        ROS_INFO("going to third way point");
      
        for(int i = 0; ros::ok() && i < 10*20; ++i){

            body_vel_pub_mavros.publish(pose_vel);
            ros::spinOnce();
            rate.sleep();
            }
        ROS_INFO("third way point finished!");
    
      
        // go to the forth waypoint
        pose_vel.position.x = 1;
        pose_vel.position.y = 0;
        pose_vel.position.z = 0.75;
    
        //send setpoints for 10 seconds
        ROS_INFO("going to forth way point");
        for(int i = 0; ros::ok() && i < 10*20; ++i){

        body_vel_pub_mavros.publish(pose_vel);
        ros::spinOnce();
        rate.sleep();
        }
        ROS_INFO("forth way point finished!");
    
        pose_vel.position.x = 0;
        pose_vel.position.y = 0;
        pose_vel.position.z = 0.75;
        
        ROS_INFO("going back to the first point!");
    
        //send setpoints for 10 seconds
        for(int i = 0; ros::ok() && i < 10*20; ++i){

        body_vel_pub_mavros.publish(pose_vel);
        ros::spinOnce();
        rate.sleep();
        }

    }

/*        else {

            
        } */
        
        ROS_INFO("Drone Z position %f", drone_pose.pose.position.z); 
        pose_vel.header.stamp = ros::Time::now(); // Time header stamp
        pose_vel.header.frame_id = "base_link"; // "base_link" frame to compute odom
        // pose_vel.type_mask = 1987; // Mask for Vx, Vy, Z pos and Yaw rate
        pose_vel.position.z = zpos; //drone_pose.pose.position.z;//0.75
        pose_vel.position.x = tag_pose_inertial.pose.position.x;
        pose_vel.position.y = tag_pose_inertial.pose.position.y;

        // ROS_INFO("Checking %f", pose_vel.position.z);

        body_vel_pub_mavros.publish(pose_vel);

        
        ros::spinOnce();
        rate.sleep();
   }
    
    return 0;
}
