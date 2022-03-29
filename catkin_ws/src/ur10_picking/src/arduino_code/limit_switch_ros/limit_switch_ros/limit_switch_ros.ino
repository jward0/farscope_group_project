  
/*
 * Limit Switch sketch for the limit switches on the robot arm.
 * This sketch is used to:
 * 1) Publish limit switch readings from the switches on the gripper arms to the 
 *    /limit_switch topic.
 *
 * Note that there is a dependency - the vacuum_switch.h header file needs to have been
 * generated within the UR10_picking package
 */

#include <ros.h>
#include <Wire.h>
#include <SPI.h>
//#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

// ---Set up arduino pins
int l_switch = 12;



unsigned long delayTime;

//define ros handler
ros::NodeHandle nh;

// Define the limit switch status publisher which publishes to a topic switch_status
// Pulishes "true" if closed and "false" if open
// FORMAT: Publisher class_name("topic_name", status_name)
std_msgs::Bool msg;
ros::Publisher switch_status("switch_status", &msg);


void setup(){
    //Initiate the switch pin
    pinMode(l_switch, INPUT);

    // Initiate the ROS node and advertise services and topic
    nh.initNode();
    nh.advertise(switch_status);

}

void check_switch(){
    if ( digitalRead(l_switch) == LOW ){
//      switch is active (closed)
        msg.data = true;
    }
    else if (  digitalRead(l_switch) == HIGH){
//      switch is inactive (open)
        msg.data = false;
    }
}


void loop(){

    // call function to check switch status
    check_switch();

    // reset switch to HIGH (open)
    digitalWrite(l_switch, HIGH);

    // publish msgs to topic
    switch_status.publish( &msg );
  
    nh.spinOnce();
    delay(1000);
}
