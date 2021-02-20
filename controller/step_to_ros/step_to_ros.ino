#include <ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <AccelStepper.h>

#define motorInterfaceType 1
#define MAX_SPEED 1500
#define MAX_ACCELERATION 1000


ros::NodeHandle nh;

void stepper_actuate_cb(const std_msgs::UInt16MultiArray &msg);

ros::Subscriber<std_msgs::UInt16MultiArray> sub_arm("niryo/stepper_actuate", &stepper_actuate_cb);

int stepper_goal = 0;                           // Goal position
AccelStepper stepper (motorInterfaceType,3,2);

void setup()
{ 
    nh.initNode();
    // Init and Create the response message that will be published later

    nh.subscribe(sub_arm);

    stepper.setMaxSpeed(MAX_SPEED);
    stepper.setAcceleration(MAX_ACCELERATION);
    stepper.setCurrentPosition(0);
}

void loop()
{
  stepper.moveTo(stepper_goal);
  stepper.run();
  
  // Keep ROS Node Up & Running
  nh.spinOnce();
}

void stepper_actuate_cb(const std_msgs::UInt16MultiArray &msg)
{
    stepper_goal = (int)msg.data[0];
}
