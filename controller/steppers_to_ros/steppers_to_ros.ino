#include <AccelStepper.h>
#include <ros.h>
#include <std_msgs/UInt16MultiArray.h>

#define enable_shield 8
#define MAX_SPEED 500
#define MAX_ACCELERATION 1000
#define motorInterfaceType 1

ros::NodeHandle nh;

void stepper_actuate_cb(const std_msgs::UInt16MultiArray &msg);
ros::Subscriber<std_msgs::UInt16MultiArray> sub_arm("niryo/stepper_actuate", &stepper_actuate_cb);

AccelStepper stepper_I(motorInterfaceType, 2, 5); // pin 3 = step, pin 6 = direction
AccelStepper stepper_II(motorInterfaceType, 3, 6);
AccelStepper stepper_III(motorInterfaceType, 4, 7);
int stepper_I_goal = 100;                           // Goal position
int stepper_II_goal = 1000;
int stepper_III_goal = 2000;


void setup()
{
    // Enable the CNC shield
    pinMode(enable_shield, OUTPUT);
    digitalWrite(enable_shield, LOW);
    
    nh.initNode();
    nh.subscribe(sub_arm);

    stepper_I.setMaxSpeed(MAX_SPEED);
    stepper_I.setAcceleration(MAX_ACCELERATION);
    stepper_I.setCurrentPosition(0);

    stepper_II.setMaxSpeed(MAX_SPEED);
    stepper_II.setAcceleration(MAX_ACCELERATION);
    stepper_II.setCurrentPosition(0);

    stepper_III.setMaxSpeed(MAX_SPEED);
    stepper_III.setAcceleration(MAX_ACCELERATION);
    stepper_III.setCurrentPosition(0);
}

void loop()
{
    stepper_I.moveTo(stepper_I_goal);
    stepper_II.moveTo(stepper_II_goal);
    stepper_III.moveTo(stepper_III_goal);

    stepper_I.run();
    stepper_II.run();
    stepper_III.run();

    // Keep ROS Node Up & Running
    nh.spinOnce();
}

void stepper_actuate_cb(const std_msgs::UInt16MultiArray &msg)
{
    stepper_I_goal = (int)msg.data[0];
    stepper_II_goal = (int)msg.data[1];
    stepper_III_goal = (int)msg.data[2];
}
