/* 
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/Int64.h>
#include <AccelStepper.h>

// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver:
#define motorPin1  8      // IN1 on the ULN2003 driver
#define motorPin2  9      // IN2 on the ULN2003 driver
#define motorPin3  10     // IN3 on the ULN2003 driver
#define motorPin4  11     // IN4 on the ULN2003 driver
#define motorInterfaceType 8

// Create a new instance of the AccelStepper class:
AccelStepper stepper = AccelStepper(motorInterfaceType, motorPin1, motorPin3, motorPin2, motorPin4);

ros::NodeHandle  nh;

void messageCb( const std_msgs::Int64& toggle_msg){
  // Set target position:
  stepper.move(toggle_msg.data);
  // Run to position with set speed and acceleration:
  stepper.runToPosition();
}

ros::Subscriber<std_msgs::Int64> sub("start_engine", &messageCb );

void setup()
{ 
  // Set the maximum speed in steps per second:
  // Set the maximum steps per second:
  stepper.setMaxSpeed(1500);
  // Set the maximum acceleration in steps per second^2:
  stepper.setAcceleration(1400);
  // init the ros node
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{  
  nh.spinOnce();
  delay(1);
}
