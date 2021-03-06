#include <MultiStepper.h>
#include <AccelStepper.h>
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Byte.h>

#define motorInterfaceType    1
#define enable_shield         8
    

ros::NodeHandle nh;

void stepper_actuate_cb(const std_msgs::Int16MultiArray &msg);
void stepper_speed_cb(const std_msgs::UInt16 &msg);
void stepper_acceleration_cb(const std_msgs::UInt16 &msg);
void stepper_reset_position_cb(const std_msgs::Byte &msg);
void stepper_disable_output_cb(const std_msgs::Byte &msg);
void stepper_enable_output_cb(const std_msgs::Byte &msg);

ros::Subscriber<std_msgs::Int16MultiArray> sub_arm("niryo/stepper_actuate", &stepper_actuate_cb);
ros::Subscriber<std_msgs::UInt16> sub_speed("niryo/stepper_speed", &stepper_speed_cb);
ros::Subscriber<std_msgs::UInt16> sub_acceleration("niryo/stepper_acceleration", &stepper_acceleration_cb);
ros::Subscriber<std_msgs::Byte> sub_reset_position("niryo/stepper_reset_position", &stepper_reset_position_cb);
ros::Subscriber<std_msgs::Byte> sub_disable_output("niryo/stepper_disable_output", &stepper_disable_output_cb);
ros::Subscriber<std_msgs::Byte> sub_enable_output("niryo/stepper_enable_output", &stepper_enable_output_cb);

AccelStepper stepper_I(motorInterfaceType, 2, 5); // step, direction
AccelStepper stepper_II(motorInterfaceType, 3, 6); 
AccelStepper stepper_III(motorInterfaceType, 4, 7); 
MultiStepper steppers; 

long stepper_goals [] = {0,0,0};
int max_speed = 500;
int max_acceleration = 1000;

void setup()
{
    // Enable the CNC shield
    pinMode(enable_shield, OUTPUT);
    digitalWrite(enable_shield, LOW);
    
    nh.initNode();
    nh.subscribe(sub_arm);
    nh.subscribe(sub_speed);
    nh.subscribe(sub_acceleration);
    nh.subscribe(sub_reset_position);
    nh.subscribe(sub_disable_output);
    nh.subscribe(sub_enable_output);

    stepper_I.setMaxSpeed(max_speed);
    stepper_I.setAcceleration(max_acceleration);
    stepper_I.setCurrentPosition(0);

    stepper_II.setMaxSpeed(max_speed);
    stepper_II.setAcceleration(max_acceleration);
    stepper_II.setCurrentPosition(0);

    stepper_III.setMaxSpeed(max_speed);
    stepper_III.setAcceleration(max_acceleration);
    stepper_III.setCurrentPosition(0);

    steppers.addStepper(stepper_I);
    steppers.addStepper(stepper_II);
    steppers.addStepper(stepper_III);
}


void loop()
{
    steppers.moveTo(stepper_goals);
    steppers.run();

    // Keep ROS Node Up & Running
    nh.spinOnce();
}

void stepper_actuate_cb(const std_msgs::Int16MultiArray &msg)
{
    stepper_goals[0] = (long)msg.data[0];
    stepper_goals[1] = (long)msg.data[1];
    stepper_goals[2] = (long)msg.data[2];
}

void stepper_speed_cb(const std_msgs::UInt16 &msg)
{
    max_speed = (int)msg.data;
    stepper_I.setMaxSpeed(max_speed);
    stepper_II.setMaxSpeed(max_speed);
    stepper_III.setMaxSpeed(max_speed);
}

void stepper_acceleration_cb(const std_msgs::UInt16 &msg)
{
    max_acceleration = (int)msg.data;
    stepper_I.setAcceleration(max_acceleration);
    stepper_II.setAcceleration(max_acceleration);
    stepper_III.setAcceleration(max_acceleration);
}

void stepper_reset_position_cb(const std_msgs::Byte &msg)
{
  stepper_I.setCurrentPosition(0);
  stepper_II.setCurrentPosition(0);
  stepper_III.setCurrentPosition(0);
}

void stepper_disable_output_cb(const std_msgs::Byte &msg)
{
  stepper_I.disableOutputs();
  stepper_II.disableOutputs();
  stepper_III.disableOutputs();
}

void stepper_enable_output_cb(const std_msgs::Byte &msg)
{
  stepper_I.enableOutputs();
  stepper_II.enableOutputs();
  stepper_III.enableOutputs();
}
