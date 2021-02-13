#include <ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <AccelStepper.h>

#define motorInterfaceType 1
#define MAX_SPEED 1500
#define MAX_ACCELERATION 1000


ros::NodeHandle nh;

void stepper_actuate_cb(const std_msgs::UInt16MultiArray &msg);

ros::Subscriber<std_msgs::UInt16MultiArray> sub_arm("niryo/stepper_actuate", &stepper_actuate_cb);
std_msgs::UInt16MultiArray states_msg;
ros::Publisher pub_states("niryo/stepper_states", &states_msg);

int stepper_goal = 0;                           // Goal position
AccelStepper stepper (motorInterfaceType,3,2);

void setup()
{ 
    nh.initNode();
    // Init and Create the response message that will be published later
    states_msg.layout.dim = (std_msgs::MultiArrayDimension *)
    malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
    states_msg.layout.dim[0].label = "height";
    states_msg.layout.dim[0].size = 4;
    states_msg.layout.dim[0].stride = 1;
    states_msg.layout.data_offset = 0;
    states_msg.data = (int *)malloc(sizeof(int) * 8);
    states_msg.data_length = 3;

    nh.subscribe(sub_arm);
    nh.advertise(pub_states);

    stepper.setMaxSpeed(MAX_SPEED);
    stepper.setAcceleration(MAX_ACCELERATION);
    stepper.setCurrentPosition(0);
}

void loop()
{
  stepper.moveTo(stepper_goal);
  stepper.run();

  states_msg.data[0] = stepper.currentPosition();
  //pub_states.publish(&states_msg);
    
  // Keep ROS Node Up & Running
  nh.spinOnce();
}

void stepper_actuate_cb(const std_msgs::UInt16MultiArray &msg)
{
    stepper_goal = (int)msg.data[0];
}
