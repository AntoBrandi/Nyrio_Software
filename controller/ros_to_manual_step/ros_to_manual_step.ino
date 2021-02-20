#include <ros.h>
#include <std_msgs/Int16MultiArray.h>

#define STEPPER_1_DIR 2
#define STEPPER_1_STEP 3

#define STEPPER_2_DIR 4
#define STEPPER_2_STEP 5

#define STEPPER_3_DIR 6
#define STEPPER_3_STEP 7

#define stepsPerRevolution 200

ros::NodeHandle nh;

void stepper_actuate_cb(const std_msgs::Int16MultiArray &msg);

ros::Subscriber<std_msgs::Int16MultiArray> sub_arm("niryo/stepper_actuate", &stepper_actuate_cb);

int actual_goal_stepper_1 = 0;
int actual_position_stepper_1 = 0;
int increment_stepper_1 = 1;


int actual_goal_stepper_2 = 0;
int actual_position_stepper_2 = 0;
int increment_stepper_2 = 1;


int actual_goal_stepper_3 = 0;
int actual_position_stepper_3 = 0;
int increment_stepper_3 = 1;

void setup()
{
    pinMode(STEPPER_1_DIR, OUTPUT);
    pinMode(STEPPER_1_STEP, OUTPUT);

    pinMode(STEPPER_2_DIR, OUTPUT);
    pinMode(STEPPER_2_STEP, OUTPUT);

    pinMode(STEPPER_3_DIR, OUTPUT);
    pinMode(STEPPER_3_STEP, OUTPUT);

    nh.initNode();
    nh.subscribe(sub_arm);
}

void loop()
{
    if(actual_goal_stepper_1<actual_position_stepper_1){
        // Set the spinning direction counterclockwise:
        digitalWrite(STEPPER_1_DIR, LOW);
        increment_stepper_1 = -1;
    } else{
        // Set the spinning direction clockwise:
        digitalWrite(STEPPER_1_DIR, HIGH);
        increment_stepper_1 = 1;
    }

    if(actual_goal_stepper_2<actual_position_stepper_2){
        // Set the spinning direction counterclockwise:
        digitalWrite(STEPPER_2_DIR, LOW);
        increment_stepper_2 = -1;
    } else{
        // Set the spinning direction clockwise:
        digitalWrite(STEPPER_2_DIR, HIGH);
        increment_stepper_2 = 1;
    }

    if(actual_goal_stepper_3<actual_position_stepper_3){
        // Set the spinning direction counterclockwise:
        digitalWrite(STEPPER_3_DIR, LOW);
        increment_stepper_3 = -1;
    } else{
        // Set the spinning direction clockwise:
        digitalWrite(STEPPER_3_DIR, HIGH);
        increment_stepper_3 = 1;
    }
    

    int dist_stepper_1 = abs(actual_goal_stepper_1-actual_position_stepper_1);
    int dist_stepper_2 = abs(actual_goal_stepper_2-actual_position_stepper_2);
    int dist_stepper_3 = abs(actual_goal_stepper_3-actual_position_stepper_3);

    int dist = max(dist_stepper_1, max(dist_stepper_2,dist_stepper_3));

    for (int i = 0; i < dist; i++)
    {
        // These four lines result in 1 step:
        if(actual_position_stepper_1!=actual_goal_stepper_1){
            digitalWrite(STEPPER_1_STEP, HIGH);
            actual_position_stepper_1 +=increment_stepper_1;
        }
        if(actual_position_stepper_2!=actual_goal_stepper_2){
            digitalWrite(STEPPER_2_STEP, HIGH);
            actual_position_stepper_2 +=increment_stepper_2;
        }
        if(actual_position_stepper_3!=actual_goal_stepper_3){
            digitalWrite(STEPPER_3_STEP, HIGH);
            actual_position_stepper_3 +=increment_stepper_3;
        }
        
        delayMicroseconds(500);


        digitalWrite(STEPPER_1_STEP, LOW);
        digitalWrite(STEPPER_2_STEP, LOW);
        digitalWrite(STEPPER_3_STEP, LOW);


        delayMicroseconds(500);   
    }

    // Keep ROS Node Up & Running
    nh.spinOnce();
}

void stepper_actuate_cb(const std_msgs::Int16MultiArray &msg)
{
    actual_goal_stepper_1 = (int)msg.data[0];
    actual_goal_stepper_2 = (int)msg.data[1];
    actual_goal_stepper_3 = (int)msg.data[2];
}
