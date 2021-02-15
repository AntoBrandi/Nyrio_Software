#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Byte.h>
#include <DynamixelSDK.h>


// Control table address (XL320)
#define ADDR_PRO_TORQUE_ENABLE          24         
#define ADDR_PRO_GOAL_POSITION          30
#define ADDR_PRO_PRESENT_POSITION       37

// Protocol version
#define PROTOCOL_VERSION                2.0                 

// Dynamixel addresses
#define DXL_I                          1
#define DXL_II                         2
#define DXL_III                        3
                   
#define BAUDRATE                        1000000
#define DEVICENAME                      "1"                
                                                            
#define TORQUE_ENABLE                   1                  
#define TORQUE_DISABLE                  0                 


// Initialize PortHandler instance
dynamixel::PortHandler *portHandler;

// Initialize PacketHandler instance
dynamixel::PacketHandler *packetHandler;

int dxl_I_comm_result = COMM_TX_FAIL;             // Communication result
int dxl_II_comm_result = COMM_TX_FAIL;             // Communication result
int dxl_III_comm_result = COMM_TX_FAIL;             // Communication result

int dxl_I_goal_position = 0;         // Goal position
int dxl_II_goal_position = 0;
int dxl_III_goal_position = 0;

uint8_t dxl_I_error = 0;                          // Dynamixel error
uint8_t dxl_II_error = 0;                          // Dynamixel error
uint8_t dxl_III_error = 0;                          // Dynamixel error

int16_t dxl_I_present_position = 0;               // Present position
int16_t dxl_II_present_position = 0;
int16_t dxl_III_present_position = 0;

ros::NodeHandle nh;

void servo_actuate_cb( const std_msgs::Int16MultiArray& msg);
void servo_disable_output_cb( const std_msgs::Byte& msg);
void servo_enable_output_cb( const std_msgs::Byte& msg);

ros::Subscriber<std_msgs::Int16MultiArray> sub_arm("niryo/servo_actuate", &servo_actuate_cb );
ros::Subscriber<std_msgs::Byte> sub_disable_output("niryo/servo_disable_output", &servo_disable_output_cb);
ros::Subscriber<std_msgs::Byte> sub_enable_output("niryo/servo_enable_output", &servo_enable_output_cb);

std_msgs::Int16MultiArray states_msg;
ros::Publisher pub_states("niryo/servo_states", &states_msg);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while(!Serial);

  // Initialize PortHandler instance
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open port
  if (portHandler->openPort())
  {
    Serial.print("Succeeded to open the port!\n");
  }
  else
  {
    Serial.print("Failed to open the port!\n");
    return;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    Serial.print("Succeeded to change the baudrate!\n");
  }
  else
  {
    Serial.print("Failed to change the baudrate!\n");
    return;
  }

  // Enable Dynamixel Torque
  dxl_I_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_I, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_I_error);
  dxl_II_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_II, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_II_error);
  dxl_III_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_III, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_III_error);

  if((dxl_I_comm_result != COMM_SUCCESS) || (dxl_II_comm_result != COMM_SUCCESS)){ //||  (dxl_III_comm_result != COMM_SUCCESS)){
    if (dxl_I_comm_result != COMM_SUCCESS)
    {
      packetHandler->getTxRxResult(dxl_I_comm_result);
    }
    if (dxl_II_comm_result != COMM_SUCCESS)
    {
      packetHandler->getTxRxResult(dxl_II_comm_result);
    }
//    if (dxl_III_comm_result != COMM_SUCCESS)
//    {
//      packetHandler->getTxRxResult(dxl_III_comm_result);
//    }
  }
  else
  {
    Serial.print("Dynamixel has been successfully connected \n");
  }
  
  nh.initNode();

  // Init and Create the response message that will be published later
  states_msg.layout.dim = (std_msgs::MultiArrayDimension *)
  malloc(sizeof(std_msgs::MultiArrayDimension)*2);
  states_msg.layout.dim[0].label = "height";
  states_msg.layout.dim[0].size = 4;
  states_msg.layout.dim[0].stride = 1;
  states_msg.layout.data_offset = 0;
  states_msg.data = (int16_t *)malloc(sizeof(int)*8);
  states_msg.data_length = 3;
  
  // Inform ROS that this node will subscribe to messages on a given topic
  nh.subscribe(sub_arm);
  nh.subscribe(sub_disable_output);
  nh.subscribe(sub_enable_output);
  nh.advertise(pub_states);
}

void loop() {
  // write the current goal
  dxl_I_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_I, ADDR_PRO_GOAL_POSITION, dxl_I_goal_position, &dxl_I_error);
  if (dxl_I_comm_result != COMM_SUCCESS)
  {
    packetHandler->getTxRxResult(dxl_I_comm_result);
  }
  else if (dxl_I_error != 0)
  {
    packetHandler->getRxPacketError(dxl_I_error);
  }


  dxl_II_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_II, ADDR_PRO_GOAL_POSITION, dxl_II_goal_position, &dxl_II_error);
  if (dxl_II_comm_result != COMM_SUCCESS)
  {
    packetHandler->getTxRxResult(dxl_II_comm_result);
  }
  else if (dxl_II_error != 0)
  {
    packetHandler->getRxPacketError(dxl_II_error);
  }

  
//  dxl_III_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_III, ADDR_PRO_GOAL_POSITION, dxl_III_goal_position, &dxl_III_error);
//  if (dxl_III_comm_result != COMM_SUCCESS)
//  {
//    packetHandler->getTxRxResult(dxl_III_comm_result);
//  }
//  else if (dxl_III_error != 0)
//  {
//    packetHandler->getRxPacketError(dxl_III_error);
//  }

  
  // read the current status 
  dxl_I_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_I, ADDR_PRO_PRESENT_POSITION, (uint16_t*)&dxl_I_present_position, &dxl_I_error);
  if (dxl_I_comm_result != COMM_SUCCESS)
  {
    packetHandler->getTxRxResult(dxl_I_comm_result);
  }
  else if (dxl_I_error != 0)
  {
    packetHandler->getRxPacketError(dxl_I_error);
  }


  dxl_II_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_II, ADDR_PRO_PRESENT_POSITION, (uint16_t*)&dxl_II_present_position, &dxl_II_error);
  if (dxl_II_comm_result != COMM_SUCCESS)
  {
    packetHandler->getTxRxResult(dxl_II_comm_result);
  }
  else if (dxl_II_error != 0)
  {
    packetHandler->getRxPacketError(dxl_II_error);
  }


//  dxl_III_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_III, ADDR_PRO_PRESENT_POSITION, (uint16_t*)&dxl_III_present_position, &dxl_III_error);
//  if (dxl_III_comm_result != COMM_SUCCESS)
//  {
//    packetHandler->getTxRxResult(dxl_III_comm_result);
//  }
//  else if (dxl_III_error != 0)
//  {
//    packetHandler->getRxPacketError(dxl_III_error);
//  }


  states_msg.data[0] = dxl_I_present_position;
  states_msg.data[1] = dxl_II_present_position;
//  states_msg.data[2] = dxl_III_present_position;
    
  pub_states.publish( &states_msg );


  nh.spinOnce();
}

void servo_actuate_cb( const std_msgs::Int16MultiArray& msg){
  dxl_I_goal_position = (int)msg.data[0];
  dxl_II_goal_position = (int)msg.data[1];
  dxl_III_goal_position = (int)msg.data[2];
}

void servo_enable_output_cb( const std_msgs::Byte& msg)
{
  dxl_I_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_I, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_I_error);
  dxl_II_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_II, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_II_error);
  dxl_III_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_III, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_III_error);
}

void servo_disable_output_cb( const std_msgs::Byte& msg)
{
  dxl_I_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_I, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_I_error);
  dxl_II_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_II, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_II_error);
  dxl_III_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_III, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_III_error);
}
