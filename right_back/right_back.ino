#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int64.h>
#define A_1 8
#define B_1 7
#define PWM_1 6 
//#define A_2 9
//#define B_2 10
//#define PWM_2 11
ros::NodeHandle  nh;
int val;
//Encoder pins definition
// Left encoder
#define Left_Encoder_PinA 2
#define Left_Encoder_PinB 4

volatile long Left_Encoder_Ticks = 0;
volatile bool LeftEncoderBSet;
/*
//Right Encoder
#define Right_Encoder_PinA 3
#define Right_Encoder_PinB 5

volatile long Right_Encoder_Ticks = 0;
volatile bool RightEncoderBSet;
*/
void SetupEncoders()
{
  // Quadrature encoders
  // Left encoder
  pinMode(Left_Encoder_PinA, INPUT_PULLUP);      // sets pin A as input  
  pinMode(Left_Encoder_PinB, INPUT_PULLUP);      // sets pin B as input
  //Attaching interrupt in Left_Enc_PinA.
  attachInterrupt(digitalPinToInterrupt(2), do_Left_Encoder, RISING);
  
/*
  // Right encoder
  pinMode(Right_Encoder_PinA, INPUT_PULLUP);      // sets pin A as input
  pinMode(Right_Encoder_PinB, INPUT_PULLUP);      // sets pin B as input
  //Attaching interrupt in Right_Enc_PinA.
  attachInterrupt(digitalPinToInterrupt(3), do_Right_Encoder, RISING); 
*/
}
void l_f_vel_to_motor( const std_msgs::Float32& vel){
  if (vel.data > 0)
  {
digitalWrite(A_1,LOW);
digitalWrite(B_1,HIGH);
analogWrite(PWM_1,vel.data);
  }
  else if(vel.data < 0)
  {
 digitalWrite(A_1,HIGH);
 digitalWrite(B_1,LOW);
 analogWrite(PWM_1,abs(vel.data));

  }
  else if(vel.data == 0)
  {

   digitalWrite(A_1,HIGH);
   digitalWrite(B_1,HIGH);
  
   }  
}
/*
void l_b_vel_to_motor( const std_msgs::Float32& vel){
  if (vel.data > 0)
  {
digitalWrite(A_2,LOW);
digitalWrite(B_2,HIGH);
analogWrite(PWM_2,vel.data);
  }
  else if(vel.data < 0)
  {
 digitalWrite(A_2,HIGH);
 digitalWrite(B_2,LOW);
 analogWrite(PWM_2,abs(vel.data));

  }
  else if(vel.data == 0)
  {

   digitalWrite(A_2,HIGH);
   digitalWrite(B_2,HIGH);
  
   }  
}

*/
ros::Subscriber<std_msgs::Float32> l_f_cmd("r_f_wheel_cmd", &l_f_vel_to_motor );
//ros::Subscriber<std_msgs::Float32> l_b_cmd("l_b_wheel_cmd", &l_b_vel_to_motor );



std_msgs::Int64 l_f_ticks;
//std_msgs::Int64 l_b_ticks;
ros::Publisher l_f_wheel("r_f_wheel", &l_f_ticks);
//ros::Publisher l_b_wheel("l_b_wheel", &l_b_ticks);

void SetupMotors()
{
//Motor Left_front 
  pinMode(A_1, OUTPUT);
  pinMode(B_1, OUTPUT);
  pinMode(PWM_1, OUTPUT);
//Motor Left_back
  //pinMode(A_2, OUTPUT);
  //pinMode(B_2, OUTPUT);
  //pinMode(PWM_2, OUTPUT);
  
}
void setup()
{
  //Serial.begin(57600);
  pinMode(13,OUTPUT);
  //Setup Encoders
  SetupEncoders();
  SetupMotors();
 // nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(l_f_wheel);
 // nh.advertise(l_b_wheel);
  nh.subscribe(l_f_cmd);
  //nh.subscribe(l_b_cmd);
}

void loop()
{
  l_f_ticks.data = Left_Encoder_Ticks;
  //l_b_ticks.data = Right_Encoder_Ticks;
  l_f_wheel.publish( &l_f_ticks );
  //l_b_wheel.publish( &l_b_ticks );
  digitalWrite(13,HIGH);
  nh.spinOnce();
}

//do_Left_Encoder() Definitions
void do_Left_Encoder()
{
   // Test transition; since the interrupt will only fire on 'rising' we don't need to read pin A
  LeftEncoderBSet = digitalRead(Left_Encoder_PinB);   // read the input pin
  Left_Encoder_Ticks -= LeftEncoderBSet ? -1 : +1;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//do_Right_Encoder() Definitions
/*
void do_Right_Encoder()
{
  
  RightEncoderBSet = digitalRead(Right_Encoder_PinB);   // read the input pin
  Right_Encoder_Ticks += RightEncoderBSet ? -1 : +1;
}
*/
