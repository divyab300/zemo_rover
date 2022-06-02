/*
 * Differential drive robot implemented with Arduino and ROS
 * Low level PID speed controllers for motors.
 * Developed by: Divya Prakash Biswas on 30th May,2022
 */
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>
#include <PID_v1.h>
#include <motor.h>
#include <filter25_1kHz.h>

#define ENCA_l 2
#define ENCB_l 4
#define ENCA_r 3
#define ENCB_r 5
#define PPR 140

#define sample_time 10     // in ms, 100 Hz
#define publish_time 100     // in ms, 10 Hz
long int curr_T =0, last_T =0, last_pub_T = 0; // for dT calculation

//************* Left Motor *******************************

double left_ref_vel=0.0, left_state_vel=0.0, left_pwm=0.0;  //Define Variables we'll be connecting to PID block

filter25_1kHz left_m_filter; // Filter object for measured motor velocity filtering

double Kp_l=4, Ki_l=70, Kd_l=0.007;   // Controller Gains

PID leftPID(&left_state_vel, &left_pwm, &left_ref_vel, Kp_l, Ki_l, Kd_l, DIRECT); // PID controller object

motor left_motor(13,12,11,ENCA_l,ENCB_l);  // Motor object

volatile long int left_pos = 0;
double left_vel = 0;
long int curr_left_pos = 0,left_pos_prev = 0;

// ************ Right Motor ********************************

double right_ref_vel=0.0, right_state_vel=0.0, right_pwm=0.0;  //Define Variables we'll be connecting to PID block

filter25_1kHz right_m_filter; // Filter object for measured motor velocity filtering

double Kp_r=3.0, Ki_r=20.0, Kd_r=0.007;  // Controller Gains

PID rightPID(&right_state_vel, &right_pwm, &right_ref_vel, Kp_r, Ki_r, Kd_r, DIRECT); // PID controller object

motor right_motor(8,9,10,ENCA_r,ENCB_r);  // Motor object

volatile long int right_pos = 0;
double right_vel = 0;
long int curr_right_pos = 0, right_pos_prev = 0;

//*************** Driving Velocities of robot *******************

float des_v = 0.0; // desired linear velocity 
float des_w = 0.0; // desired angular velocity
float wheel_sep = 0.2; // in meters

// ************** ROS initializations ******************

ros::NodeHandle nh;

std_msgs::Int16 r_motor_pos;
std_msgs::Int16 l_motor_pos;

void cmd_Cb(const geometry_msgs::Twist& cmd_msg)
{
  des_v = cmd_msg.linear.x;
  des_w = cmd_msg.angular.z;
}

ros::Publisher rwheel("rwheel", &r_motor_pos);
ros::Publisher lwheel("lwheel", &l_motor_pos);
ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel",cmd_Cb);

void setup() {
  
  nh.initNode();
  nh.advertise(rwheel);
  nh.advertise(lwheel);
  nh.subscribe(cmd_sub);
  
  left_motor.init_pins();
  attachInterrupt(digitalPinToInterrupt(left_motor.get_encA()),readLeftEncoder,RISING);
  left_ref_vel = (double)-150.0; // -ve to rotate left wheel CCW
  leftPID.SetMode(AUTOMATIC);

  right_motor.init_pins();
  attachInterrupt(digitalPinToInterrupt(right_motor.get_encA()),readRightEncoder,RISING);
  right_ref_vel = (double)150.0;
  rightPID.SetMode(AUTOMATIC);
 
  delay(100); // Initial hold for millis should have some +ve value

}

void loop() {
  curr_T = millis();

  if(curr_T-last_T > sample_time) // Motor controller
  {
    double dT = (double)(curr_T - last_T)*1e-3;

    curr_left_pos = left_pos;
    left_vel = (double)(curr_left_pos - left_pos_prev)/dT;  // Measure the current velocity of left motor
    left_pos_prev = curr_left_pos;
    
    curr_right_pos = right_pos;
    right_vel = (double)(curr_right_pos - right_pos_prev)/dT;  // Measure the current velocity of right motor
    right_pos_prev = curr_right_pos;
    
  
    // Low Pass Filtering ; Cutoff freq = 25Hz
    left_m_filter.calc_output(left_vel);
    left_state_vel = left_m_filter.flt_vel;

    right_m_filter.calc_output(right_vel);
    right_state_vel = right_m_filter.flt_vel;
  
    leftPID.Compute();
    rightPID.Compute();
    
    left_motor.set_pwm(left_pwm);
    right_motor.set_pwm(right_pwm);


    last_T = curr_T;

  }

  if(curr_T-last_pub_T > publish_time) // ROS publishers
  {
    r_motor_pos.data = curr_right_pos;
    l_motor_pos.data = -curr_left_pos;

    rwheel.publish(&r_motor_pos);
    lwheel.publish(&l_motor_pos);
    nh.spinOnce();

    float dvel = 0.5*des_w*wheel_sep; 
    left_ref_vel = (double)-637*(des_v - dvel);  // Update the motor velocity references
    right_ref_vel = (double)637*(des_v + dvel);

    last_pub_T = curr_T;
  }

}
