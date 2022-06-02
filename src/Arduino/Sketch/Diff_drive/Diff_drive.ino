#include <PID_v1.h>

class motor 
{
  public:
  motor(int inA, int inB, int pwm_pin, int encA, int encB)
  {
    INA = inA;
    INB = inB;
    PWM_pin = pwm_pin;
    ENCA = encA;
    ENCB = encB;
  }

  void init_pins()
  {
    pinMode(INA,OUTPUT);
    pinMode(INB,OUTPUT);
    pinMode(PWM_pin,OUTPUT);
    pinMode(ENCA,INPUT);
    pinMode(ENCB,INPUT);
  }

  int get_encA() {return ENCA;}
  int get_encB() {return ENCB;}

  void set_pwm(int pwm_val)
  {
    pwm_val = constrain(pwm_val,-255,255);
    if (pwm_val>=0)
    {
      digitalWrite(INA,HIGH);
      digitalWrite(INB,LOW);
      analogWrite(PWM_pin,pwm_val);
    }
    else
    {
      digitalWrite(INA,LOW);
      digitalWrite(INB,HIGH);
      analogWrite(PWM_pin,abs(pwm_val));
    }
  }

  private:
  int INA,INB,PWM_pin,ENCA,ENCB;
};

class filter
{
  public:
  double prev_m_vel = 0.0, flt_vel=0.0; // previous motor velocity, filtered value

  void calc_output(double M_vel)
  {
    flt_vel = 0.854*flt_vel + 0.072*M_vel + 0.072*prev_m_vel; 
    prev_m_vel = M_vel;
  }
};

////////////////////////////////////

#define ENCA_l 2
#define ENCB_l 4
#define ENCA_r 3
#define ENCB_r 5
#define PPR 140

#define sample_time 10     // in ms, 100 Hz
long int curr_T =0, last_T =0; // for dT calculation

///////////////// Left Motor /////////////////////

double left_ref_vel=0.0, left_state_vel=0.0, left_pwm=0.0;  //Define Variables we'll be connecting to PID block

filter left_m_filter; // Filter object for measured motor velocity filtering

double Kp_l=4, Ki_l=70, Kd_l=0.007;   // Controller Gains

PID leftPID(&left_state_vel, &left_pwm, &left_ref_vel, Kp_l, Ki_l, Kd_l, DIRECT); // PID controller object

motor left_motor(13,12,11,ENCA_l,ENCB_l);  // Motor object

volatile long int left_pos = 0;
double left_vel = 0;
long int left_pos_prev = 0;

void readLeftEncoder()
{
  int b = digitalRead(left_motor.get_encB());

  if(b>0)
  {
    left_pos++;
  }
  else
  {
    left_pos--;
  }
}

///////////////// Right Motor /////////////////////

double right_ref_vel=0.0, right_state_vel=0.0, right_pwm=0.0;  //Define Variables we'll be connecting to PID block

filter right_m_filter; // Filter object for measured motor velocity filtering

double Kp_r=3.0, Ki_r=20.0, Kd_r=0.007;  // Controller Gains

PID rightPID(&right_state_vel, &right_pwm, &right_ref_vel, Kp_r, Ki_r, Kd_r, DIRECT); // PID controller object

motor right_motor(8,9,10,ENCA_r,ENCB_r);  // Motor object

volatile long int right_pos = 0;
double right_vel = 0;
long int right_pos_prev = 0;

void readRightEncoder()
{
  int b = digitalRead(right_motor.get_encB());

  if(b>0)
  {
    right_pos++;
  }
  else
  {
    right_pos--;
  }
}

//////////////////////////////////////////////////////////

void setup() {
  
  Serial.begin(9600);
  
  left_motor.init_pins();
  left_motor.set_pwm(150);
  attachInterrupt(digitalPinToInterrupt(left_motor.get_encA()),readLeftEncoder,RISING);
  left_ref_vel = (double)-150.0;
  leftPID.SetMode(AUTOMATIC);

  right_motor.init_pins();
  left_motor.set_pwm(150);
  attachInterrupt(digitalPinToInterrupt(right_motor.get_encA()),readRightEncoder,RISING);
  right_ref_vel = (double)150.0;
  rightPID.SetMode(AUTOMATIC);

  
  delay(100);
}

void loop() {

  curr_T = millis();

  if(curr_T-last_T > sample_time)
  {
    double dT = (double)(curr_T - last_T)*1e-3;

    long int curr_left_pos = left_pos;
    left_vel = (double)(curr_left_pos - left_pos_prev)/dT;  // Measure the current velocity of left motor
    left_pos_prev = curr_left_pos;
    
    long int curr_right_pos = right_pos;
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

  if (Serial.available())
  {
    char a = Serial.read();
    if (a=='h') {left_ref_vel = -150; right_ref_vel = -150;}
    else if (a=='j') {left_ref_vel = -200; right_ref_vel = -200;}
    else if (a=='k') {left_ref_vel = -300; right_ref_vel = -300;}
    else if (a=='l') {left_ref_vel = -400; right_ref_vel = -400;}
  }
  
//  Serial.print(left_ref_vel);Serial.print(" ");Serial.print(left_state_vel);Serial.print(" ");
//  Serial.print(right_ref_vel);Serial.print(" ");Serial.println(right_state_vel);

  Serial.print(left_state_vel);Serial.print(" ");Serial.println(right_state_vel);
}
