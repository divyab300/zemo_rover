/*
    Developed by: Divya Prakash Biswas on 31st May,2022
*/

#include "Arduino.h"
#include "filter25_1kHz.h"

void filter25_1kHz::calc_output(double M_vel)
  {
    flt_vel = 0.854*flt_vel + 0.072*M_vel + 0.072*prev_m_vel; 
    prev_m_vel = M_vel;
  }