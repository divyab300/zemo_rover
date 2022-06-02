#ifndef filter25_1kHz_h
#define filter25_1kHz_h

class filter25_1kHz
{
    public:

    double prev_m_vel = 0.0, flt_vel = 0.0; // previous motor velocity, filtered value
    void calc_output(double M_vel);
};

#endif