#ifndef motor_h
#define motor_h

class motor
{
    public:
        motor(int inA, int inB, int pwm_pin, int encA, int encB);
        void init_pins();
        int get_encA();
        int get_encB();
        void set_pwm(int pwm_val);

    private:
    int INA,INB,PWM_pin,ENCA,ENCB;
};  

#endif