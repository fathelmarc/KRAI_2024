class Controller {
private:
  float KP, KI, KD;
  float encPrev;
  float angular_vel_Prev;
  float error_integral;
  float error_previous;
  float pwm_max = 255;

  float total_gear_ratio = 19.2 * 3;
  float enc_ppr = 200;

  float angular_vel_Filt = 0;
  float angular_vel = 0;
public:

  Controller(float kp_, float ki_, float kd_)
    : KP(kp_), KI(ki_), KD(kd_) {
    encPrev = 0.0;
    angular_vel_Prev = 0.0;
    angular_vel_Filt = 0.0;
    error_integral = 0.0;
    error_previous = 0.0;
  }

  float control(float target, float enc, float deltaT) {
    /*convert nilai*/
    float radian = (enc - encPrev) / deltaT; //encoder count menjadid encoder count/second
    encPrev = enc;
    angular_vel = radian / (total_gear_ratio * enc_ppr); // convert encoder count/second jadi radian/second

    //masukkan ke low pass filter untuk memperbagus hasil
    angular_vel_Filt = 0.854 * angular_vel_Filt + 0.0728 * angular_vel + 0.0728 * angular_vel_Prev;
    angular_vel_Prev = angular_vel;

    /*hitung pid*/
    //ubah angular_vel_Filt menjadi angular_vel jika ingin menghitung tanpa filter
    float error = target - angular_vel_Filt; 

    error_integral += error * deltaT;
    
    float error_derivative = (error - error_previous) / deltaT;
    error_previous = error;
    
    float u = KP * error + KI * error_integral + KD * error_derivative;
    return u;
  }

  float getFilteredValue() const {
    return angular_vel_Filt;
  }
  float getPureValue() const{
    return angular_vel;
  }
  
  void setMotor(int cwPin, int ccwPin, float pwmVal) {
    float lim_pwm = fmax(-pwm_max, fmin(pwmVal, pwm_max));
    if (lim_pwm > 0) {
      analogWrite(cwPin, fabs(lim_pwm));
      analogWrite(ccwPin, 0);
    } else if (lim_pwm < 0) {
      analogWrite(cwPin, 0);
      analogWrite(ccwPin, fabs(lim_pwm));
    } else {
      analogWrite(cwPin, 0);
      analogWrite(ccwPin, 0);
    }
  }
};