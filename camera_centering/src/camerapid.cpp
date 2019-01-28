#include <camera_centering/camerapid.h>


PID::PID(double dt, double max, double min, double K_p, double K_d, double K_i)
: dt(dt), max(max),min(min),K_p(K_p),K_d(K_d), K_i(K_i), error(0), pre_error(0)
  ,integral(0)
{
}

PID::~PID()
{
}

PID::calculate()
{
    //Propotional
    double Pout = K_p * error;

    //Integral
    integral += error * dt;
    double Iout = K_i * integral;

    //Derivative
    double derivative = (error - pre_error) / dt;
    double Dout = K_d * derivative;

    //Total
    double tot = Pout + Iout + Dout;
    if (tot > max){tot = max;}
    else if (tot< min){tot = min;}

    //Update pre_error
    pre_error = error;

return output;
}

PID::updateError(double err)
{
    error = err;
}
