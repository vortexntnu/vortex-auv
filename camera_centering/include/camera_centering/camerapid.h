class PID{
private:
  double dt;
  double max;
  double min;
  double K_p;
  double K_d;s
  double integral
  double error;
  double pre_error;
public:
  PID(double dt, double max, double min, double K_p, double K_d, double K_i);
  ~PID();

  double calculate();
  updateError(double err);
}
