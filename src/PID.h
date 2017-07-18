#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  double total_error;
  double best_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;
  double throttle;
  double best_Kp;
  double best_Ki;
  double best_Kd;

  /*
  * Track number of frames
  */ 
  int n_init_frames;
  int n_error_frames;
  int n_frames;

  /*
  * Optimization parameters for coefficients
  */
  int number_loops;    // number of loops
  int twiddle_section; // tracking section of twiddle algorithm
  int param_optimized; // parameter being optimized
  double dp;
  double di;
  double dd;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double kp, double ki, double kd, double throttle_init, int init_frames, int error_frames);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * Calculate steering angle.
  */
  double SteeringAngle();

  /*
  * Update PID state with twiddle algorithm
  */
  void UpdateState();

  /*
  * Update optimization parameters
  */
  void UpdateCurrentOptimizationParameter(double multiplier);

  /*
  * Update PID parameters
  */
  void UpdateCurrentParameter(double multiplier);
};

#endif /* PID_H */
