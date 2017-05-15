#ifndef PID_H
#define PID_H
#define PID_STARTUP 300

#define T_UP 1
#define T_DOWN 2
#define T_INIT 0
#define MIN_RESOLUTION 0.1

#include <uWS/uWS.h>
#include <vector>
#include <array>

using namespace std;

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /* Local state */
  int n;
  double err;

  /* Twiddle */
  vector<double> p;
  vector<double> dp;
  double best_err;
  int t_index, t_state;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

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
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
   * Guess our control value.
   */
  double Guess();

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
   * Restart our PID controller
   */
  void TwiddleInit();

  /*
   * We finished a run, so update our error terms
   * and reset our counters for the next Twiddle
   * iteration.
   *
   */
  void TwiddleUpdate();

  /*
   * We finished a run and have obtained a total error err.
   * Take the next step in the twiddle algorithm, either
   * 1) going UP on the value of parameter t_index,
   * 2) going DOWN on the value of parameter t_index,
   * 3) staying UP/DOWN and incrementing our delta by 1.1x
   * 4) refining our delta to be finer grained by 0.9x
   *
   * If a given parameter t_index is changed by less than
   * MIN_RESOLUTION, stay on that parameter until we've found
   * something that works within MIN_RESOLUTION.
   *
   * We found this helpful as we first do our best to stabilize
   * the gross updates with proportional error.  Once these
   * are refined we step and repeat the same for the differential
   * error.  Once that is fine, we skip to the integral error and
   * repeat.  After all three parameters are at the same resolution,
   * we repeat each in turn in search of optimum values.
   *
   */
  void TwiddleStep(double err);

};

#endif /* PID_H */
