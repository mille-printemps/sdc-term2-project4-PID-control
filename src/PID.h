#ifndef PID_H
#define PID_H

#include <vector>

class PID {
public:
  /*
  * Errors
  */
  double p_error_;
  double i_error_;
  double d_error_;
  
  /*
  * Coefficients
  */ 
  double Kp_;
  double Ki_;
  double Kd_;

  /*
  * Constructor
  */
  PID(std::vector<double> initial_steps, double threshold);

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
  * Calculate the total PID error.
  */
  double TotalError();
  
  double Twiddle(double error, double best_error);
  
private:
  
  enum Status {
    PHASE_ONE,
    PHASE_TWO,
    PHASE_THREE
  };
  
  double previous_cte_;
  
  std::vector<double> coefficients_;
  std::vector<double> steps_;

  int number_of_coefficients_;
  int coefficient_index_;
  double threshold_;
  
  Status status_;
};

#endif /* PID_H */
