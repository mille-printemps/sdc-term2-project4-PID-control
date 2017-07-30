#include <iostream>
#include <numeric>
#include <assert.h>
#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
  for (int i=0; i<3; i++) {
    coefficients_.push_back(0.0);
    steps_.push_back(1.0);
  }
  
  coefficient_index_ = 0;
  
  status_ = PHASE_ONE;
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  // Kp_ = Kp;
  // Ki_ = Ki;
  // Kd_ = Kd;
  
  p_error_ = 0;
  i_error_ = 0;
  d_error_ = 0;
  
  previous_cte_ = 0;
}

void PID::UpdateError(double cte) {
  p_error_ = cte;
  i_error_ += cte;
  d_error_ = cte - previous_cte_;
  
  previous_cte_ = cte;
}

double PID::TotalError() {
  return -(coefficients_[0] * p_error_ + coefficients_[1] * i_error_ + coefficients_[2] * d_error_);
  // return -(Kp_ * p_error_ + Ki_ * i_error_ + Kd_ * d_error_);
}

double PID::Twiddle(double error, double best_error, double threshold) {
  double step = accumulate(steps_.begin(), steps_.end(), 0.0);
  
  cout << "Total Step: " << step << endl;
  
  if (step < threshold) {
    return -1;
  }

  double latest_best_error = 0;
  int index = coefficient_index_ % 3;
  
  switch(status_) {
    case PHASE_ONE:
    {
      coefficients_[index] += steps_[index];
      status_ = PHASE_TWO;
      latest_best_error = best_error;
      break;
    }
    case PHASE_TWO:
    {
      if (error < best_error) {
        steps_[index] *= 1.1;
        status_ = PHASE_ONE;
        coefficient_index_++;
        latest_best_error = error;
      } else {
        coefficients_[index] -= 2 * steps_[index];
        status_ = PHASE_THREE;
        latest_best_error = best_error;
      }
      break;
    }
    case PHASE_THREE:
    {
      if (error < best_error) {
        steps_[index] *= 1.1;
        latest_best_error = error;
      } else {
        coefficients_[index] += steps_[index];
        steps_[index] *= 0.9;
        latest_best_error = best_error;
      }
      status_ = PHASE_ONE;
      coefficient_index_++;
      break;
    }
    default:
      assert(false);
  }
  
  cout << "Kp: " << coefficients_[0] << " Ki: "  << coefficients_[1] << " Kd: " << coefficients_[2] << endl;
  cout << "Step1: " << steps_[0] << " Step2: " << steps_[1] << " Step3: " << steps_[2] << endl;
  
  return latest_best_error;
}
