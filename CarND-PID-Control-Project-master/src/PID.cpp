#include "PID.h"
#include <iostream>
#include <stdlib.h>
#include <cmath>
#include <vector>
PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {

  this->Kp = Kp_;
  this->Ki = Ki_;
  this->Kd = Kd_;

  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;

  double best_err = 9999.9;

}

void PID::UpdateError(double cte) {

  d_error = cte - p_error; //now we can subtract from prev error which would be p_error allowing use to use prev_cte in such a way that it works
  p_error = cte;
  i_error += cte;
  
}

double PID::TotalError() {

  double steer = -Kp*p_error - Kd*d_error - Ki*i_error; 
  return steer;

}

void PID::Twiddle(double cte) {
  std::vector<double> p = {0.0005, 0, 2.5};
  std::vector<double> dp = {1.,1.,1.};
  best_err = TotalError();
  double tol = 0.2;

  while((dp[0]+dp[1]+dp[2]) > tol) {
    for (int i=0;i<p.size();i++) {
      p[i] += dp[i];
      this->Kp = p[0];
      this->Ki = p[1];
      this->Kd = p[2];
      double err = TotalError();
      if (abs(err)<best_err){
        best_err = err;
        dp[i]*=1.1;
      }
      else{
        p[i] -= 2.0*dp[i];
        this->Kp = p[0];
        this->Ki = p[1];
        this->Kd = p[2];
        err = TotalError();
        if (abs(err) < best_err) {
          best_err = abs(err);
          dp[i] *= 1.1;
        }
        else {
          p[i] += dp[i];
          dp[i] *= 0.9;
        }
      }
    }
  }
}