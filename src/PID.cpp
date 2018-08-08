#include "PID.h"
#include <math>
#include <iostream>
#include <uWS/uWS.h>
#include <numeric>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    p[0] = Kp;
    p[1] = Ki;
    p[2] = Kd;
    
    //Initialize errors
    prev_cte = 0;
    twiddle_error = 0;
    err[0] = 0;
    err[1] = 0;
    err[2] = 0;
    
    //optimizer timestemp
    optimizer_t = 0;
    
}

void PID::UpdateError(double cte) {
    if (prev_cte == 0) prev_cte = cte;
    err[0] = cte ; // p_error;
    err[1] += cte ; // i_error
    err[2] = cte - prev_cte; // d_error
    
    prev_cte = cte;
}

double PID::TotalError() {
    double total_error = pow(err[0],2);
    return total_error;
}

double PID::CalculateSteer(){
    double steer;
    double max_steer = 1.0;
    double min_steer = -1.0;
    
    steer = -p[0] * err[0] - p[1]*err[1] - p[2] *err[2];
    
    if(steer > max_steer) steer = max_steer;
    if(steer < min_steer) steer = min_steer;
    
    return steer;
}

double PID::CalculateThrottle(double speed, <#double steer#>){
    double throttle;
    
    if (is_twiddle and (optimizer_t < 10)){ //if just start the optimizer, remove throttle to avoid noise to steering
        throttle = 0.0;
    } else{
        throttle = 0.3;
    }
    
    return throttle;
}

void PID::Twiddle(){
    if (is_twiddle){
        if(optimizer_t == start_measure){
            twiddle_error = TotalError(); //start calculate twiddle error
        }
        if (optimizer_t > start_measure){
            twiddle_error += TotalError(); // while between start_measure and end_measure, accumulate error
        }
        
        if (optimizer_t > end_measure){
             cout << " optimizer measure error end with total error " << twiddle_error <<endl;
            
            double sum_dp = dp[0] + dp[1] + dp[2];{
                
            }
            if ((fabs(sum_dp) > twiddle_tol) & (twiddle_iter < max_iters)){
                
            }
            
        }
    }
}
