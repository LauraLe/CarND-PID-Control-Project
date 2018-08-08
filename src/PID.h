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
  double prev_cte;
    double twiddle_error;
   double err[3];

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;
  double p[3];
 
 /*
  * Twiddle Parameters
  */
    const int start_measure = 300;         // starts measuring error for twiddle
    const int end_measure = 700;           // ending measuring error for twiddle
    const double twiddle_tol = 0.2;   // Twiddle weights tolerance
    const int max_iters = 5000;           // Maximum number of iterations
    const int num_params = 3;
    
    double dp[3];
    double best_p[3];
    double best_error;
    
    bool is_twiddle = false;
    bool is_retrial = false;
    
    int optimizer_t;
    int twiddle_pass;
    int twiddle_iter;
    // server
    uWS::WebSocket<uWS::SERVER> server;

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
  * Calculate the total PID error.
  */
  double TotalError();
    
  /*
   * Calculate the steering angle
   */
    double CalculateSteer();
    
    /*
     * Calculate Throttle given Steer Value & speed
     */
    double CalculateThrottle(double speed, double steer);
    
    /*
     * Initializes twiddle parameters.
     */
    void InitializeTwiddle();
    
    /*
     * Optimizes parameters (Kp, Ki, Kd).
     */
    void Twiddle();
    
    /*
     * Restarts simulator.
     */
    void RestartSim(uWS::WebSocket<uWS::SERVER> ws);
    
    /*
     * Stores server info.
     */
    void SetServer(uWS::WebSocket<uWS::SERVER> ws);
    
};

#endif /* PID_H */
