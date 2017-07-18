#include "PID.h"
#include <algorithm>  // for max function
#include <iostream>
#include <limits>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double kp, double ki, double kd, double throttle_init, int init_frames, int error_frames) {

    // Initialize throttle
    throttle = throttle_init;
    
    // Initialize errors
    p_error = 0.;
    i_error = 0.;
    d_error = 0.;
    total_error = 0.;
    best_error = 0;

    // Initialize coefficients
    Kp = kp;
    Ki = ki;
    Kd = kd;
    best_Kp = Kp;
    best_Ki = Ki;
    best_Kd = Kd;

    // Initialize number of frames    
    n_init_frames = init_frames;
    n_error_frames = error_frames;
    n_frames = 0;

    // Parameter being optimized [0-2] for P, I and D
    param_optimized = 0;

    // Variations of parameters
    dp = 0.1 * Kp;
    di = 0.1 * Ki;
    dd = 0.1 * Kd;

    // twiddle section
    twiddle_section = -2;  // we run a few loops to ensure car goes fast enough before optimization
    number_loops = 1;

}

void PID::UpdateError(double cte) {

    d_error = cte - p_error;  // p_error is the previous cte
    p_error = cte;
    //i_error = 0.95 * i_error + 0.05 * cte;  // similar to an integral, exp average, limit higher bound
    i_error += cte;
}

double PID::TotalError() {

    if (n_frames > n_init_frames)
    {
        total_error += p_error * p_error;
    }
    return total_error;
}

double PID::SteeringAngle() {

    double steering = - Kp * p_error  - Ki * i_error - Kd * d_error;

    // Limit steering to [-1, 1]
    steering = max(steering, -1.);
    steering = min(steering, 1.);

    std::cout << "Perror: " << p_error << " Ierror: " << i_error << " Derror: " << d_error << std::endl;

    return steering;
}

void PID::UpdateState() {  // Use twiddle algorithm

    // std::cout << "Frame: " << n_frames << std::endl;

    n_frames += 1;

    if (n_frames > n_init_frames + n_error_frames)
    {
        if (twiddle_section < -1)  // Few loops before initialization to ensure car is fast enough
        {       
            twiddle_section += 1;
        }
        
        if (twiddle_section == -1)  // it is the first initialization
        {
            best_error = std::numeric_limits<double>::max();

            // Try another set of parameters
            UpdateCurrentParameter(1.);            
            twiddle_section += 1;
        }

        else if (twiddle_section == 0)  // We are trying "parameter + d_parameter" 
        {
            if (total_error < best_error)
            {
                best_error = total_error;
                best_Kp = Kp;
                best_Ki = Ki;
                best_Kd = Kd;

                // We increase d_parameter and test next set of parameters
                UpdateCurrentOptimizationParameter(1.2);
                param_optimized = (param_optimized + 1) % 3;
                UpdateCurrentParameter(1.);
            }
            else
            {
                UpdateCurrentParameter(-2.);  // We try "parameter - d_parameter"
                twiddle_section = 1;
            }
        }
        
        else if (twiddle_section == 1)  // We are trying "parameter - d_parameter"
        {
            if (total_error < best_error)
            {
                best_error = total_error;
                best_Kp = Kp;
                best_Ki = Ki;
                best_Kd = Kd;

                // We increase d_parameter and test next set of parameters
                UpdateCurrentOptimizationParameter(1.2);
                param_optimized = (param_optimized + 1) % 3;
                UpdateCurrentParameter(1.);
                twiddle_section = 0;
            }
            else
            {
                // We set back to original parameters, decrease d_parameter and test next set
                UpdateCurrentParameter(1.);
                UpdateCurrentOptimizationParameter(0.8);
                param_optimized = (param_optimized + 1) % 3;
                UpdateCurrentParameter(1.);
                twiddle_section = 0;
            }
        }

        if ((number_loops % 20) == 0)   // We reinitialize dp and error to ensure algorithm does not get stuck
        {
            dp = 0.1 * Kp;
            di = 0.1 * Ki;
            dd = 0.1 * Kd;
            best_error = std::numeric_limits<double>::max();
            Kp = best_Kp;
            Ki = best_Ki;
            Kd = best_Kd;
        }

        if ((number_loops % 60) == 0)   // We increase speed
        {
            throttle *= 1.0;
        }

        // Output current variables
        std::cout << std::string(20, '*') << "  Parameters update  " << std::string(20, '*') << std::endl;
        std::cout << "Number loops: " << number_loops << std::endl;
        std::cout << "Twiddle section: " << twiddle_section << std::endl;
        std::cout << "Parameter optimized: " << param_optimized << std::endl;
        std::cout << "Best error: " << best_error << std::endl;
        std::cout << "Last error: " << total_error << std::endl;
        std::cout << "Kp: " << Kp << " Ki: " << Ki << " Kd: " << Kd << std::endl;
        std::cout << "dp: " << dp << " di: " << di << " dd: " << dd << std::endl;
        std::cout << "Best Kp: " << best_Kp << " Best Ki: " << best_Ki << " Best Kd: " << best_Kd << std::endl;
        std::cout << "Throttle :" << throttle << std::endl;

        // Reset variables for another loop of parameter testing
        total_error = 0.;
        n_frames = 0.;
        number_loops += 1;

    }
}

void PID::UpdateCurrentOptimizationParameter(double multiplier)
{
    if (param_optimized == 0)
    {
        dp *= multiplier;
        dp = min(dp, Kp * 0.5);  // We limit update to prevent negative Kp, Ki, Kd
    }
    if (param_optimized == 1)
    {
        di *= multiplier;
        di = min(di, Ki * 0.5);
    }
    if (param_optimized == 2)
    {
        dd *= multiplier;
        dd = min(dd, Kd * 0.5);
    }
}

void PID::UpdateCurrentParameter(double multiplier)
{
    if (param_optimized == 0)
    {
        Kp += multiplier * dp;
    }
    if (param_optimized == 1)
    {
        Ki += multiplier * di;
    }
    if (param_optimized == 2)
    {
        Kd += multiplier * dd;
    }
}