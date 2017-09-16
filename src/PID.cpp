#include "PID.h"
#include <math.h>

using namespace std;

/*
* TODO: Complete the PID class.
*/

#define CONTROL_ACTION_MAX (1.0)
#define CONTROL_ACTION_MIN (-1.0)
#define SATURATE(x, inf, sup) ( (x > sup) ? sup : ( (x < inf) ? inf : x))
#define SATURATE_CONTROL_ACTION(x) SATURATE(x, CONTROL_ACTION_MIN, CONTROL_ACTION_MAX)

PID::PID() {Init(0.0,0.0,0.0);}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;

	this->p_error = 0.0;
	this->i_error = 0.0;
	this->d_error = 0.0;
	this->cte_prev = 0.0;
}

void PID::UpdateError(double cte) {
	this->p_error = cte;
	this->i_error = cte - this->cte_prev;
	this->d_error += cte;

	this->cte_prev = cte;
}

double PID::TotalError() {
	return 1e6;
}


double PID::runPID( double error )
{/*
	this->UpdateError( error );

	double control_action;

	control_action = - this->Kp * this->p_error
					 - this->Kd * this->d_error
					 - this->Ki * this->i_error;

	control_action = SATURATE_CONTROL_ACTION(control_action);

	return control_action;

*/
	double saturated_control_action, unsaturated_control_action, integral_action, proportional_action, derivative_action;
	static double error_prev = 0.0;
	static double integral_action_prev = 0.0, proportional_action_prev = 0.0;
	static double unsaturated_control_action_prev = 0.0, saturated_control_action_prev = 0.0;
	int is_saturating;

	//				(there is saturation) ? yes (positive ? 1 : -1) : no;
	is_saturating = (fabs(unsaturated_control_action_prev - saturated_control_action_prev) > 0.1) ?
				    ( ((unsaturated_control_action_prev - saturated_control_action_prev) > 0) ? 1 : -1 )  :  0 ;


	proportional_action = this->Kp * error;

	derivative_action = this->Kd * (error - error_prev);


	// Integrator anti-windup: Integrator clampling:
	// Do not accumulate error in the integrator if there is saturation and error in the same direction,
	// and slowly discharge it.
	if( (is_saturating > 0 && error > 0.0 ) || (is_saturating < 0 && error < 0.0 )){
		integral_action =	integral_action_prev * 0.9;
	}
	else{
		integral_action =	integral_action_prev	+  this->Ki * (error + error_prev);
	}

	unsaturated_control_action = proportional_action + derivative_action + integral_action;

	// Saturate the output between the steering limits.
	saturated_control_action = SATURATE_CONTROL_ACTION ( unsaturated_control_action );
	integral_action_prev = integral_action;
	error_prev = error;
	unsaturated_control_action_prev = unsaturated_control_action;



	saturated_control_action_prev = saturated_control_action;

	return -saturated_control_action;

}
