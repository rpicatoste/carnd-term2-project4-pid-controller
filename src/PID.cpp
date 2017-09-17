#include "PID.h"
#include <math.h>

using namespace std;

/*
* TODO: Complete the PID class.
*/

#define SATURATE(x, inf, sup) ( (x > sup) ? sup : ( (x < inf) ? inf : x))

PID::PID() {Init(0.0, 0.0, 0.0, 0.0, 0.0);}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, double min_control_action, double max_control_action) {
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;

	this->integral_action_prev = 0.0;
	this->error_prev = 0.0;
	this->unsaturated_control_action_prev = 0.0;
	this->saturated_control_action_prev = 0.0;

	this->min_control_action = min_control_action;
	this->max_control_action = max_control_action;

}

double PID::runPID( double error )
{
	double integral_action, proportional_action, derivative_action;
	double saturated_control_action, unsaturated_control_action;
	int is_saturating;

	proportional_action = this->Kp * error;
	derivative_action = this->Kd * (error - this->error_prev);

	// Integral action and anti-windup
	//				(there is saturation) ? yes (positive ? 1 : -1) : no;
	is_saturating = (fabs(this->unsaturated_control_action_prev - this->saturated_control_action_prev) > 0.1) ?
				    ( ((this->unsaturated_control_action_prev - this->saturated_control_action_prev) > 0) ? 1 : -1 )  :  0 ;

	// Anti-windup: Integrator clampling:
	// Do not accumulate error in the integrator if there is saturation and error in the same direction,
	// and slowly discharge it.
	if( (is_saturating > 0 && error > 0.0 ) || (is_saturating < 0 && error < 0.0 )){
		integral_action =	integral_action_prev * 0.9;
	}
	else{
		integral_action =	integral_action_prev	+  this->Ki * (error + error_prev);
	}

	unsaturated_control_action = proportional_action + derivative_action + integral_action;
	saturated_control_action = SATURATE ( unsaturated_control_action, this->min_control_action, this->max_control_action );

	// Save previous iteration values.
	this->integral_action_prev = integral_action;
	this->error_prev = error;
	this->unsaturated_control_action_prev = unsaturated_control_action;
	this->saturated_control_action_prev   = saturated_control_action;

	return -saturated_control_action;

}
