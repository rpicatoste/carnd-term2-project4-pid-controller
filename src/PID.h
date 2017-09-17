#ifndef PID_H
#define PID_H

class PID {
public:
	/*
	* Values to be stored from one iteration to the next.
	*/
	double integral_action_prev;
	double error_prev;
	double unsaturated_control_action_prev;
	double saturated_control_action_prev;

	/*
	* Coefficients
	*/
	double Kp;
	double Ki;
	double Kd;

	// Min and max control action values.
	double min_control_action;
	double max_control_action;

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
	void Init(double Kp, double Ki, double Kd, double min_control_action, double max_control_action);

	/*
	* Run the PID controller
	*/
	double runPID( double error );

};

#endif /* PID_H */
