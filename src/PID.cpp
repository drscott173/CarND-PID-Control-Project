#include "PID.h"
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {
	Kp = 0;				// k for
	p_error = 0.0;		// proportional error

	Ki = 0;				// k for
	i_error = 0.0;		// integral error

	Kd = 0;				// k for
	d_error = 0.0;	    // differential error

	n = 0;				// number of steps we've run
	best_err = -1;		// best error we've achieved
	t_state = T_UP;		// twiddle state (moving parameter up, down)
	t_index = 0;        // twiddle index (which parameter p[i] are we focusing on)
	err = 0;			// cumulative squared error
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	p_error = 0.0;
	i_error = 0.0;
	d_error = 0.0;

	/* local state */
	n = 0;
	err = 0;

	/*
	 * Coefficients
	 */
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
}


double PID::Guess() {
	//
	// Return our best guess from the PID controller.
	// Return the sum of -k * p[i] for all p[i]
	//
	return -Kp*p_error - Kd*d_error - Ki*i_error;
}


void PID::UpdateError(double cte) {
	//
	// We see a new error term from our control system.
	// Update proportional, differential and integral error terms,
	// and add 1 to the iteration counter n.
	//
	d_error = (cte - p_error);
	p_error = cte;
	i_error += cte;

	n += 1;
	cout << "STEP " << n;
	cout << " error [" << p_error << ", " << d_error << ", " << i_error << "]=";
	cout << Guess() << "\r" << flush;
	if (n > PID_STARTUP) err += (cte*cte);
}

double PID::TotalError() {
	//
	// If we've passed startup conditions, return the average squared
	// error across all iterations.
	//
	if (n < PID_STARTUP) return 0.0;
	return (err/(n-PID_STARTUP));
}

/**
 * Baked-in Twiddle optimization
 */

void PID::TwiddleInit() {
	//
	// Initialize our Twiddle search for error terms.
	//
	// P contains our error term constants.
	// dp tells us how much to change p[i] either up or down.
	// t_index tells us which error term i we're working on
	// t_state tells us whether we're trying to add (up) or
	// subtract (down) dp[i] in search of a better total error.
	//
	p = {0, 0, 0};
	dp = {1, 1, 1};
	t_index = 0;
	t_state = T_UP;
	best_err = -1;
	TwiddleUpdate();
}

void PID::TwiddleUpdate() {
	//
	// We call this routine after one entire run of our
	// system, which we use to compare the performance of our
	// error terms.  We first restore all our error constants,
	// then reset our counters (n for steps, err for total error)
	// as well as our cumulative error terms for proportional,
	// differential, and integral error.
	//
	cout << "UPDATE" << endl;
	cout << "p = {" << p[0] << ", " << p[1] << ", " << p[2] << "};" << endl;
	cout << "dp = {" << dp[0] << ", " << dp[1] << ", " << dp[2] << "};" << endl;
	cout << "t_state " << (t_state == T_UP ? "UP" : (t_state == T_DOWN ? "DOWN" : "INIT")) << endl;
	cout << "t_index " << t_index << endl;

	this->Kp = p[0];
	this->Kd = p[1];
	this->Ki = p[2];
	n = 0;
	err = 0;
	p_error = 0.0;
	i_error = 0.0;
	d_error = 0.0;
}

void PID::TwiddleStep(double run_err) {
	// See documentaiton in PID.h
	//
	// "advance" tells us whether p[t_index] has reached
	// a fine enough resolution to proceed with the next
	// parameter in our series.
	//
	cout << "Twiddle step err=" << run_err << " n=" << n << " best=" << best_err << endl;
	bool advance = (dp[t_index] == 0);
	if  (!advance) {
		if (best_err == -1) {
			// first time thru
			best_err = run_err;
			t_state = T_UP;
			p[t_index] += dp[t_index];
		}
		else if ((run_err < best_err) && (n > PID_STARTUP)) {
			// new best score!
			cout << "New score: " << run_err << endl;
			best_err = run_err;
			dp[t_index] *= 1.1;
			advance = (fabs(dp[t_index]) < MIN_RESOLUTION);
			if (!advance) {
				p[t_index] += (t_state == T_UP ? 1.0 : -1.0) * dp[t_index];
			}
		}
		else if (t_state == T_UP) {
			// we were up, no joy, so try down
			p[t_index] -= 2*dp[t_index];
			t_state = T_DOWN;
		}
		else if (t_state == T_DOWN) {
			// we were down, no joy, so restore our value
			// and shift to finer grain resolution in the deltas dp.
			p[t_index] += dp[t_index];
			dp[t_index] *= 0.9;
			advance = (fabs(dp[t_index]) < MIN_RESOLUTION);
			if (!advance) {
				t_state = T_UP;
				p[t_index] += dp[t_index];
			}
		}
	}
	if (advance) {
		// t_index is being refined with sufficient resolution,
		// so shift to the next parameter.
		t_index = (t_index+1)%3;
		p[t_index] += dp[t_index];
		t_state = T_UP;
	}
}
