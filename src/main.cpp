#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

#define TUNE_STEERING 0      // Set to 1 to use Twiddle and tune steering parameters
#define TUNE_SPEED 0		 // Same for speed
#define MAX_STEPS 10000	     // The number of steps for one "run"

// for convenience
using json = nlohmann::json;
using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_last_of("]");
	if (found_null != std::string::npos) {
		return "";
	}
	else if (b1 != std::string::npos && b2 != std::string::npos) {
		return s.substr(b1, b2 - b1 + 1);
	}
	return "";
}

int main()
{
	uWS::Hub h;
	double max_speed;

	PID pid_steer, pid_throttle;
	pid_steer.TwiddleInit();
	pid_throttle.TwiddleInit();

	/* initialize our controllers */
	//pid.p = {0.0875913, 5.80869e-05, 0.0223804};

	// PID controller for steering
	pid_steer.p = {0.158161, 1.69977, 0.000489072};
	pid_steer.dp = {2.45227e-05, 2.68442e-05, 2.23031e-05};

	// PID controller for throttle
	pid_throttle.p = {-0.353402, 3.80884, -0.000491255};
	pid_throttle.dp = {6.25688e-08, 2.601e-07, 6.78076e-08};

	pid_steer.TwiddleUpdate();
	pid_throttle.TwiddleUpdate();
	max_speed = 0;

	h.onMessage([&pid_steer, &pid_throttle, &max_speed]
				 (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event

		if (length && length > 2 && data[0] == '4' && data[1] == '2')
		{
			auto s = hasData(std::string(data).substr(0, length));
			if (s != "") {
				auto j = json::parse(s);
				std::string event = j[0].get<std::string>();
				if (event == "telemetry") {
					// j[1] is the data JSON object
					double cte = std::stod(j[1]["cte"].get<std::string>());
					double speed = std::stod(j[1]["speed"].get<std::string>());
					double angle = std::stod(j[1]["steering_angle"].get<std::string>());
					double steer_value, throttle, max_speed, n;
					max_speed = max(speed, max_speed);
					n = pid_steer.n; // number of iterations

					/*
					 * TODO: Calculate steering value here, remember the steering value is
					 * [-1, 1].
					 * NOTE: Feel free to play around with the throttle and speed. Maybe use
					 * another PID controller to control the speed!
					 */
					pid_steer.UpdateError(cte);
					pid_throttle.UpdateError(cte);
					steer_value = pid_steer.Guess();
					steer_value = max(-1.0, min(1.0, steer_value));

					// shift throttle to be centered on 0.3 with an interval [-1, 1]
					// to give us [-0.7, 1.3]
					throttle = -0.2+((max(-1.0, min(1.0, pid_throttle.Guess())) + 1.0)/2.0);

					// cout << endl << "Steer: " << steer_value << endl;

					if ((n > PID_STARTUP) &&    		// we've passed startup noise
							((fabs(cte) > 3.5) || 		// off road
									(speed < 3) || 		// too slow
									(n > MAX_STEPS) 	// finished a trial
							)) {

						double score = pid_steer.TotalError()-n-max_speed;

						cout << "Off track!  Resetting at n=" << n << " max_speed=" << max_speed;
						cout << " Error=" << pid_steer.TotalError() << endl;
						std::string reset_msg = "42[\"reset\",{}]";
						ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);

						// Tweak our parameters if needed
						if (TUNE_STEERING) pid_steer.TwiddleStep(score);
						if (TUNE_SPEED) pid_throttle.TwiddleStep(score);
						cout << "STEERING" << endl;
						pid_steer.TwiddleUpdate();
						cout << "THROTTLE" << endl;
						pid_throttle.TwiddleUpdate();
					}

					// DEBUG
					// std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

					json msgJson;
					msgJson["steering_angle"] = steer_value;
					msgJson["throttle"] = throttle;
					auto msg = "42[\"steer\"," + msgJson.dump() + "]";
					// std::cout << msg << std::endl;
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				}
			} else {
				// Manual driving
				std::string msg = "42[\"manual\",{}]";
				ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			}
		}
	});

	// We don't need this since we're not using HTTP but if it's removed the program
	// doesn't compile :-(
	h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
		const std::string s = "<h1>Hello fun world!</h1>";
		if (req.getUrl().valueLength == 1)
		{
			res->end(s.data(), s.length());
		}
		else
		{
			// i guess this should be done more gracefully?
			res->end(nullptr, 0);
		}
	});

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
		std::cout << "Connected yay!!!" << std::endl;
	});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
		ws.close();
		std::cout << "Disconnected" << std::endl;
	});

	int port = 4567;
	if (h.listen(port))
	{
		std::cout << "Listening to port " << port << std::endl;
	}
	else
	{
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}
	h.run();
}
