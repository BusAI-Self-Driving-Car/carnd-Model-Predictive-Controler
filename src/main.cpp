#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <Eigen/Core>
#include <Eigen/QR>
#include "mpc.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    // cout << "Debug: sdata" << endl;
    // cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      // cout << "Debug: s" << endl;
      // cout << s << endl;
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          //////////////////////////////////////////////////////////////////////
          // Get the updated data
          //   - ref path, state of vehicle, and control inputs.
          //   - j[1] is the data JSON object
          //////////////////////////////////////////////////////////////////////
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double delta = j[1]["steering_angle"];
          double a = j[1]["throttle"];

          //////////////////////////////////////////////////////////////////////
          // Calculate steering angle and throttle using MPC.
          //   - Both are in between [-1, 1].
          //////////////////////////////////////////////////////////////////////

          //********************************************************************
          // transform waypoints from global coordinate to car's coordinate
          //********************************************************************
          vector<double> waypoints_x;
          vector<double> waypoints_y;

          for (int i = 0; i < ptsx.size(); i++) {
            double dx = ptsx[i] - px;
            double dy = ptsy[i] - py;
            waypoints_x.push_back(dx * cos(-psi) - dy * sin(-psi));
            waypoints_y.push_back(dx * sin(-psi) + dy * cos(-psi));
          }

          double* ptrx = &waypoints_x[0];
          double* ptry = &waypoints_y[0];

          // convert vector<double> to Eigen::Map<Eigen::VectorXd>
          Eigen::Map<Eigen::VectorXd> waypoints_x_eig(ptrx, 6);
          Eigen::Map<Eigen::VectorXd> waypoints_y_eig(ptry, 6);

          //********************************************************************
          // Fit polynomial
          //********************************************************************
          const int ORDER = 3;
          auto coeffs = polyfit(waypoints_x_eig, waypoints_y_eig, ORDER);

          //********************************************************************
          // Get the referece line points using the polynomial to display
          //********************************************************************
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          for (double i = 0; i < 100; i += 3) {
            next_x_vals.push_back(i);
            next_y_vals.push_back(polyeval(coeffs, i));
          }

          //********************************************************************
          // Get current error estimates
          // - cte:
          //   The cross track error is calculated by evaluating at polynomial at x, f(x)
          //   and subtracting y.
          // - epsi:
          //   Current heading error epsi is the tangent to the road curve at x
          //   epsi = arctan(f') where f' is the derivative of the fitted polynomial
          //   f' = coeffs[1] + 2.0*coeffs[2]*x + 3.0*coeffs[3]*x*x
          //********************************************************************
          double cte = polyeval(coeffs, 0);  // px = 0, py = 0
          double epsi = -atan(coeffs[1]);  // p

          //********************************************************************
          // Get the delayed state as the new initial state
          //********************************************************************
          // Center of gravity needed related to psi and epsi
          const double Lf = 2.67;
          // Latency for predicting time at actuation
          const double latency = 0.1;

          // Predict state after latency using the kinematic model
          // x, y and psi are all zero after transformation above
          double pred_px = 0.0 + v * latency; // Since psi is zero, cos(0) = 1, can leave out
          double pred_py = 0.0; // Since sin(0) = 0, y stays as 0 (y + v * 0 * latency)
          double pred_psi = 0.0 + v * -delta / Lf * latency; // change of sign because turning left is negative in simulator but positive for MPC
          double pred_v = v + a * latency;
          double pred_cte = cte + v * sin(epsi) * latency;
          double pred_epsi = epsi + v * -delta / Lf * latency;

          // Feed in the predicted state values
          const int NUMBER_OF_STATES = 6;
          Eigen::VectorXd state(NUMBER_OF_STATES);
          state << pred_px, pred_py, pred_psi, pred_v, pred_cte, pred_epsi;

          //********************************************************************
          // Solve for new actuations
          // And to show predicted x and y in the future
          // Using Model Predictive Control
          //********************************************************************
          mpc.Solve(state, coeffs);

          //////////////////////////////////////////////////////////////////////
          // Construct the message to the simulater:
          //   1) Control input: steering, throttle
          //   2) Predicted path points to be displayed
          //////////////////////////////////////////////////////////////////////

          json msgJson;
          // Remember to divide by deg2rad(25) before you send the steering
          // value back to keep the values within [-1, 1].
          msgJson["steering_angle"] = mpc.get_steering();
          msgJson["throttle"] = mpc.get_throttle();

          //********************************************************************
          // Display the MPC predicted trajectory
          //********************************************************************
          vector<vector<double>> predited_path = mpc.get_predicted_path();

          msgJson["mpc_x"] = predited_path[0];
          msgJson["mpc_y"] = predited_path[1];

          //********************************************************************
          // Display the waypoints/reference line
          //********************************************************************
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          //********************************************************************
          // Construct the message to be passed to the simulator / car
          //********************************************************************
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;

          //********************************************************************
          // Latency
          //   The purpose is to mimic real driving conditions where
          //   the car does actuate the commands instantly.
          //
          //   Feel free to play around with this value but should be to drive
          //   around the track with 100ms latency.
          //
          //   DEFAULT VALUE IS 100 MILLISECONDS.
          //********************************************************************
          this_thread::sleep_for(chrono::milliseconds(100));

          //********************************************************************
          // send the message to the simulator via WebSocket
          //********************************************************************
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
