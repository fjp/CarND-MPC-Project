#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          // get steering angle and acceleration from simulator
          // The sign of delta needs to be inverted because you are getting
          // double delta = j[1]["steering_angle"]; from the simulator.
          // In the simulator, "left" is negative and "right" is positive,
          // while psi is measured the other way around, i.e. "left" is negative
          // (angles are measured counter-clockwise).
          double delta = j[1]["steering_angle"];
          delta = - delta;
          double acceleration = j[1]["throttle"];

          cout << "delta" << delta << endl;

          //v *= 0.447; // mph -> m/s
          // Put latency into initial state values
          // predict state in 100ms to account for the actuator (simulated) latency
          double latency = 0.1;
          px = px + v*cos(psi)*latency;
          py = py + v*sin(psi)*latency;
          psi = psi + v*delta/mpc.Lf*latency;
          v = v + acceleration*latency;


          // TODO: fit a polynomial to the above x and y coordinates
          double* pptsx = &ptsx[0];
          double* pptsy = &ptsy[0];

          Eigen::Map<Eigen::VectorXd> ptx_map(pptsx,6);
          Eigen::Map<Eigen::VectorXd> pty_map(pptsy,6);

          // Transform waypoints from map to vehicle coordinates.
          auto ptsx_vehicle = Eigen::VectorXd(ptsx.size());
          auto ptsy_vehicle = Eigen::VectorXd(ptsy.size());
          for (auto i = 0; i < ptsx.size(); ++i){

              ptsx_vehicle(i) = (ptsx[i] - px) * cos(psi) + (ptsy[i] - py) * sin(psi);
              ptsy_vehicle(i) = -(ptsx[i] - px) * sin(psi) + (ptsy[i] - py) * cos(psi);
          }

          // Fit a polynomial to upcoming waypoints
          Eigen::VectorXd coeffs = polyfit(ptsx_vehicle, ptsy_vehicle, 3);




          // TODO: calculate the cross track error in vehicle coordinates
          // The cross track error is calculated by evaluating at polynomial at x, f(x)
          // and subtracting y, which is zero in vehicle coordinates.
          //double cte = polyeval(coeffs, 0) - 0;
          // To account for the latency, the predicted states are used to calculate the cte.
          // cross track error is distance in y, from the vehicle coordinate systems's perspective
          double cte = polyeval(coeffs, px) - py;
          //cout << "cte: " << cte << endl;

          // TODO: calculate the orientation error
          // Due to the sign starting at 0, the orientation error is -f'(x).
          // derivative of coeffs[0] + coeffs[1] * x -> coeffs[1]
          //double epsi = psi - atan(coeffs[1]);

          // epsi is the difference between desired heading and actual px = 0
          //double epsi = atan(coeffs[1]+2*coeffs[2]*px+2*coeffs[3]*px*px);
          //double epsi = atan(coeffs[1]);
          double epsi = psi - atan(coeffs[1] + 2 * px * coeffs[2] + 3 * coeffs[3] *pow(px,2));


          Eigen::VectorXd state(6);
          //state << px, py, psi, v, cte, epsi;
          state << 0, 0, 0, v, cte, epsi;

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          auto vars = mpc.Solve(state, coeffs);

          // Note if δ is positive we rotate counter-clockwise,
          // or turn left. In the simulator however, a positive value implies a
          // right turn and a negative value implies a left turn.
          // One solution is to leave the update equation as is and multiply
          // the steering value by -1 before sending it back to the server.
          // The steering angle solution is given in radians but the simulator
          // expects the input to be [-1, 1]. Therefore the solution is divided
          // by 25 degress which corresponds to 0.46332 radians.
          double steer_value = -vars[6]/deg2rad(25);
          double throttle_value = vars[7];


          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          mpc_x_vals = mpc.mpc_x_vals;
          mpc_y_vals = mpc.mpc_y_vals;


          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals; //= ptsx;
          vector<double> next_y_vals; //= ptsy;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          for (auto i = 0; i < ptsx.size() ; ++i){
              next_x_vals.push_back(ptsx_vehicle(i));
              next_y_vals.push_back(ptsy_vehicle(i));
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
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
  // program
  // doesn't compile :-(
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

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
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
