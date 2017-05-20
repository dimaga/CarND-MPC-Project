#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <algorithm>
#include <cmath>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

namespace {
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
    auto b2 = s.rfind("}]");
    if (found_null != std::string::npos) {
      return "";
    } else if (b1 != std::string::npos && b2 != std::string::npos) {
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
  Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                          int order) {
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

  void transform_points(double px, double py, double psi,
                        std::vector<double>* ptsx, std::vector<double>* ptsy) {
    assert(nullptr != ptsx);
    assert(nullptr != ptsy);
    assert(ptsx->size() == ptsy->size());

    const double cos_psi = std::cos(psi);
    const double sin_psi = std::sin(psi);

    for (std::size_t i = 0, count = ptsx->size(); i < count; ++i) {
      const double ox = ptsx->at(i);
      const double oy = ptsy->at(i);

      ptsx->at(i) = (ox - px) * cos_psi + (oy - py) * sin_psi;
      ptsy->at(i) = -(ox - px) * sin_psi + (oy - py) * cos_psi;
    }
  }
}  // namespace

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    std::string sdata = std::string(data).substr(0, length);
    std::cout << sdata << std::endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      std::string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          std::vector<double> ptsx = j[1]["ptsx"];
          std::vector<double> ptsy = j[1]["ptsy"];
          const double px = j[1]["x"];
          const double py = j[1]["y"];
          const double psi = j[1]["psi"];
          const double v = j[1]["speed"];

          // Transform points to keep them in the local reference frame of the
          // car, in which px, py and psi are zero.
          transform_points(px, py, psi, &ptsx, &ptsy);

          const int order = std::min<int>(3, ptsx.size() - 1);

          Eigen::VectorXd v_ptsx(ptsx.size());
          Eigen::VectorXd v_ptsy(ptsy.size());
          for (std::size_t i = 0, size = ptsx.size(); i < size; ++i) {
            v_ptsx[i] = ptsx.at(i);
            v_ptsy[i] = ptsy.at(i);
          }

          const Eigen::VectorXd coeffs = polyfit(v_ptsx, v_ptsy, order);

          // Since we are working in the local reference frame of the car,
          // px, py and psi are zero
          const double cte = polyeval(coeffs, 0);
          const double ecte = -std::atan(coeffs[1]);

          Eigen::VectorXd state(6);
          state << 0.0, 0.0, 0.0, v, cte, ecte;

          std::vector<double> actuators = mpc.Solve(state, coeffs);

          /*
          * TODO: Calculate steeering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          const double steer_value = actuators.at(0);
          const double throttle_value = actuators.at(1);

          json msg_json;
          msg_json["steering_angle"] = steer_value;
          msg_json["throttle"] = throttle_value;

          //.. add (x,y) points to list here, points are in reference to the
          // vehicle's coordinate system the points in the simulator are
          // connected by a Green line
          msg_json["mpc_x"] = mpc.x_vals_;
          msg_json["mpc_y"] = mpc.y_vals_;

          //.. add (x,y) points to list here, points are in reference to the
          // vehicle's coordinate system the points in the simulator are
          // connected by a Yellow line

          msg_json["next_x"] = ptsx;
          msg_json["next_y"] = ptsy;


          auto msg = "42[\"steer\"," + msg_json.dump() + "]";
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
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
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
