#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "map.h"
#include "planner.h"
#include "utils.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;

  // Waypoint map to read from
  const std::string MAP_FILENAME = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  // const double max_s = 6945.554;
  const double kLaneWidthMeters = 4;
  const double kUpdatePeriodSeconds = 0.02;

  Map map;
  ReadMap(MAP_FILENAME, map);
  
  std::cout << "Loaded map " << MAP_FILENAME << ", total " << map.GetSize() << " points" << std::endl;

  std::unique_ptr<Planner> planner;

  h.onMessage([&planner](uWS::WebSocket<uWS::SERVER> ws, char *data,
                         size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    // auto sdata = string(data).substr(0, length);
    // cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          CarEx car{Car{Point{car_x, car_y}, car_yaw, MiphToMs(car_speed)},
                    FrenetPoint{car_s, car_d}};

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];

          std::vector<Point> unprocessedPath;
          unprocessedPath.reserve(previous_path_x.size());
          for (size_t i = 0; i < previous_path_x.size(); ++i) {
            unprocessedPath.push_back({previous_path_x[i], previous_path_y[i]});
          }

          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          FrenetPoint endPath{end_path_s, end_path_d};

          // Sensor Fusion Data, a list of all other cars on the same side of
          // the road.

          std::vector<OtherCar> sensors;

          for (const auto &input : j[1]["sensor_fusion"]) {
            int id = input[0];
            Point pos{input[1], input[2]};
            Point speed{MiphToMs(input[3]), MiphToMs(input[4])};
            FrenetPoint fnPos{input[5], input[6]};
            OtherCar ocar{id, pos, speed, fnPos};
            sensors.push_back(ocar);
          }

          auto plannedPath =
              planner->Update(car, unprocessedPath, endPath, sensors);

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for (const auto &pt : plannedPath) {
            next_x_vals.push_back(pt.x);
            next_y_vals.push_back(pt.y);
          }

          // TODO: define a path made up of (x,y) points that the car will visit
          // sequentially every .02 seconds
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          // this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  h.onConnection([&h, &planner, &map, kUpdatePeriodSeconds, kLaneWidthMeters](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
    planner = std::unique_ptr<Planner>(new Planner(map, kUpdatePeriodSeconds, kLaneWidthMeters));
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
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
