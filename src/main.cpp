#include <fstream>
#include <assert.h>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "fst.h"

using namespace std;
using json = nlohmann::json;

const int kLaneSizeMeters = 4;
const double kTimeStepSeconds = 0.02;
const double kMaxSpeedMph = 49.5;

const int kSensorFusionId = 0;
const int kSensorFusionX = 1;
const int kSensorFusionY = 2;
const int kSensorFusionVx = 3;
const int kSensorFusionVy = 4;
const int kSensorFusionS = 5;
const int kSensorFusionD = 6;

const int kNumPointsAhead = 50;

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
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Computes Euclidian distance.
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Returns the index of the point in "maps" that is closest to (x, y).
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y) {
  double closestLen = 100000; // large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); i++) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x, y, map_x, map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);
  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];
  double heading = atan2((map_y - y), (map_x - x));
  double angle = fabs(theta - heading);
  angle = min(2 * pi() - angle, angle);

  if (angle > pi() / 4) {
    closestWaypoint++;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transforms from Cartesian (x, y) coordinates to Frenet (s, d) coordinates.
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);
  int prev_wp = (next_wp == 0) ? maps_x.size() - 1 : next_wp - 1;

  double n_x = maps_x[next_wp] - maps_x[prev_wp];
  double n_y = maps_y[next_wp] - maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // Find the projection of x onto n.
  double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  // Calculate the d coordinate.
  double frenet_d = distance(x_x, x_y, proj_x, proj_y);
  // Check if d value is positive or negative by comparing it to a center point.
  double center_x = 1000 - maps_x[prev_wp];
  double center_y = 2000 - maps_y[prev_wp];
  double centerToPos = distance(center_x, center_y, x_x, x_y);
  double centerToRef = distance(center_x, center_y, proj_x, proj_y);
  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // Calculate the s coordinate.
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; i++) {
    frenet_s += distance(maps_x[i], maps_y[i], maps_x[i+1], maps_y[i+1]);
  }
  frenet_s += distance(0, 0, proj_x, proj_y);

  return {frenet_s, frenet_d};
}

// Transforms from Frenet (s, d) coordinates to Cartesian (x, y) coordinates.
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp + 1] && prev_wp < (int)(maps_s.size() - 1)) {
    prev_wp++;
  }

  int wp2 = (prev_wp + 1) % maps_x.size();
  double heading = atan2(maps_y[wp2] - maps_y[prev_wp], maps_x[wp2] - maps_x[prev_wp]);

  // The x,y,s along the segment.
  double seg_s = s - maps_s[prev_wp];
  double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
  double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

  double perp_heading = heading - pi() / 2;
  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);
  return {x, y};
}

void push_point(const double x, const double y, vector<double>* ptsx, vector<double>* ptsy) {
  ptsx->push_back(x);
  ptsy->push_back(y);
}

void push_point(const vector<double>& xy, vector<double>* ptsx, vector<double>* ptsy) {
  assert(xy.size() == 2);
  push_point(xy[0], xy[1], ptsx, ptsy);
}

double mph_to_mps(double mph) {
  return mph / 2.24;
}

double mps_to_mph(double mps) {
  return mps * 2.24;
}

double get_middle(int lane) {
  return kLaneSizeMeters / 2 + kLaneSizeMeters * lane;
}

bool is_in_lane(int lane, double d) {
  double middle = get_middle(lane);
  return (d > middle - kLaneSizeMeters / 2) && (d < middle + kLaneSizeMeters / 2);
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors.
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from.
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0.
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x;
    double y;
    float s;
    float dx;
    float dy;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> dx;
    iss >> dy; 
    push_point(x, y, &map_waypoints_x, &map_waypoints_y);
    push_point(dx, dy, &map_waypoints_dx, &map_waypoints_dy);
    map_waypoints_s.push_back(s);
  }

  double ref_v = 0;  // Reference velocity.
  FST fst(Lane::MIDDLE);

  h.onMessage([&map_waypoints_x,
               &map_waypoints_y,
               &map_waypoints_s,
               &map_waypoints_dx,
               &map_waypoints_dy,
               &ref_v, &fst](uWS::WebSocket<uWS::SERVER> ws,
                             char *data, size_t length,
                             uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    // auto sdata = string(data).substr(0, length);
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

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          if (!previous_path_x.empty()) {
             car_s = end_path_s;
          }

          // Predict where the other cars will be in the future.
          vector<double> other_car_s;
          for (const vector<double>& other_car : sensor_fusion) {
            if (is_in_lane(fst.current_lane, other_car[kSensorFusionD])) {
              double vx = other_car[kSensorFusionVx];
              double vy = other_car[kSensorFusionVy];
              double s = other_car[kSensorFusionS];

              double v = sqrt(vx * vx + vy * vy);
              s += previous_path_x.size() * kTimeStepSeconds * v;
              other_car_s.push_back(s);
            } 
          }

          Predictions predictions;
          predictions.ego_s = car_s;
          predictions.other_car_s = other_car_s;
          fst.NextState(predictions);

          if (fst.current_state == State::LANE_CHANGE_LEFT ||
              fst.current_state == State::LANE_CHANGE_RIGHT) {
            ref_v -= mps_to_mph(0.1); 
          } else if (ref_v < kMaxSpeedMph) {
            ref_v += mps_to_mph(0.1); 
          }

          // Create a list of widely spaced (x, y) waypoints, evenly spaced at 30m.
          // Later, these points will be interpolated using a spline.
          vector<double> ptsx;
          vector<double> ptsy;

          // Reference state: either where the ego car is, or at the previous path's endpoint.
          double ref_x, ref_y, ref_yaw, ref_x_prev, ref_y_prev;

          if (previous_path_x.size() < 2) {
            // Set the reference state to the current position of the ego car.
            ref_x = car_x;
            ref_y = car_y;
            ref_x_prev = car_x - cos(car_yaw);
            ref_y_prev = car_y - sin(car_yaw);
            ref_yaw = deg2rad(car_yaw);
          } else {
            // Set the reference state to the previous path's endpoint.
            ref_x = previous_path_x[previous_path_x.size() - 1];
            ref_y = previous_path_y[previous_path_y.size() - 1];
            ref_x_prev = previous_path_x[previous_path_x.size() - 2];
            ref_y_prev = previous_path_y[previous_path_y.size() - 2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
          }

          push_point(ref_x_prev, ref_y_prev, &ptsx, &ptsy);
          push_point(ref_x, ref_y, &ptsx, &ptsy);

          // In Frenet coordinates, add points that are spaced evenly 30m apart, ahead of the starting reference.
          push_point(getXY(car_s + 30, get_middle(fst.current_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y), &ptsx, &ptsy);
          push_point(getXY(car_s + 60, get_middle(fst.current_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y), &ptsx, &ptsy);
          push_point(getXY(car_s + 90, get_middle(fst.current_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y), &ptsx, &ptsy);

          // Transform the points to the ego car's local coordinates.
          // This means that the last point of the previous path is at origin (x, y, yaw) = (0, 0, 0).
          for (int i = 0; i < ptsx.size(); i++) {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            ptsx[i] = shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw);
            ptsy[i] = shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw);
          }

          // Interpolate the points.
          tk::spline spline;
          spline.set_points(ptsx, ptsy);
          
          // Define the actual points we will use for the planner.
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Start with all of the previous path points; these are the points that the car hasn't yet reached. 
          next_x_vals.insert(next_x_vals.end(), previous_path_x.begin(), previous_path_x.end());
          next_y_vals.insert(next_y_vals.end(), previous_path_y.begin(), previous_path_y.end());

          // Calculate how to break up spline points so that we travel at our reference velocity.
          double target_x = 30;
          double target_y = spline(target_x);
          double target_dist = sqrt(target_x * target_x + target_y * target_y);

          double x_offset = 0;
          for (int i = 1; i <= kNumPointsAhead - previous_path_x.size(); i++) {
            double N = target_dist / (kTimeStepSeconds * mph_to_mps(ref_v));
            double x_point = x_offset + target_x / N;
            double y_point = spline(x_point); 
            x_offset = x_point;

            // Transform from local coordinates back into map coordinates.
            double x_ref = x_point;
            double y_ref = y_point;
            x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);
            push_point(x_point + ref_x, y_point + ref_y, &next_x_vals, &next_y_vals);
          }

          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
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
