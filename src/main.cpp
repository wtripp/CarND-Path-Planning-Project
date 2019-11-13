#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
  
  // TODO: start in lane 1
  int lane = 1;
  
  // TODO: Have a reference velocity to target
  double ref_vel = 0; // mph

  h.onMessage([&ref_vel, &lane, &map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          // TODO: Previous list of points (i.e., last path car was following
          // before we calculate more points).
          int prev_size = previous_path_x.size();
          
          // TODO: Collision avoidance
          
          // Work with Frenet coordinates
          // If there are waypoints, set car's s to last waypoint in path
          if(prev_size > 0) {
            car_s = end_path_s;
          }
          
          // Initialize Booleans used for lane changing logic.
          bool too_close = false;
          bool free_right = true;
          bool free_left = true;
          
          // Find ref_x to use
          for(int i = 0; i < sensor_fusion.size(); i++) {
            
            // d value of other car, to check what lane it's in
            float d = sensor_fusion[i][6];

            // velocity and velocity magnitude of other car, to check where it will be
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx + vy*vy); // velocity magnitude

            // s value of other car, to check how close it is
            double check_car_s = sensor_fusion[i][5];
            
            // ego lane range. +-2 is one-half of each lane            
            double left_boundary = 2+4*lane-2;
            double right_boundary = 2+4*lane+2;

            // If using prev. path points, project s value out, 
            // predict where car will be, i.e, will it hit any other cars?
            check_car_s += ((double)prev_size*0.02*check_speed);
            
            // Check current lane
            if(d > left_boundary && d < right_boundary) {

              // is car in front and gap is less than 30 m?
              if(check_car_s > car_s && check_car_s - car_s < 30) {
                too_close = true;
              }
            }
              
            // Check left lane
            if(d < left_boundary) {
              
              // is car in range 50 m behind or 100 m in front of ego?
              if((check_car_s > car_s && check_car_s - car_s > 100) ||
                 (check_car_s < car_s && check_car_s - car_s < -50)) {
                free_left == false;
              }
            }
            
            
            if(d > right_boundary) {
              
              // is car in range 50 m behind or 100 m in front of ego?
              if((check_car_s > car_s && check_car_s - car_s > 100) ||
                 (check_car_s < car_s && check_car_s - car_s < -50)) {
                free_right == false;
              }
            }
            
            

          }
          // If a vehicle is too close, and going above a 
          // safe speed when close to a car (29.5 mph)
          // decrement speed 1 m/s at a time.
          if(too_close && ref_vel > 29.5) {
            ref_vel -= 0.224;

            if(free_right && lane < 2) {
              lane += 1;
            }

            if(free_left && lane > 0) {
              lane -= 1;
            }
          }
          // Otherwise increment speed up to max ref velocity
          // This also addresses start of simulation, so that car
          // doesn't increase speed too quickly.

          // Could improve by having it increase at every waypoint, rather than
          // at a constant rate.
          else if(ref_vel < 49.5) {
            ref_vel += 0.224;
          }
          // TODO: Initialize a list of sparse, widely spaced (x,y) wapoints, evenly spaced
          // at 30m. Later, we interpolate these waypoints with a spline and
          // fill it with more points.
          vector<double> ptsx;
          vector<double> ptsy;
          
          // TODO: Reference x,y,yaw states
          // Either we reference the starting point as where the car is or
          // at the previous paths endpoint.
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          // TODO: Set up starting points:
          // Get last few points from previous path, calculate what angle
          // the car WAS heading in based on those points, and push them on to
          // a list of two previous points.
          
          // if previous set of points is almost empty, use car as starting reference
          if(prev_size < 2) {
            
            // Use two points that make the path tangent to angle of the car.
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          
          //Otherwise use last 2 points of path as starting reference
          else {
            
            // Redefine reference state as previous path endpoint.
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];
            
            // Also take previous state, i.e., second-to-last points
            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
            
            // Use two points that make the path tangent to the
            // previous car's endpoint
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }
          
          // In Frenet add 3 evenly spaced 30m points ahead of starting reference
          // Incorporates lane changes into it.
          vector<double> next_waypoint0 = getXY(car_s+30, (2+4*lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_waypoint1 = getXY(car_s+60, (2+4*lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_waypoint2 = getXY(car_s+90, (2+4*lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          ptsx.push_back(next_waypoint0[0]);
          ptsx.push_back(next_waypoint1[0]);
          ptsx.push_back(next_waypoint2[0]);

          ptsy.push_back(next_waypoint0[1]);
          ptsy.push_back(next_waypoint1[1]);
          ptsy.push_back(next_waypoint2[1]);


          // TODO: Shift path to vehicle coordinate system, so last point
          // of previous path (i.e., car's location is at (0,0) with yaw 0.
          // Makes the math easier.
          for(int i = 0; i < ptsx.size(); i++) {

            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            
            ptsx[i] = (shift_x*cos(0 - ref_yaw) - shift_y*sin(0 - ref_yaw));
            ptsy[i] = (shift_x*sin(0 - ref_yaw) + shift_y*cos(0 - ref_yaw));
            
          }
          
          // TODO: Create a spline.
          tk::spline s;
          
          // TODO: Set (x,y) original sparse "anchor" way points to the spline.
          s.set_points(ptsx, ptsy);
          
          // TODO: Build the future path.
          // Start by defining the actual (x,y) points we'll use for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          // TODO: Start with all the previous path points.
          for(int i = 0; i < previous_path_x.size(); i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          // TODO: Calculate how to break up spline points so that we travel
          // at our desired reference velocity, i.e., get points along spline.
          // Need to be spaced so their at the exact right speed for our speed limit.
          
          // look out by some horizon values
          double target_x = 30.0;
          
          // Find this point on our spline.
          double target_y = s(target_x);
          
          // Calculate linear distance from car at origin to this horizon point 
          double target_dist = sqrt(target_x*target_x + target_y*target_y);
          
          
          // N * 0.02 s * desired_vel m/s = d m
          // How much x goes up w/ each point. Starts at 0.
          double x_add_on = 0;
          
          // Add points along spline.
          for(int i = 1; i <= 50-previous_path_x.size(); i++) {
            
            // Calculate # spline points if car travels 0.02 m/s at ref vel.
            // (50 m/s * 0.02 s = 1 m). 2.24 is for mph to m/s calc.
            double N = (target_dist/(0.02*ref_vel/2.24));
            
            // Get next x value
            double x_point = x_add_on+(target_x)/N;
            
            // Calculate spline at each point.
            double y_point = s(x_point);
            
            // Reset x_add_on and ref points
            x_add_on = x_point;
            
            double x_ref = x_point;
            double y_ref = y_point;
            
            // Rotate back to original rotation
            x_point = (x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw));
            
            x_point += ref_x;
            y_point += ref_y;
            
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);

          }
          
          json msgJson;

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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