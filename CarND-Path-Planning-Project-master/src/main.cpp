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

  int lane = 1; //decide starting lane

  double ref_vel = 0.0; //in mph --> using it as a default speed when switching lanes, 50 is max
  // because we also start at 0 and increase over time, we won't face any jerk/accelration limit errors

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &ref_vel, &lane]
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

          json msgJson;

          int prev_size = previous_path_x.size(); //will be 50 but it can be changes

          // time for sensor fusion!! how do we incorporate that data in order to actually detect cars when driving?

          if (prev_size > 0){
            car_s = end_path_s;
          }

          bool too_close_left = false; // we'll use this bool to determine if we're too close to a car or not
          bool too_close_front = false;
          bool too_close_right = false;

          for (int i=0; i<sensor_fusion.size();i++){
            
            float d = sensor_fusion[i][6];
            int target_lane;

            if ((d > 0) && (d < 4)){
              target_lane = 0;
            }
            else if ((d > 4) && (d < 8)){
              target_lane = 1;
            }
            else if ((d > 8) && (d < 12)){
              target_lane = 2;
            }
            else {
              continue;
            }

            double vx = sensor_fusion[i][3]; //velocity of x direction
            double vy = sensor_fusion[i][4]; 
            double check_speed = sqrt(vx*vx+vy*vy); //calculate magnitude of the distance (vx, vy) vector
            double check_car_s = sensor_fusion[i][5]; //speed value of the car

            check_car_s += ((double)prev_size*0.02*check_speed); //projecting path points out --> extrapolation, where will out car be in the future?


            // checking if vehicle is right in front of us
            if ((target_lane == lane) && (check_car_s > car_s) && ((check_car_s-car_s)<30)){
              too_close_front = true;
            }

            // checking if vehicle is on the left side of us
            if ((target_lane == lane-1) && (check_car_s > car_s - 30 && check_car_s < car_s + 30)){
              too_close_left = true;
            }

            // checking if vehicle is on the right side of us
            if ((target_lane == lane+1) && (check_car_s > car_s - 30 && check_car_s < car_s + 30)){
              too_close_right = true;
            }

          } 

          if (too_close_front){
            ref_vel -= .224; 
            if (lane > 0 && !too_close_left) { //no car to the left and left lane exists 
              lane -= 1;
            }

            else if (lane != 2 && !too_close_right) { //no car to the left and left lane exists 
              lane += 1;
            }
          }

          else{

            if(ref_vel<49.5){ //remember that we're starting with a velo of 0 so we add it up over time 
            
              ref_vel += .224;

            }

          }
          
          double x = car_x;
          double y = car_y;
          double yaw = deg2rad(car_yaw);
          vector<double> ptsx;
          vector<double> ptsy;

          // if no previous path points exist, then we can base the whole thing off of the car's current points
          if (prev_size<2){

            // we're fixing the yaw values of the car in such a way that now our path planner follows the car instead of making up other stuff
            double prev_x = car_x - cos(car_yaw);
            double prev_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_y);
            ptsy.push_back(car_y);

          }
          // use previous path as the new starting point
          else {
            // indexing to most recent path point
            x = previous_path_x[prev_size-1];
            y = previous_path_y[prev_size-1];

            double x_prev = previous_path_x[prev_size-2];
            double y_prev = previous_path_y[prev_size-2];

            // taking derivatives between x and y (rate of change) and using that to calculate the yaw value
            yaw = atan2(y-y_prev, x-x_prev);

            ptsx.push_back(x_prev);
            ptsx.push_back(x);

            ptsy.push_back(y_prev);
            ptsy.push_back(y);

          }
          // now instead of calculating ahead for every single meter, we can calculate ahead in 30,60,90 meters (3 main points, fit polynomial to those measurements)
          vector<double> next_30m = getXY(car_s+30,(2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_60m = getXY(car_s+60,(2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_90m = getXY(car_s+90,(2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_30m[0]);
          ptsx.push_back(next_60m[0]);
          ptsx.push_back(next_90m[0]);

          ptsy.push_back(next_30m[1]);
          ptsy.push_back(next_60m[1]);
          ptsy.push_back(next_90m[1]);

          /*

          remember from MPC (Motion Predictive Control) that motion planning becomes so much easier when we generally make sure that the car itself is at the origin 
          of a graph rather than having ex. theta be at 45 deg. At every timestep, we ensure that our car, on the graph, is back at the origin allowing for easier 
          predictions + control. 
          */

          for (int i=0;i<ptsx.size();i++){

            double shiftx = ptsx[i]-x;
            double shifty = ptsy[i]-y;

            ptsx[i] = shiftx*cos(0-yaw)-shifty*sin(0-yaw);
            ptsy[i] = shiftx*sin(0-yaw)+shifty*cos(0-yaw);

          }

          tk::spline s; //creating our spline object

          s.set_points(ptsx,ptsy);

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // starting with previous points from before every update --> just appending to the list

          for (int i = 0; i < previous_path_x.size(); i++) {
           
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);

          }

          double target_x = 30; //remember that we want a distance of 30m minimum between each car
          double target_y = s(target_x); //we're using the in-built spline functions to map up x with y together
          double target_dist = sqrt(target_x*target_x+ target_y*target_y);
          // taking magnitude of the point vectors to get actual distances

          double x_add_on = 0; // this is like the starting point of the origin --> remember that we want to center our car along the origin

          for (int i=1; i<= 50-previous_path_x.size();i++){

            double n = (target_dist/(0.02*ref_vel/2.24)); 
            /*
            When we get our points along the spline, we need to break them up into equal landmarks at different points (ex if I wanted 30 points, then I need 30 evenly spaced points).
            This code will allow that. n is the number of points that we'll define for the car to go through
            Note: 0.02 is being multiplied since the car updates at every 0.02 seconds and we're dividing by 2.24 because this value needs to me in meters/sec, not mph.
            */

           double x_point = x_add_on+(target_x)/n; //x_point will be those x values in time where we placed our n landmarks
           double y_point = s(x_point); //calling up on spline again to map this out!! :)

           x_add_on = x_point;

           double x_ref = x_point;
           double y_ref = y_point;

          //  rotating back to the normal point (origin) after making our rotations earlier --> same thing as lines 163, 164

          x_point = x_ref*cos(yaw)-y_ref*sin(yaw);
          y_point = x_ref*sin(yaw)+y_ref*cos(yaw);

          // now just simply updating previous variables and appending them to our list of next values!

          x_point += x;
          y_point += y;

          next_x_vals.push_back(x_point);
          next_y_vals.push_back(y_point);
         
          }

          // Ending input code
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