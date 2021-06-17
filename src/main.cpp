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
  double max_speed = 49.5;
  double max_accel = 0.224;
  double speed_ref = 0;
  int lane = 1;
  h.onMessage([&max_accel, &max_speed, &speed_ref,&lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //double speed_ref = 0;            
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

          
          ///////	START OF MY CODE	///////

          
          double copy_speed = max_speed; 
          bool too_close = false;
          bool safe_turn_left = true;
          bool safe_turn_right = true;
          
          // Lane settings
          double lane_width = 4;
          double current_center_lane = lane*lane_width + lane_width/2;
          
          int prev_size = previous_path_x.size();
          
          //update car_s
          if (prev_size>0){
            car_s = end_path_s;
          }
          
          //Sensor fusion. Sensing the surrounding vehicles
          for (int sensed = 0; sensed<sensor_fusion.size();sensed++){
            float d = sensor_fusion[sensed][6];
            double vx = sensor_fusion[sensed][3];
            double vy = sensor_fusion[sensed][4];
            double speed_check = sqrt(vx*vx+vy*vy);
            double check_s = sensor_fusion[sensed][5];
            check_s += ((double)prev_size*0.02*speed_check);
            // Is the the same lane?
            if ((current_center_lane+lane_width/2> d ) && (d > current_center_lane-lane_width/2 )){
            //Check is vehicle too close
              if ((check_s>car_s) && ((check_s-car_s) < 30)){
                too_close = true;
                copy_speed = speed_check;
              }
            }
            
            //Is it on the adjacent left lane?
            if ((current_center_lane-lane_width/2> d ) && (d > 0 )&& (d>current_center_lane-3*lane_width/2)){
              if (((check_s-car_s) < 30) && ((check_s-car_s) > - 10)){
                safe_turn_left = false;
              }
            }
            
            //Is it on the adjacent right lane?
            if ((current_center_lane+lane_width/2< d ) && (d<current_center_lane+3*lane_width/2)){
              if (((check_s-car_s) < 30) && ((check_s-car_s) > - 10)){
                safe_turn_right = false;
              }
            }                         
          }
          
          // Behavior planning
          if (too_close){ //Event
            
            //First option: Turn left
            if (safe_turn_left &&  (lane>0)){
              lane-=1;
              std::cout <<"Left turn!"<<std::endl;
            } 
            //Second option: Turn right
            else if (safe_turn_right &&  (lane<2)){
              lane+=1;
              std::cout <<"Right turn!"<<std::endl;
            }
            //Third option: Slow down to the vehicle speed
            else{
              if (speed_ref > copy_speed){
                speed_ref -=max_accel;
              }
            }
          }
          
          //If no obstacle, speed up
          else if (speed_ref<max_speed){
            speed_ref += max_accel;            
          }
          
          /// Path planning
          
          //  Setting the spline //
          vector<double> pts_spline_x;
          vector<double> pts_spline_y;
          
          // initializing the references
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);  

          //First loop
          if (prev_size<2){
            double prev_x = car_x - cos(car_yaw);
            double prev_y = car_y -sin(car_yaw);
          
            pts_spline_x.push_back(prev_x);
            pts_spline_x.push_back(car_x);
            
            pts_spline_y.push_back(prev_y);
            pts_spline_y.push_back(car_y);
          }
          
          else{
            
            //Setting the new reference values
            ref_x = previous_path_x[prev_size-1];;
            ref_y = previous_path_y[prev_size-1];;
            double prev_x =  previous_path_x[prev_size-2];
            double prev_y =  previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-prev_y,ref_x-prev_x);
            
            //add them to the spline's points list
            pts_spline_x.push_back(prev_x);
            pts_spline_x.push_back(ref_x);
            
            pts_spline_y.push_back(prev_y);
            pts_spline_y.push_back(ref_y);            
            
          }
          
          //Setting 3 new points for the spline. Using 30m, 60m and 90m as a reference:
          
          double spline_reference_dist = 30;
          
          vector<double> spline_point_1 = getXY(car_s+1*spline_reference_dist,lane_width/2+(lane*lane_width),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> spline_point_2 = getXY(car_s+2*spline_reference_dist,lane_width/2+(lane*lane_width),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> spline_point_3 = getXY(car_s+3*spline_reference_dist,lane_width/2+(lane*lane_width),map_waypoints_s,map_waypoints_x,map_waypoints_y);

          //Add the points to the points list
          pts_spline_x.push_back(spline_point_1[0]);
          pts_spline_x.push_back(spline_point_2[0]);
          pts_spline_x.push_back(spline_point_3[0]);
          
          pts_spline_y.push_back(spline_point_1[1]);
          pts_spline_y.push_back(spline_point_2[1]);
          pts_spline_y.push_back(spline_point_3[1]);            

          // Setting to origin and aligning:        
          for (int i=0; i<pts_spline_x.size();i++){
            
            //Setting in the origin
            double shift_x =  pts_spline_x[i] - ref_x;
            double shift_y =  pts_spline_y[i] - ref_y;
            
            //Setting the yaw to 0
            pts_spline_x[i] =  (shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
            pts_spline_y[i] = (shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
            
          }
          
          // Creating the spline and setting the points
          tk::spline s;
          s.set_points(pts_spline_x,pts_spline_y);
          
          // --------- //
          
          // Set the next points
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          
          //Start with the previous points:
          for(int i = 0; i<previous_path_x.size();i++){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          //creating points with constant velocity
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_distance = sqrt(target_x*target_x + target_y*target_y);
          
          double current_x_point = 0;
          
          for (int i = 1; i<50-previous_path_x.size();i++){
            double n_step = (target_distance/(0.02*speed_ref/2.24));
            double x_point = current_x_point + (target_x/n_step);
            double y_point = s(x_point);
            current_x_point = x_point;
            
            //undo the rotation and transition
            double x_ref = x_point;
            double y_ref = y_point;
            x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
            y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));
            x_point+=ref_x;
            y_point+=ref_y;
                          
            
            //Add the points to the next_vals
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }
          
          
          ///////	END OF MY CODE	///////
          
          
          
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