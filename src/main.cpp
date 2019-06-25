#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "MPC.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
std::vector<double> map2VehCoordTrnsf(std::vector<double>,std::vector<double>,double,double,double,int);
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
    std::cout << sdata << std::endl;
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
		  double a = j[1]["throttle"];
          /**
           * Calculate steering angle and throttle using MPC.

           * Both are in between [-1, 1].
           */
		   
		   // we have to transfer the x and y from the map coordinate
		   // into the vehicle coordinate
		   
		  std::vector<double> vTrnsf_ptsx = map2VehCoordTrnsf(ptsx,ptsy,px,py,psi,1);
          std::vector<double> vTrnsf_ptsy = map2VehCoordTrnsf(ptsx,ptsy,px,py,psi,2);
		  // creating a VectorXd from vector. We need this conversion
		  // because the polyfit funciton only acccepts VectorXd inputs
		  Eigen::Map<Eigen::VectorXd> vTrnsf_ptsx_inVectorXd(vTrnsf_ptsx.data(),vTrnsf_ptsx.size());
		  Eigen::Map<Eigen::VectorXd> vTrnsf_ptsy_inVectorXd(vTrnsf_ptsy.data(),vTrnsf_ptsy.size());
          auto coeffs = polyfit(vTrnsf_ptsx_inVectorXd,vTrnsf_ptsy_inVectorXd,3) ;
		  // cte is obtained by comparing f(x) with y, because x and y are now in 
		  // vehicle coordinate, both of the are zero for the current time
		  // therefore cte = poyeval ( coeff , 0) - 0 
		  double cte = polyeval(coeffs, 0.0) ;
		  double epsi = - atan(coeffs[1]);
		  
		  // now lets prepare the state variable for the mpc.solve function
		  Eigen::VectorXd state(6);
		  // dealing with latency 
		  double delta = j[1]["steering_angle"]; 
		  delta*=-1; //  change of sign because turning left is negative sign in simulator but positive yaw for MPC
		  //to convert miles per hour to meter per second, and you should convert ref_v too v*=0.44704;
		  psi = 0;
		  px = 0;
		  py = 0;
		  double latency = 0.1; // 100 millisecond
		  px = px + v*cos(psi)*latency; 
		  py = py + v*sin(psi)*latency;
		  cte= cte + v*sin(epsi)*latency;
		  epsi = epsi + v*delta*latency/mpc.dLf;
		  psi = psi + v*delta*latency/mpc.dLf;
		  v = v + a*latency;
		  double x = px, y = py, psii = psi, vv = v;
		  state << x, y, psii, vv, cte, epsi; 
		  
		  
		  
		  double steer_value;
          double throttle_value;
		  std::vector<double> output = mpc.Solve(state, coeffs);
		  std::vector<double> OptimalX , OptimalY;
		  std::cout<< output.size() <<std::endl;
		  if (output.size() > 0)
		  {
			steer_value = output[0] / (-deg2rad(25)) ;
			throttle_value = output[1] ;
			for (int i = 0 ; i< output.size() /2 - 4  ; i++)
			{
				OptimalX.push_back(output[2*i+2]);
				OptimalY.push_back(output[2*i+2+1]);
			}
			
		  }
		  
          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the 
          //   steering value back. Otherwise the values will be in between 
          //   [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          // Display the MPC predicted trajectory 
          std::vector<double> mpc_x_vals = OptimalX;
          std::vector<double> mpc_y_vals = OptimalY;

          /**
           * add (x,y) points to list here, points are in reference to 
           *   the vehicle's coordinate system the points in the simulator are 
           *   connected by a Green line
           */

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          // Display the waypoints/reference line
          vector<double> next_x_vals = vTrnsf_ptsx;
          vector<double> next_y_vals = vTrnsf_ptsy;

          /**
           *  add (x,y) points to list here, points are in reference to 
           *   the vehicle's coordinate system the points in the simulator are 
           *   connected by a Yellow line
           */

          msgJson["next_x"] = next_x_vals ;
          msgJson["next_y"] = next_y_vals ;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          //   the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          //   around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE SUBMITTING.
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
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

std::vector<double> map2VehCoordTrnsf(std::vector<double> ptsx,
		std::vector<double> ptsy,double px,double py,double psi,
								int indicator)
{
	std::vector<double> vTransX,vTransY;
	double dTemp = 0.0;
	if (indicator == 1) // x coordinate transfer
	{
		for (int i = 0; i < ptsx.size(); i++)
		{
			dTemp = cos(-psi)*(ptsx[i] - px) - sin(-psi)*(ptsy[i] - py);
			vTransX.push_back(dTemp);
		}
		return vTransX;
	}
	else if (indicator == 2) // y coordinate
	{
		for (int i = 0; i < ptsy.size(); i++)
		{
			dTemp = cos(-psi)*(ptsy[i] - py) + sin(-psi)*(ptsx[i] - px);
			vTransY.push_back(dTemp);
		}
		return vTransY;
	}	
}
