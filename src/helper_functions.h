#ifndef HELPER_FUNCTIONS_H
#define HELPER_FUNCTIONS_H

#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include "map.h"

// Position/control measurement
struct control_s {
  
  double velocity;  // Velocity [m/s]
  double yawrate;   // Yaw rate [rad/s]
};

// Ground truth position
struct ground_truth {
  
  double x;   // Global vehicle x position [m]
  double y;   // Global vehicle y position
  double theta; // Global vehicle yaw [rad]
};

// Landmark observation measurement
struct LandmarkObs {
  
  int id;    // ID of matching landmark in the map
  double x;  // x-position of landmark observation (vehicle coordinates)
  double y;  // y-position of landmark observation (vehicle coordinates)
};

// Euclidean distance
inline double dist(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

inline double * getError(double gt_x, double gt_y, double gt_theta, 
                         double pf_x, double pf_y, double pf_theta) {
  static double error[3];
  error[0] = fabs(pf_x - gt_x);
  error[1] = fabs(pf_y - gt_y);
  error[2] = fabs(pf_theta - gt_theta);
  error[2] = fmod(error[2], 2.0 * M_PI);
  if (error[2] > M_PI) {
    error[2] = 2.0 * M_PI - error[2];
  }
  return error;
}

inline bool read_map_data(std::string filename, Map& map) {
  std::ifstream in_file_map(filename.c_str(),std::ifstream::in);
  if (!in_file_map) {
    return false;
  }
  
  std::string line_map;

  while(getline(in_file_map, line_map)){
    float landmark_x_f, landmark_y_f;
    int id_i;

    std::istringstream iss_map(line_map);
    iss_map >> landmark_x_f;
    iss_map >> landmark_y_f;
    iss_map >> id_i;

    Map::single_landmark_s single_landmark_temp;
    single_landmark_temp.id_i = id_i;
    single_landmark_temp.x_f  = landmark_x_f;
    single_landmark_temp.y_f  = landmark_y_f;

    map.landmark_list.push_back(single_landmark_temp);
  }
  return true;
}

inline bool read_control_data(std::string filename, 
                              std::vector<control_s>& position_meas) {
  std::ifstream in_file_pos(filename.c_str(),std::ifstream::in);
  if (!in_file_pos) {
    return false;
  }

  std::string line_pos;

  while(getline(in_file_pos, line_pos)){
    double velocity, yawrate;

    std::istringstream iss_pos(line_pos);
    iss_pos >> velocity;
    iss_pos >> yawrate;

    control_s meas;
    meas.velocity = velocity;
    meas.yawrate = yawrate;

    position_meas.push_back(meas);
  }
  return true;
}

inline bool read_gt_data(std::string filename, std::vector<ground_truth>& gt) {
  std::ifstream in_file_pos(filename.c_str(),std::ifstream::in);
  if (!in_file_pos) {
    return false;
  }

  std::string line_pos;

  while(getline(in_file_pos, line_pos)){
    double x, y, azimuth;

    std::istringstream iss_pos(line_pos);
    iss_pos >> x;
    iss_pos >> y;
    iss_pos >> azimuth;

    ground_truth single_gt; 
    single_gt.x = x;
    single_gt.y = y;
    single_gt.theta = azimuth;

    gt.push_back(single_gt);
  }
  return true;
}

inline bool read_landmark_data(std::string filename, 
                               std::vector<LandmarkObs>& observations) {
  std::ifstream in_file_obs(filename.c_str(),std::ifstream::in);
  if (!in_file_obs) {
    return false;
  }

  std::string line_obs;

  while(getline(in_file_obs, line_obs)){
    double local_x, local_y;

    std::istringstream iss_obs(line_obs);
    iss_obs >> local_x;
    iss_obs >> local_y;

    LandmarkObs meas;
    meas.x = local_x;
    meas.y = local_y;

    observations.push_back(meas);
  }
  return true;
}

#endif  // HELPER_FUNCTIONS_H
