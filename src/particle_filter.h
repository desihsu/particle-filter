#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include "helper_functions.h"

struct Particle {
  int id;
  double x;
  double y;
  double theta;
  double weight;
  std::vector<int> associations;
  std::vector<double> sense_x;
  std::vector<double> sense_y;
};

class ParticleFilter {
public:
  ParticleFilter();
  ~ParticleFilter();

  std::vector<Particle> particles;

  const bool& initialized() const;

  // std[] is in the form of: (std of x [m], std of y [m], std of yaw [rad])
  void init(double x, double y, double theta, double std[]);

  // std_pos[] is in the form of: (std of x [m], std of y [m], std of yaw [rad])
  void prediction(double dt, double std_pos[], double velocity, double yaw_rate);

  // Finds which observations correspond to which landmarks
  void dataAssociation(std::vector<LandmarkObs> predicted, 
                       std::vector<LandmarkObs>& observations);
  
  // Updates the weights of each particle based on the likelihood
  // of the observed measurement
  void updateWeights(double sensor_range, double std_landmark[], 
                     const std::vector<LandmarkObs> &observations,
                     const Map &map_landmarks);
  
  void resample();

  std::string getAssociations(Particle best);
  std::string getSenseX(Particle best);
  std::string getSenseY(Particle best);

private:
  int num_particles; 
  bool is_initialized;
  std::vector<double> weights;
};

#endif  // PARTICLE_FILTER_H
