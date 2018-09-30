#include "particle_filter.h"
#include <algorithm>
#include <iostream>
#include <iterator>
#include <math.h>
#include <numeric>
#include <random>
#include <sstream>
#include <string>
#include "helper_functions.h"

static std::default_random_engine gen;

ParticleFilter::ParticleFilter() : num_particles(0), is_initialized(false) {}

ParticleFilter::~ParticleFilter() {}

const bool& ParticleFilter::initialized() const {
  return is_initialized;
}

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  num_particles = 100;
  std::normal_distribution<double> dist_x(x, std[0]);
  std::normal_distribution<double> dist_y(y, std[1]);
  std::normal_distribution<double> dist_theta(theta, std[2]);

  for (int i = 0; i < num_particles; ++i) {
    particles.push_back(Particle{ i, dist_x(gen), dist_y(gen), 
                                  dist_theta(gen), 1.0 });
    weights.push_back(1.0);
  }

  is_initialized = true;
}

void ParticleFilter::prediction(double dt, double std_pos[], 
                                double v, double yaw_rate) {
  std::normal_distribution<double> noise_x(0, std_pos[0]);
  std::normal_distribution<double> noise_y(0, std_pos[1]);
  std::normal_distribution<double> noise_theta(0, std_pos[2]);

  for (auto p = particles.begin(); p < particles.end(); ++p) {
    if (fabs(yaw_rate) < 0.00001) {
      p->x += v * dt * cos(p->theta) + noise_x(gen);
      p->y += v * dt * sin(p->theta) + noise_y(gen);
      p->theta += noise_theta(gen);
    }
    else {
      p->x += ((v / yaw_rate) * (sin(p->theta + yaw_rate * dt) - 
               sin(p->theta)) + noise_x(gen));
      p->y += ((v / yaw_rate) * (cos(p->theta) - 
               cos(p->theta + yaw_rate * dt)) + noise_y(gen));
      p->theta += yaw_rate * dt + noise_theta(gen);
    }
  }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, 
                                     std::vector<LandmarkObs>& observations) {
  for (auto o = observations.begin(); o < observations.end(); ++o) {
    auto nearest = std::min_element(predicted.begin(), predicted.end(), 
                            [&](const LandmarkObs& a, const LandmarkObs& b) {
                                  return (dist(a.x, a.y, o->x, o->y) < 
                                          dist(b.x, b.y, o->x, o->y)); });
    o->id = nearest - predicted.begin();
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std[], 
                                   const std::vector<LandmarkObs> &observations, 
                                   const Map &map) {
  for (auto p = particles.begin(); p < particles.end(); ++p) {
    std::vector<LandmarkObs> predicted, transformed;
    std::vector<int> associations;
    std::vector<double> sense_x, sense_y;
    p->weight = 1.0;

    for (auto l = map.landmark_list.begin(); l < map.landmark_list.end(); ++l) {
      if ((fabs(p->x - l->x_f) <= sensor_range && 
           fabs(p->y - l->y_f) <= sensor_range)) {
        predicted.push_back(LandmarkObs{ l->id_i, l->x_f, l->y_f });
      }
    }

    for (auto o = observations.begin(); o < observations.end(); ++o) {
      double x = p->x + cos(p->theta) * o->x - sin(p->theta) * o->y;
      double y = p->y + sin(p->theta) * o->x + cos(p->theta) * o->y;
      transformed.push_back(LandmarkObs{ o->id, x, y });
    }

    dataAssociation(predicted, transformed);

    for (auto o = transformed.begin(); o < transformed.end(); ++o) {
      int i = o->id;
      double x = predicted[i].x;
      double y = predicted[i].y;

      p->weight *= (exp(-pow(x - o->x, 2) / (2 * pow(std[0], 2)) - 
                        pow(y - o->y, 2) / (2 * pow(std[1], 2))));
      p->weight /= 2 * M_PI * std[0] * std[1];
      associations.push_back(predicted[i].id);
      sense_x.push_back(x);
      sense_y.push_back(y);
    }

    weights[p-particles.begin()] = p->weight;
    p->associations = associations;
    p->sense_x = sense_x;
    p->sense_y = sense_y;
  }
}

void ParticleFilter::resample() {
  std::vector<Particle> samples;
  std::discrete_distribution<> dist(weights.begin(), weights.end());

  for (int i = 0; i < num_particles; ++i) {
    samples.push_back(particles[dist(gen)]);
  }

  particles = samples;
}

std::string ParticleFilter::getAssociations(Particle best) {
  std::vector<int> v = best.associations;
  std::stringstream ss;
  std::copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  std::string s = ss.str();
  s = s.substr(0, s.length()-1);
  return s;
}

std::string ParticleFilter::getSenseX(Particle best) {
  std::vector<double> v = best.sense_x;
  std::stringstream ss;
  std::copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  std::string s = ss.str();
  s = s.substr(0, s.length()-1);
  return s;
}

std::string ParticleFilter::getSenseY(Particle best) {
  std::vector<double> v = best.sense_y;
  std::stringstream ss;
  std::copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  std::string s = ss.str();
  s = s.substr(0, s.length()-1);
  return s;
}
