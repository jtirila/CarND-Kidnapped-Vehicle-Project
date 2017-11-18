/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"
#include "map.h"

#define X_TOLERANCE 5
#define Y_TOLERANCE 5

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
  num_particles = 50;
  is_initialized = true;
  double part_theta;
  double part_x;
  double part_y;

  // TODO: check the ranges
  std::normal_distribution<> dist_x(0.0, X_TOLERANCE);
  std::normal_distribution<> dist_y(0.0, Y_TOLERANCE);
  std::normal_distribution<> dist_theta(0.0, 0.1);
  std::random_device rd;
  std::default_random_engine gen(rd());

  for(int particle_idx = 0; particle_idx < num_particles; particle_idx++){
    part_x = x + dist_x(gen);
    part_y = y + dist_y(gen);
    part_theta = theta + dist_theta(gen);
    particles.push_back(Particle(particle_idx, part_x, part_y, part_theta));
  }
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
  cout << "velocity:" << velocity << "yaw rate: " << yaw_rate << "\n";
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

  std::random_device rd;
  std::default_random_engine gen(rd());
  std::normal_distribution<> dist_x(0.0, 5 * std_pos[0]);
  std::normal_distribution<> dist_y(0.0, 5 * std_pos[1]);
  std::normal_distribution<> dist_theta(0.0, 2 * std_pos[2]);

  Particle* part;
  for(int part_id = 0; part_id < particles.size(); part_id++){
    double noise_x = dist_x(gen);
    double noise_y = dist_y(gen);
    double noise_theta = dist_theta(gen);
    part = &particles[part_id];
    part->theta += yaw_rate * delta_t + noise_theta;
    if(yaw_rate > 0.00001) {
      part->x += velocity / yaw_rate * (std::sin(part->theta + yaw_rate * delta_t) - std::sin(part->theta)) + noise_x;
      part->y += velocity / yaw_rate * (-std::cos(part->theta + yaw_rate * delta_t) + std::cos(part->theta)) + noise_y;
    } else {
      part->x += velocity * std::cos(part->theta) * delta_t + noise_x;
      part->y += velocity * std::sin(part->theta) * delta_t + noise_y;
    }
  }
}

void dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations){
Map::single_landmark_s ParticleFilter::FindClosestLandmark(const std::vector<Map::single_landmark_s> &landmarks, const std::vector<double> &tobs){
  double min_dist;
  int min_ind = 0;
  for(int i = 0; i < landmarks.size(); i++){
    std::vector<double> landmark_vec = {landmarks[i].x_f, landmarks[i].y_f};
    double current_dist = VectorDist(landmark_vec, tobs);
    // std::cout << "\nindex " << i << ": dist between (" << landmarks[i][0] << ", " << landmarks[i][1] << ") and (" <<
    //              tobs[0] << ", " << tobs[1] << "): " <<  current_dist << "; min so far: " << min_dist;
    if(i > 0) {
      if (current_dist < min_dist) {
        min_ind = i;
        min_dist = current_dist;
      }
    }
    else {
      min_ind = 0;
      min_dist = current_dist;
    }
  }
  return landmarks[min_ind];
}


void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

  // Container for transformed observations:
  std::vector<std::vector<double>> tobss;

  // A variable to hold the closest observation
  std::vector<LandmarkObs> closest_obss;

  for(int p_ind = 0; p_ind < particles.size(); p_ind++){
    // Keep the particle in variable for brevity
    Particle particle = particles[p_ind];
    particle.weight = 1.0;
    std::vector<int> ass_idx;
    std::vector<double> ass_x;
    std::vector<double> ass_y;

    // Also keep a version of the particle where we just save the x, y and theta values into a std::vector
    std::vector<double> particle_vec = {particle.x, particle.y, particle.theta};

    for(int i = 0; i < observations.size(); i++){
      std::vector<double> obs_vec = {observations[i].x, observations[i].y};
      std::vector<double> tobs_vec = car_map_transform(obs_vec, particle_vec);
      tobss.push_back(tobs_vec);
      Map::single_landmark_s closestLandmark = FindClosestLandmark(map_landmarks.landmark_list, tobs_vec);
      particle.weight *= EvaluateGaussian(tobs_vec, closestLandmark);
      ass_idx.push_back(closestLandmark.id_i);
      ass_x.push_back(tobs_vec[0]);
      ass_y.push_back(tobs_vec[1]);
    }
    // SetAssociations(particle, ass_idx, ass_x, ass_y);
    particles[p_ind] = particle;
  }
}


void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

  ExtractWeights();

  std::random_device rd;
  std::default_random_engine gen(rd());
  std::discrete_distribution<int> distr(weights.begin(), weights.end());
  std::vector<Particle> p2;


  for(int i = 0; i < num_particles; i++){
    p2.push_back(particles[distr(gen)]);
  }
  this->particles = p2;

}

void ParticleFilter::ExtractWeights(){
  std::vector<double> new_weights;
  for(const Particle& part : particles){
   new_weights.push_back(part.weight);
  }
  weights.swap(new_weights);
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
