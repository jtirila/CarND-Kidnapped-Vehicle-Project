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

#define X_MAX 100
#define X_MIN -100
#define Y_MAX 100
#define Y_MIN -100

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
  num_particles = 1000;
  is_initialized = true;
  double part_theta;
  double part_x;
  double part_y;

  // TODO: check the ranges
  std::uniform_real_distribution<double> my_dis_x(X_MIN, X_MAX);
  std::uniform_real_distribution<double> my_dis_y(X_MIN, X_MAX);
  std::uniform_real_distribution<double> my_dis_theta(-M_PI, M_PI);
  std::default_random_engine gen;

  for(int particle_idx = 0; particle_idx < num_particles; particle_idx++){
    part_x = my_dis_x(gen);
    part_y = my_dis_y(gen);
    part_theta = my_dis_theta(gen);
    particles.push_back(Particle(particle_idx, part_x, part_y, part_theta));
  }
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
  cout << "velocity:" << velocity << "yaw rate: " << yaw_rate << "\n";
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

  std::default_random_engine gen;
  std::normal_distribution<> dist_x(0.0, std_pos[0]);
  std::normal_distribution<> dist_y(0.0, std_pos[1]);
  std::normal_distribution<> dist_theta(0.0, std_pos[2]);
  double noise_x = dist_x(gen);
  double noise_y = dist_y(gen);
  double noise_theta = dist_theta(gen);

  Particle part;
  for(int part_id = 0; part_id < particles.size(); part_id++){
    part = particles[part_id];
    part.x += velocity / yaw_rate * (std::sin(part.theta + yaw_rate * delta_t) - std::sin(part.theta)) + noise_x;
    part.y += velocity / yaw_rate * (- std::cos(part.theta + yaw_rate * delta_t) + std::cos(part.theta)) + noise_y;
    part.theta += yaw_rate * delta_t + noise_theta;
  }


}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.

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

  // A variable to hold for each measurement the closest observation

  for(int p_ind = 0; p_ind < particles.size(); p_ind++){
    // Keep the particle in variable for brevity
    Particle particle = particles[p_ind];

    // Also keep a version of the particle where we just save the x, y and theta values into a std::vector
    std::vector<double> particle_vec = {particle.x, particle.y, particle.theta};

    for(int i = 0; i < observations.size(); i++){
      std::vector<double> obs_vec = {observations[i].x, observations[i].y};
      std::vector<double> tobs_vec = car_map_transform(obs_vec, particle_vec);
      tobss.push_back(tobs_vec);
      Map::single_landmark_s closest_obs = FindClosestLandmark(map_landmarks.landmark_list, tobs_vec);
      particle.weight = EvaluateGaussian(tobs_vec, closest_obs);
    }
  }


}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
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
