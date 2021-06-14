/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;

using namespace std;
static default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles = 100;  // TODO: Set the number of particles

  double std_x = std[0];
	double std_y = std[1];
	double std_theta = std[2];

	std::normal_distribution<double> dist_x(x, std_x);
	std::normal_distribution<double> dist_y(y, std_y);
	std::normal_distribution<double> dist_theta(theta, std_theta);

	for(int i = 0; i < num_particles; i++) {
		Particle p;
		p.id = i;
		p.x = dist_x(gen);
		p.y = dist_y(gen);
		p.theta = dist_theta(gen);
		p.weight = 1.0;
		particles.push_back(p);
    weights.push_back(1.0);
	}
	is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
  // TODO: Add measurements to each particle and add random Gaussian noise.
  // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
  //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
  //  http://www.cplusplus.com/reference/random/default_random_engine/
  // define normal distributions for sensor noise

  normal_distribution<double> N_x(0, std_pos[0]);
  normal_distribution<double> N_y(0, std_pos[1]);
  normal_distribution<double> N_theta(0, std_pos[2]);

  for (int i = 0; i < num_particles; i++) {

    if (fabs(yaw_rate) < 0.00001) {  
      particles[i].x += velocity * delta_t * cos(particles[i].theta);
      particles[i].y += velocity * delta_t * sin(particles[i].theta);
    } 
    else {
      particles[i].x += velocity / yaw_rate * (sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta));
      particles[i].y += velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate*delta_t));
      particles[i].theta += yaw_rate * delta_t;
    }

    particles[i].x += N_x(gen);
    particles[i].y += N_y(gen);
    particles[i].theta += N_theta(gen);
  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */

  for (unsigned int i=0;i<observations.size();i++){

    LandmarkObs current_obs = observations[i];

    double min_dist = std::numeric_limits<double>::max();
    int map_id = -1;

    for (unsigned int j = 0; j < predicted.size(); j++){

      LandmarkObs current_pre = predicted[j];

      double distance = dist(current_obs.x,current_obs.y,current_pre.x,current_pre.y);

      if (distance < min_dist){
        min_dist = distance;
        map_id = current_pre.id;
        
      }

    }

    observations[i].id = map_id;
  }

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */

  for (int i = 0; i < num_particles; i++) {

    double p_x = particles[i].x;
    double p_y = particles[i].y;
    double p_theta = particles[i].theta;

    vector<LandmarkObs> final_pred;

    for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++) {

      float land_x = map_landmarks.landmark_list[j].x_f;
      float land_y = map_landmarks.landmark_list[j].y_f;
      int land_id = map_landmarks.landmark_list[j].id_i;

      if (fabs(land_x - p_x) <= sensor_range && fabs(land_y - p_y) <= sensor_range) {
        final_pred.push_back(LandmarkObs{ land_id, land_x, land_y });
      }
    }

    vector<LandmarkObs> map_conversion;

    for (unsigned int j = 0; j < observations.size(); j++) {

      double t_x = cos(p_theta)*observations[j].x - sin(p_theta)*observations[j].y + p_x;
      double t_y = sin(p_theta)*observations[j].x + cos(p_theta)*observations[j].y + p_y;

      map_conversion.push_back(LandmarkObs{ observations[j].id, t_x, t_y });
    }

    dataAssociation(final_pred, map_conversion);

    particles[i].weight = 1.0;

    for (unsigned int j = 0; j < map_conversion.size(); j++) {

      double obs_x, obs_y, pre_x, pre_y;
      obs_x = map_conversion[j].x;
      obs_y = map_conversion[j].y;

      int associated_prediction = map_conversion[j].id;

      for (unsigned int k = 0; k < final_pred.size(); k++) {
        if (final_pred[k].id == associated_prediction) {
          pre_x = final_pred[k].x;
          pre_y = final_pred[k].y;
        }
      }

      double std_x = std_landmark[0];
      double std_y = std_landmark[1];
      double obs_gaussian = (1/(2*M_PI*std_x*std_y)) * exp(-(pow(pre_x-obs_x,2)/(2*pow(std_x, 2)) + (pow(pre_y-obs_y,2)/(2*pow(std_y, 2)))));

      particles[i].weight *= obs_gaussian;
    }
  }

}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

  vector<Particle> new_particles;
  vector<double> weights;

  for (int i = 0; i < num_particles; i++) {
    weights.push_back(particles[i].weight);
  }

  uniform_int_distribution<int> particle_idx(0, num_particles-1);
  int index = particle_idx(gen);

  double max_weight = *max_element(weights.begin(), weights.end());

  double beta = 0.0;

  uniform_real_distribution<double> rand_weight(0.0, max_weight);

  for (int i = 0; i < num_particles; i++) {
    
    beta += rand_weight(gen) * 2.0;
    while (beta > weights[index]) {
      beta -= weights[index];
      index = (index + 1) % num_particles;
    }
    new_particles.push_back(particles[index]);

  }

  particles = new_particles;


}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}