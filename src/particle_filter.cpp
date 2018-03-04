/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <iostream>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>
#include <algorithm>
#include <assert.h>
#include <list>
#include <numeric>

#include "particle_filter.h"
#include <random>

const int NUM_PARTICLES = 10;

using namespace std;

ParticleFilter::ParticleFilter() : num_particles(0), is_initialized(false), generator_(rd_())
{
}

void ParticleFilter::init(double x, double y, double theta, double std[])
{
	// Setup Normal distributions for sampling
	normal_distribution<double> Gauss_x = normal_distribution<double>(x, std[0]);
	normal_distribution<double> Gauss_y = normal_distribution<double>(y, std[1]);
	normal_distribution<double> Gauss_t = normal_distribution<double>(theta, std[2]);

	num_particles = NUM_PARTICLES;
	this->particles.resize(num_particles);
	for (int i = 0; i < this->num_particles; i++)
	{
		Particle &p = particles[i];
		p.id = i;
		p.x = Gauss_x(generator_);
		p.y = Gauss_y(generator_);
		p.theta = Gauss_t(generator_);
		p.weight = 1.0;
	}
	this->is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate)
{
	// Implemet CTRV motion model
	double xp, yp, tp;
	double xp0, yp0, tp0;
	for (int i = 0; i < this->num_particles; i++)
	{
		Particle &p = particles[i];

		xp0 = particles[i].x;
		yp0 = particles[i].y;
		tp0 = particles[i].theta;
		if (fabs(yaw_rate) > 0.0001)
		{
			tp = tp0 + yaw_rate * delta_t;
			xp = xp0 + (velocity / yaw_rate) * (sin(tp) - sin(tp0));
			yp = yp0 + (velocity / yaw_rate) * (cos(tp0) - cos(tp));
		}
		else
		{
			tp = tp0;
			xp = p.x + velocity * delta_t * cos(tp0);
			yp = p.y + velocity * delta_t * sin(tp0);
		}

		// Add measurement uncertainty and update particle position
		normal_distribution<double> Gauss_x = normal_distribution<double>(xp, std_pos[0]);
		normal_distribution<double> Gauss_y = normal_distribution<double>(yp, std_pos[1]);
		normal_distribution<double> Gauss_t = normal_distribution<double>(tp, std_pos[2]);

		particles[i].x	   = Gauss_x(generator_);
		particles[i].y	   = Gauss_y(generator_);
		particles[i].theta = Gauss_t(generator_);
	}
}

/* 
	Implement Nearest Neighbor association
*/
void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	vector<double> d2;
	double dx, dy;
	for (int i = 0; i < (int) observations.size(); i++)
	{
		int index = -1;
		d2.resize(predicted.size());
		for (int j = 0; j < (int) predicted.size(); j++)
		{
			dx = (observations[i].x - predicted[j].x);
			dy = (observations[i].y - predicted[j].y);

			// since distance^2 is increasing monotonically, okay to use squared distance
			d2[j] = dx*dx + dy*dy;
		}

		// Find the nearest neighbor
		auto itr = std::min_element(d2.begin(), d2.end()); // how to handle multiple mins?
		index = (int) std::distance(d2.begin(), itr);
		double min_dist = d2[index];

		if (index > -1 && index < (int) predicted.size())
		{
			observations[i].id = predicted[index].id;
		}
		else
		{
			assert("failed to find min distance to landmark");
		}
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks)
{
	// Define working variables
	double std_x = std_landmark[0], std_y = std_landmark[1];
	double gnorm = 0.5 / (std_x*std_y);
	double total_weight = 0.0;
	double cost, sint;
	int it;

	vector<LandmarkObs> tobs(observations.size());
	for (Particle &p : this->particles)
	{
		// Transform observations in VEHICLE coords to Map coords
		cost = cos(p.theta);
		sint = sin(p.theta);
		it = 0;
		for (LandmarkObs obs : observations)
		{
			LandmarkObs &t = tobs[it++];
			t.id = obs.id;
			t.x = obs.x * cost - obs.y * sint + p.x;
			t.y = obs.x * sint + obs.y * cost + p.y;
		}

		LandmarkObs lmobs;
		vector<LandmarkObs> predicted_obs;
		double xp, yp;
		const double sr2 = sensor_range * sensor_range;
		for (const Map::single_landmark_s lm : map_landmarks.landmark_list)
		{
			// Transform in to vehicle space
			xp = lm.x_f - p.x;
			yp = lm.y_f - p.y;

			// Check bounding box for the sensor range
			if ((xp >= -sensor_range && xp <= sensor_range) && (yp >= -sensor_range && yp <= sensor_range))
			{
				// Check within range
				if ((xp*xp + yp * yp) <= sr2)
				{
					lmobs.x = lm.x_f;
					lmobs.y = lm.y_f;
					lmobs.id = lm.id_i;
					predicted_obs.push_back(lmobs);
				}
			}
		}

		// Map observations to landmarks
		dataAssociation(predicted_obs, tobs);

		// Compute likelihood of current particle position
		double ztotal = 0.0;
		double total_weight = 0.0;
		double dx, dy;
		p.associations.clear();
		p.sense_x.clear();
		p.sense_y.clear();
		for (LandmarkObs &to : tobs)
		{
			// Index to the best landmark
			int lm_index = to.id - 1;

			// check indexing is correct
			if (map_landmarks.landmark_list[lm_index].id_i == to.id) 
			{
				// Save for visualization
				p.associations.push_back(to.id);
				p.sense_x.push_back(to.x);
				p.sense_y.push_back(to.y);

				dx = (to.x - map_landmarks.landmark_list[lm_index].x_f)/std_x;
				dy = (to.y - map_landmarks.landmark_list[lm_index].y_f)/std_y;

				ztotal += -0.5 * (dx*dx+dy*dy);
			}
			else
			{
				assert(false);
			}
		}
		if (ztotal < -200)
		{
			p.weight = exp(-200);
		}
		else
		{
			p.weight = gnorm * exp(ztotal);
		}
	}

	// Normalize the weights
	total_weight = 0.0;
	for (int i = 0; i < num_particles; i++)
	{
		total_weight = total_weight + particles[i].weight;
	}
	
	if (total_weight > 0.0)
	{
		for (int i = 0; i < num_particles; i++)
		{
			particles[i].weight = particles[i].weight / total_weight;
		}
	}
}

void ParticleFilter::resample()
{
	vector<double> pdist;
	double total = 0;
	for (int i = 0; i < num_particles; i++)
	{
		pdist.push_back(particles[i].weight);
	}

	// Create New distribution
	std::discrete_distribution<int> ddist(pdist.begin(), pdist.end());

	// Resample according to the weights
	vector<Particle> resampled_particles(num_particles);

	//	vector<int> rint(num_particles);
	for (int i = 0; i < num_particles; i++)
	{
		resampled_particles[i]=particles[ddist(generator_)];
	}

	// Copy new distribution to particle filter (Reset the weights?)
	//std::copy(resampled_particles.begin(), resampled_particles.end(), this->particles.begin());


	particles = resampled_particles;

}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) 
	//           world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

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
