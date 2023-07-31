#ifndef TypeDefs_H
#define TypeDefs_H

// Eigen
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

// C++
#include <chrono>
#include <vector>

// Array of bool
typedef Eigen::Array<bool, Eigen::Dynamic, 1> ArrayXb;


// RADAR DATA ARRAYS
// ------------------------------------------------------
// Arrays with input radar data
class RadarData
{
public:
	// Data arrays
	Eigen::ArrayXXd points;			// XYZ (x=r*sin(a)*cos(e))
	Eigen::ArrayXXd spher;			// RAE (a=atan2(x,y))
	Eigen::ArrayXd  power;			// Signal intensity
	Eigen::ArrayXd  doppler;		// Doppler velocity
	ArrayXb 		inliers;		// Inliers based on Doppler velocity

	// Resize function
	void resize(const int & new_size)
	{
		this->points.resize(new_size, 3);
		this->spher.resize(new_size, 3);
		this->power.resize(new_size);
		this->doppler.resize(new_size);
		this->inliers = ArrayXb::Ones(new_size);
	}
};



// MEASURING TIME
// ------------------------------------------------------
// Class to measure time
typedef std::chrono::steady_clock timeclock_t;
typedef std::chrono::time_point<timeclock_t> timepoint_t;
typedef std::chrono::duration<double, std::milli> duration_t;

class Timing : public std::vector<timepoint_t>
{
public:
	inline void measure() {this->push_back(timeclock_t::now());}

	inline void measure(int idx) {this->at(idx) = timeclock_t::now();}

	double total_time()
	{
		duration_t duration = this->back() - this->front();
		return duration.count();
	}

	std::vector<double> breakdown()
	{
		std::vector<double> elapsed;
		for (auto it = this->begin()+1; it != this->end(); ++it)
		{
			duration_t duration = *it - *(it-1);
			elapsed.push_back(duration.count());
		}
		return elapsed;
	}
};

#endif
