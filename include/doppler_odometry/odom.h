// Eigen
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>

// Type definitions
#include "typedefs.h"

// OpenMP
#include <omp.h>

// C++
#include <chrono>
#include <iostream>
#include <deque>


class Odometry
{
public:
	// Constructor
	Odometry();
	
	// Option variables
	bool verbose = true;
	int min_points = 50;
	bool use_power = true;
	std::string threshold_mode = "percent"; // percent, mad, fixed, ransac
	double threshold_value = 0.7;
	double length_between_wheels = 0.505;	// Husky
	double max_linear_vel = 1.5;
	double max_ang_vel = 1.5;
	
	// Calibration parameters
	Eigen::Vector3d sensor_pos = Eigen::Vector3d::Zero();
	Eigen::Quaterniond sensor_rot = Eigen::Quaterniond::Identity();
	Eigen::Quaterniond calib_rot = Eigen::Quaterniond::Identity();
	double x_diff = -0.9944683540059478;

	// Number of frames processed
	int number_frames;

	// Average execution time
	Timing time_points;
	double avg_exec_time;
	Eigen::ArrayXd avg_elapsed;

	// Doppler velocity
	Eigen::Vector3d dopp_vel;
	Eigen::Matrix3d dopp_cov;
	
	// Angular velocity
	Eigen::Vector3d ang_vel;
	Eigen::Matrix3d ang_cov;

	// Data matrices
	RadarData data;

	// Stored previous sets of cells
	int num_previous;

	// Stamps (nanoseconds) of each message
	long unsigned int prev_stamp;
	long unsigned int curr_stamp;

	// Local transformation and pose
	Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d transform;
	Eigen::Matrix4d old_transform;

protected:
	// Initialize
	void initialize();

	// Doppler velocity and inliers pointer function
	std::function<void(RadarData&, Eigen::Vector3d&, Eigen::Matrix3d&)> dopplerVelocity;
	// Doppler velocity and inliers using Ransac
	void dopplerVelocityRansac(RadarData & data, Eigen::Vector3d & vel, Eigen::Matrix3d & covar);
	// Doppler velocity and inliers using TLS
	void dopplerVelocityTls(RadarData & data, Eigen::Vector3d & vel, Eigen::Matrix3d & covar);

	// Perform all odometry tasks
	void odometryDopplerOnly();



	// HELPER FUNCTIONS
	// Degrees to radians
	template <typename Derived>
	Derived deg2rad(const Derived in)
	{
		Derived out = in * Derived(M_PI) / Derived(180.);
		return out;
	}

	 // Radians to degrees
	template <typename Derived>
	Derived rad2deg(const Derived in)
	{
		Derived out = in * Derived(180.) / Derived(M_PI);
		return out;
	}

	 // Calculate hat matrix
	template <typename Derived>
	Eigen::Matrix<typename Derived::Scalar, 3, 3> hatMatrix(const Eigen::MatrixBase<Derived>& in)
	{
		// Create output matrix
		Eigen::Matrix<typename Derived::Scalar, 3, 3> out;

		// Check right size of vector
		if (in.size() != 3)
		{
			std::cout << "Cannot hat operator matrix of " << in.rows() << "x" << in.cols() << std::endl;
			out.setZero();
			return out;
		}

		// Create and output
		out << 0., -in(2), in(1),
				in(2), 0., -in(0),
				-in(1), in(0), 0.;
		return out;
	}

     // Calculate T matrix from twist vector
    template <typename Derived>
    Eigen::Matrix<typename Derived::Scalar, -1, -1> calculateTransform(const Eigen::MatrixBase<Derived>& chi)
    {
        Eigen::Matrix<typename Derived::Scalar, 4, 4> T, Tchi;
        Tchi.setZero();
        Tchi(0,1) = -chi(5); Tchi(0,2) =  chi(4);
        Tchi(1,0) =  chi(5); Tchi(1,2) = -chi(3);
        Tchi(2,0) = -chi(4); Tchi(2,1) =  chi(3);

        #if 1
        // Pseudo exponential
        T = Tchi.exp();
        T(0,3) = chi(0); T(1,3) = chi(1); T(2,3) = chi(2);
        #else
        // Real exponential
        Tchi(0,3) = chi(0); Tchi(1,3) = chi(1); Tchi(2,3) = chi(2);
        T = Tchi.exp();
        #endif

        return T;
    }

     // Calculate twist vector from T matrix
    template <typename Derived>
    Eigen::Matrix<typename Derived::Scalar, -1, -1> calculateTwist(const Eigen::MatrixBase<Derived>& T)
    {
        Eigen::Matrix<typename Derived::Scalar, 6, 1> chi;
        Eigen::Matrix<typename Derived::Scalar, 4, 4> Tchi;
        Tchi = T.log();

        #if 1
        // Pseudo logarithm
        chi(0) = T(0,3);    chi(1) = T(1,3);    chi(2) = T(2,3);
        chi(3) = Tchi(2,1); chi(4) = Tchi(0,2); chi(5) = Tchi(1,0);
        #else
        // Real logarithm
        chi(0) = Tchi(0,3);    chi(1) = Tchi(1,3);    chi(2) = Tchi(2,3);
        chi(3) = Tchi(2,1);    chi(4) = Tchi(0,2);    chi(5) = Tchi(1,0);
        #endif

        return chi;
    }

};
