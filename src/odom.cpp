// Odometry class
#include "odom.h"

// C++
#include <iostream>


using namespace std;
using namespace Eigen;

Odometry::Odometry()
{
	// Initialize variables
	number_frames = 0;
	prev_stamp = 0.;

	// Timing variables
	avg_exec_time = 0.;
	avg_elapsed = ArrayXd::Zero(1);

	// Initialize transformations
	transform.setIdentity();
}

void Odometry::initialize()
{
	// Set Doppler function	
	if (threshold_mode == "ransac")
		dopplerVelocity = std::bind(&Odometry::dopplerVelocityRansac, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	else
		dopplerVelocity = std::bind(&Odometry::dopplerVelocityTls, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

	// Doppler velocity and inliers
	Vector3d vel;
	dopplerVelocity(data, vel, dopp_cov);

	// Time measure
	time_points.measure();
	avg_exec_time = time_points.total_time();

	if (verbose)
		cout << "Initialization successful." << endl;
}

void Odometry::odometryDopplerOnly()
{
	if (verbose)
		cout << "\nDOPPLER ODOMETRY: " << endl;

	// DOPPLER
	// --------------------------
	// Obtain Doppler velocity and segmentation
	Vector3d vel = Vector3d::Zero();
		// Ensure there are points available
	if (data.inliers.size() > min_points)
	{
		dopplerVelocity(data, vel, dopp_cov);
	}
	else
	{
		if (verbose)
			cout << "Not enough points, dropping this frame." << endl;
		return;
	}

	if (verbose)
	{
		cout << "Inliers: " << data.inliers.count() << " / " << data.inliers.rows()
		 	 << " (" << 100.*double(data.inliers.count())/double(data.inliers.rows()) << "%)" << endl;
		cout << "Sensor velocity: " << vel.transpose() << endl;
		cout << "Covariance of velocity: " << endl << dopp_cov << endl;
		cout << endl;
	}

	// Time elapsed for Doppler velocity and segmentation
    time_points.measure();
	


	// ANGULAR VELOCITIES
	// --------------------------
	// Transform frame of reference (set to base_link)
	Vector3d sensor_vel = calib_rot._transformVector(sensor_rot._transformVector(vel));
	if (verbose)
		cout << "Sensor velocity (rotated and calibrated calibration): " << sensor_vel.transpose() << endl;

	// Calculate angular velocities
	double w_z = -sensor_vel(1) / x_diff;
	double w_y = sensor_vel(2) / x_diff;
	ang_vel << 0., w_y, w_z;
	if (verbose)
		cout << "Angular velocities: " << rad2deg(w_y) << "º, "
									<< rad2deg(w_z) << "º" << endl << endl;
									
	// Calculate their covariance matrix
	Matrix3d Jac = Matrix3d::Zero();
	Jac << 0., 0., 0.,
			0., 0., 1./x_diff,
			0., -1./x_diff, 0.;
			
	ang_cov = Jac * dopp_cov * Jac.transpose();
	
	if (verbose)
		cout << "Angular velocity covariance: " << endl << ang_cov << endl;

	// Time elapsed for angular velocity estimation
	time_points.measure();

	// UPDATE POSE
	// --------------------------
	// Update global velocity
	dopp_vel = vel;

	// Obtain rotation
	double time_diff = double(curr_stamp - prev_stamp) * 1e-9;
	double angle = ang_vel.norm() * time_diff;
	
	// Velocity of base
	Vector3d base_vel = sensor_vel - ang_vel.cross(sensor_pos);
	
	if (verbose)
		cout << "Velocity of the base: " << base_vel.transpose() << endl;

	// Obtain translation
	Vector3d trans = base_vel * time_diff;
	
	if (verbose)
		cout << "Translation: " << trans.transpose() << endl;

	// Create transform
	transform.setIdentity();
	transform.block<3,1>(0,3) = trans;
	transform.block<3,3>(0,0) = AngleAxisd(angle, ang_vel.normalized()).toRotationMatrix();

	// Update pose
	pose *= transform;

	// Time elapsed for pose update
	time_points.measure();
}
