// Odometry class
#include "odom.h"

// Ransac from RTL
#include "ransac_rtl.h"

// Eigen
#include <Eigen/Core>

using namespace std;
using namespace Eigen;


void Odometry::dopplerVelocityRansac(RadarData & data, Vector3d & vel, Matrix3d & covar)
{
	// Create RANSAC problem
	std::vector<Vector4d> ransac_data;
	std::vector<int> ransac_inliers;
	for (int i=0; i<data.inliers.rows(); ++i)
	{
		Vector4d datum;
		double phi = data.spher(i, 1), the = data.spher(i, 2);
		datum << sin(phi)*cos(the), cos(phi)*cos(the), sin(the), -data.doppler(i);
		ransac_data.push_back(datum);
	}
	
	// Solve
	ransacDoppler(ransac_data, vel, ransac_inliers, threshold_value);

	// Output inliers
	data.inliers.setZero();
	for (int i=0; i<ransac_inliers.size(); ++i)
		data.inliers(ransac_inliers[i]) = true;
		
	// Covariance (TODO)
	covar.setZero();
}

void Odometry::dopplerVelocityTls(RadarData & data, Vector3d & vel, Matrix3d & covar)
{
	// Parameters to tune: iterations, residual threshold, convergence threshold
	int cloud_size = data.spher.rows();

	// Create matrices
	MatrixXd A(cloud_size, 3), B(cloud_size, 1);
	ArrayXd res = ArrayXd::Zero(cloud_size);

	// Populate Ax=B
	for (int i=0; i<cloud_size; ++i)
	{
		double phi = data.spher(i, 1), the = data.spher(i, 2);
		double sp = sin(phi), cp = cos(phi), st = sin(the), ct = cos(the);
		A.row(i) << sp*ct, cp*ct, st;
		B(i, 0) = -data.doppler(i);
	}

	// IRLS
	vel.setZero();
	Vector3d vel_old=Vector3d::Zero();
	DiagonalMatrix<double, Dynamic> W(cloud_size);
	for (int i=0; i<20; ++i)
	{
		if (i > 0)
		{
			if (threshold_mode == "percent")
			{
				// Update inliers (Trimmed, Value%)
				int k = cloud_size * threshold_value;
				std::vector<double> res_sort(res.data(), res.data() + res.size());
				nth_element(res_sort.begin(), res_sort.begin()+k, res_sort.end());
				double res_th = res_sort[k];
				data.inliers = res < res_th;
				cout << "Threshold: " << res_th << endl;
			}
			else if (threshold_mode == "mad")
			{
				// Update inliers (threshold = median + Value*MAD)
				std::vector<double> res_sort(res.data(), res.data() + res.size());
				nth_element(res_sort.begin(), res_sort.begin()+res_sort.size()/2, res_sort.end());
				double res_med = res_sort[res_sort.size()/2];
				for (int j=0; j<res_sort.size(); ++j)
					res_sort[j] = abs(res_sort[j] - res_med);
				nth_element(res_sort.begin(), res_sort.begin()+res_sort.size()/2, res_sort.end());
				double mad = res_sort[res_sort.size()/2];
				data.inliers = res <= res_med + threshold_value*mad;
			}
			else
			{
				// Update inliers (Fixed threshold Value)
				data.inliers = res <= threshold_value;
			}
		}

		// Solve
		if (use_power)
			W.diagonal() = data.inliers.cast<double>() * data.power;
		else
			W.diagonal() = data.inliers.cast<double>();
			
			
		vel = (A.transpose() * W * A).ldlt().solve(A.transpose() * W * B);

		// Update residuals
		res = (A*vel-B).cwiseAbs();

		// Convergence
		if (i > 0)
		{
			double diff = (vel - vel_old).norm();
			vel_old = vel;
			if (diff < 1e-5)
				break;
		}
	}

	// Calculate covariance matrix
	double sigma = (res.matrix().transpose() * W * res.matrix()).sum() / (cloud_size - 3.);
	covar = sigma * (A.transpose() * W * A).inverse();
}
