// Ransac Template Library (RTL)
#include "RTL.hpp"

// Eigen
#include <Eigen/Dense>
#include <Eigen/Core>


// Define types
typedef Eigen::Vector3d 				model_t;
typedef Eigen::Matrix<double, 5, 1>		datum_t;
typedef std::vector<datum_t>			data_t;

// Create templated class for RANSAC Doppler velocity and segmentation
class DopplerEstimator : virtual public RTL::Estimator<model_t, datum_t, data_t>
{
public:
	virtual model_t ComputeModel(const data_t & data, const std::set<int> & samples)
	{
		// Build Ax = B problem
		Eigen::MatrixXd A(samples.size(), 3);
		Eigen::MatrixXd B(samples.size(), 1);
		Eigen::DiagonalMatrix<double, Eigen::Dynamic> W(samples.size());
		int i = 0;
		for (auto itr = samples.begin(); itr != samples.end(); ++itr, ++i)
		{
			const datum_t & datum = data[*itr];
			A.row(i) << datum(0), datum(1), datum(2);
			B.row(i) << datum(3);
			W.diagonal()(i) = datum(4);
		}

		// Solve it
		model_t velocity;
		velocity = (A.transpose() * W * A).ldlt().solve(A.transpose() * W * B);

		return velocity;
	}

	virtual double ComputeError(const model_t & velocity, const datum_t & datum)
	{
		// err = Ax - B
		return double( abs(velocity.dot(datum.head(3)) - datum(3)) );
	}
};

// Function to call RANSAC algorithm and return inliers and model
void ransacDoppler(const data_t & data, model_t & velocity, std::vector<int> & inliers, const double & threshold)
{
	DopplerEstimator estimator;
	RTL::RANSAC<model_t, datum_t, data_t> ransac(&estimator);
	ransac.SetParamThreshold(threshold);	// Error threshold to consider inlier
	double loss = ransac.FindBest(velocity, data, data.size(), 6);
	inliers = ransac.FindInliers(velocity, data, data.size());
}
