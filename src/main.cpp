// C++
#include <chrono>
#include <fstream>

// ROS
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>

// Odometry class
#include "odom.h"

// Eigen
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>


using namespace std;
using namespace Eigen;


class OdometryWorker : public Odometry
{
	public:
	
	// Constructor
	OdometryWorker() 
	{
		outfile.open("/home/andres/catkin_mro/src/doppler_odometry/results/odom.csv");
	}
  
  	//------------------------------------
	// POINT CLOUD PARSER
	//------------------------------------
	// Copy the data from the PointCloud2 message to custom array
	void msg2data(const sensor_msgs::PointCloud2::ConstPtr msg, RadarData & data)
	{
		// Resize data
		int cloud_size = msg->height * msg->width;
		data.resize(cloud_size);

		// Create iterators (all fields are type 7 = float32)
		sensor_msgs::PointCloud2ConstIterator<float> it_xyz(*msg, "x");
		sensor_msgs::PointCloud2ConstIterator<float> it_sph(*msg, "range");
		sensor_msgs::PointCloud2ConstIterator<float> it_pow(*msg, "power");
		sensor_msgs::PointCloud2ConstIterator<float> it_dop(*msg, "doppler");
		int idx = 0;

		// Copy data
		for (; it_xyz != it_xyz.end(); ++it_xyz, ++it_sph, ++it_pow, ++it_dop, ++idx)
		{
			data.points.row(idx) << double(it_xyz[0]), double(it_xyz[1]), double(it_xyz[2]);	// XYZ
			data.spher.row(idx) << double(it_sph[0]), double(it_sph[2]), double(it_sph[1]);		// RPT (from RTP)
			data.power(idx) = double(it_pow[0]);
			data.doppler(idx) = double(it_dop[0]);
		}
	}

	//------------------------------------
	// PUBLISHER
	//------------------------------------
	// Publish world - radar TF and odometry
	void publish(const std_msgs::Header & header, const Matrix4d & pose)
	{	
		static tf2_ros::TransformBroadcaster tf_broadcaster;
	
		// Create TF message
		geometry_msgs::TransformStamped tf_msg;
			// Header
		tf_msg.header = header;
		tf_msg.header.frame_id = tf_from_frame;
		tf_msg.child_frame_id = tf_to_frame;
			// Translation
		tf_msg.transform.translation.x = pose(0,3);
		tf_msg.transform.translation.y = pose(1,3);
		tf_msg.transform.translation.z = pose(2,3);
			// Rotation
		Quaterniond q(pose.block<3,3>(0,0));
		tf_msg.transform.rotation.x = q.x();
		tf_msg.transform.rotation.y = q.y();
		tf_msg.transform.rotation.z = q.z();
		tf_msg.transform.rotation.w = q.w();
		
		// Create odometry message
		auto odom_msg = nav_msgs::Odometry();
			// Set header
		odom_msg.header = header;
		odom_msg.header.frame_id = tf_from_frame;
		odom_msg.child_frame_id = tf_to_frame;
			// Set pose as above
		odom_msg.pose.pose.position.x = pose(0,3);
		odom_msg.pose.pose.position.y = pose(1,3);
		odom_msg.pose.pose.position.z = pose(2,3);
		odom_msg.pose.pose.orientation.x = q.x();
		odom_msg.pose.pose.orientation.y = q.y();
		odom_msg.pose.pose.orientation.z = q.z();
		odom_msg.pose.pose.orientation.w = q.w();

		// Publish
		tf_broadcaster.sendTransform(tf_msg);
		odom_pub_->publish(odom_msg);
	}

	//------------------------------------
	// CALLBACK
	//------------------------------------
	void callback(const sensor_msgs::PointCloud2::ConstPtr msg)
	{
		if (verbose)
		{
			cout << "Message received: " << endl;
			cout << "Frame (internal): " << number_frames << endl;
		}

		// Start measuring time
		time_points.clear();
		time_points.measure();

		// Save timestamp
		ros::Time time(msg->header.stamp);
		curr_stamp = time.toNSec();
		if (prev_stamp == 0)
			prev_stamp = curr_stamp;


		// READ POINT CLOUD DATA
		// --------------------------	
		// Update current
		msg2data(msg, data);

		// Time elapsed for inputting data
		time_points.measure();

		// Initialize with the first data
		if (number_frames == 0)
		{
			// Initialize odometry
			initialize();
		}
		// Perform odometry calculations from then on
		else
		{
			// Perform odometry
			odometryDopplerOnly();	// Doppler only 3D odometry
		}


		// PUBLISH AND WRITE
		// --------------------------
		// Publish pose
		publish(msg->header, pose);
		
		// Write pose
		Quaterniond q(pose.block<3,3>(0,0));
		outfile << msg->header.stamp.sec << "," << msg->header.stamp.nsec << ",";
		outfile << pose(0,3) << "," << pose(1,3) << "," << pose(2,3) << ",";
		outfile << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << "\n";


		// TIME STATS
		// --------------------------
		// Average execution time update
		double exec_time = time_points.total_time();
		vector<double> breakdown = time_points.breakdown();
		Map<ArrayXd> elapsed(breakdown.data(), breakdown.size());
		if (number_frames == 0)
			avg_elapsed = elapsed;
		avg_exec_time = (avg_exec_time*number_frames + exec_time)/(number_frames+1);
		avg_elapsed = (avg_elapsed*number_frames + elapsed) / (number_frames+1);
		
		// Display
		if (verbose)
		{
			// Execution time
			cout << "\nExecution time: " << exec_time;
			cout << "\nTiming breakdown: \n";
			cout << elapsed << endl;

			// Average execution time
			cout << "\nAverage execution time: " << avg_exec_time;
			cout << "\nBreakdown of average: \n" << avg_elapsed;
			cout << endl << endl;
		}


		// UPDATE FOR NEXT FRAME
		// --------------------------
		// Update frame count
	 	number_frames++;
	 	prev_stamp = curr_stamp;

	}
	
	// Variables
	string tf_from_frame = "/odom";
	string tf_to_frame = "/base_link";
	
	// Publisher
	ros::Publisher const * odom_pub_;
	
	// Writer
	std::ofstream outfile;
};


// Global variable
bool verbose = true;

// Get parameter if exist
template<typename T>
void checkGetParam(const ros::NodeHandle & n, const string & p, T & var)
{
	if (n.hasParam(p))
	{
		n.getParam(p, var);
		if (verbose)
			cout << "Read parameter: " << p << endl;
	}
}


// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------


int main(int argc, char **argv)
{
	//						Start
	//----------------------------------------------------------------------
	ros::init(argc, argv, "doppler_odometry");
	
	// Initialize Variables
	string input_topic = "/hugin_raf_1/radar_data";

	// Initialize node and odometry worker
	ros::NodeHandle n;
	OdometryWorker worker;
	
	// Get parameters into worker
	checkGetParam(n, "verbose", verbose);
	worker.verbose = verbose;
		// Radar input topic
	checkGetParam(n, "input_topic", input_topic);
		// TF publisher
	checkGetParam(n, "tf_from_frame", worker.tf_from_frame);
	checkGetParam(n, "tf_to_frame", worker.tf_to_frame);
		// Calibration
	checkGetParam(n, "sensor_to_icr_x", worker.x_diff);
			// Sensor pose
	vector<double> sensor_pose, calib_rot;
	checkGetParam(n, "sensor_pose", sensor_pose);
	if (sensor_pose.size() == 7)
	{
		worker.sensor_pos = Map<Vector3d>(sensor_pose.data());
		worker.sensor_rot = Map<Quaterniond>(sensor_pose.data()+3);
	}
	else if (verbose)
		cout << "Sensor pose not set because of wrong input size" << endl;
			// Calibration rotation
	checkGetParam(n, "calib_rot", calib_rot);
	if (calib_rot.size() == 4)
		worker.calib_rot = Map<Quaterniond>(calib_rot.data());
	else if (verbose)
		cout << "Calibration rotation not set because of wrong input size" << endl;
		// Least Squares parameters
	checkGetParam(n, "min_points", worker.min_points);
	checkGetParam(n, "use_power", worker.use_power);
	checkGetParam(n, "threshold_mode", worker.threshold_mode);
	checkGetParam(n, "threshold_value", worker.threshold_value);
	checkGetParam(n, "max_linear_vel", worker.max_linear_vel);
	checkGetParam(n, "max_ang_vel", worker.max_ang_vel);
		// Initial pose
	vector<double> init_pose;
	checkGetParam(n, "initial_pose", init_pose);
	if (init_pose.size() == 16)
		worker.pose = Eigen::Map<Matrix4d>(init_pose.data());
	else if (verbose)
		cout << "Initial pose not set because of wrong input size" << endl;
		
		
	// Set publisher
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/doppler_odom", 1000);
	worker.odom_pub_ = &odom_pub;
		
	// Subscribe to input topic and spin
	ros::Subscriber sub = n.subscribe(input_topic, 100, &OdometryWorker::callback, &worker);
	cout << "Doppler Odometry node: starting." << endl;
	ros::spin();
}
