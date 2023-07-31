// C++
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <nav_msgs/msg/odometry.hpp>

using namespace std;

// ------------------------------------------------------
//						CLASS
// ------------------------------------------------------
class EvaluationNode : public rclcpp::Node
{
  public:
	// Constructor
    EvaluationNode()
    : Node("radar_odom_node")
	{
		// Declare parameters
		this->declare_parameter("odom_topic", "/radar/odom");
		this->declare_parameter("gt_topic", "");
		this->declare_parameter("odom_file", "");
		this->declare_parameter("gt_file", "");

		// Get parameters
		odom_topic = this->get_parameter("odom_topic").get_parameter_value().get<string>();
		gt_topic = this->get_parameter("gt_topic").get_parameter_value().get<string>();
		odom_filename = this->get_parameter("odom_file").get_parameter_value().get<string>();
		gt_filename = this->get_parameter("gt_file").get_parameter_value().get<string>();

		// Create subscription to odometry messages
		std::function<void(std::shared_ptr<nav_msgs::msg::Odometry>)> fnc = std::bind(&EvaluationNode::callback, this, placeholders::_1, "odom");
		odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic, 10, fnc);
		if (gt_topic.size() > 1)
		{
			fnc = std::bind(&EvaluationNode::callback, this, placeholders::_1, "gt");
			gt_sub = this->create_subscription<nav_msgs::msg::Odometry>(gt_topic, 10, fnc);
		}

		// Open results file
		if (odom_filename.size() > 5)
		{
			RCLCPP_INFO(this->get_logger(), "Creating odom file in: " + odom_filename);
			odom_file.open(odom_filename);
			odom_file << "#time \ttx \tty \ttz \tqx \tqy \tqz \tqw\n";
		}
		if (gt_filename.size() > 5)
		{
			RCLCPP_INFO(this->get_logger(), "Creating gt file in: " + gt_filename);
			gt_file.open(gt_filename);
			gt_file << "#time \ttx \tty \ttz \tqx \tqy \tqz \tqw\n";
		}
    }

  private:
	// Write in results file
	void write_results(const std_msgs::msg::Header & header, const vector<double> & pose, ofstream & file)
	{
		// Create timestamp string
		rclcpp::Time timestamp(header.stamp);
		char timestr[20];
		snprintf(timestr, sizeof(timestr), "%.9f", timestamp.seconds());

		// Write to file
		file << timestr << " ";
		for (long unsigned int i=0; i<pose.size(); ++i)
			file << " " << pose[i];
		file << endl;
	}

	//------------------------------------
	// CALLBACK
	//------------------------------------
    void callback(const nav_msgs::msg::Odometry::SharedPtr msg, const string & topic)
    {
		RCLCPP_INFO(this->get_logger(), topic + " message received");

		// Get pose from message structure
		vector<double> pose(7);
		pose[0] = msg->pose.pose.position.x;
		pose[1] = msg->pose.pose.position.y;
		pose[2] = msg->pose.pose.position.z;
		pose[3] = msg->pose.pose.orientation.x;
		pose[4] = msg->pose.pose.orientation.y;
		pose[5] = msg->pose.pose.orientation.z;
		pose[6] = msg->pose.pose.orientation.w;

		// Write if file is open
		if (topic == "odom")
			if (odom_file.is_open())
				write_results(msg->header, pose, odom_file);
		if (topic == "gt")
			if (gt_file.is_open())
				write_results(msg->header, pose, gt_file);
    }

	//------------------------------------
	// VARIABLES
	//------------------------------------
	// Input parameters
	string odom_topic, gt_topic;
	string odom_filename, gt_filename;

	// Declare subscriptions
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gt_sub;

	// Output results file
	ofstream odom_file;
	ofstream gt_file;
};


int main(int num_arg, char *argv[])
{
	//						Start
	//----------------------------------------------------------------------
	try
	{
		// Start ROS
		//----------------------------------------------------------------------
		cout << "Evaluation node: READY." << endl;
	    rclcpp::init(num_arg, argv);
		rclcpp::spin(std::make_shared<EvaluationNode>());
		rclcpp::shutdown();

		cout << "Evaluation node: SHUTTING DOWN" << endl;

		return 0;

	}
	catch (std::exception &e)
	{
		cout << "Exception caught: " << e.what() << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Untyped exception!!");
		return -1;
	}
}
