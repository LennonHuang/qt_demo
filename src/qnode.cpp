/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/qt_demo/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qt_demo {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"qt_demo");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
    my_cmd_publisher = n.advertise<geometry_msgs::Twist>("my_cmd_vel",1000);
    my_cmd_sub = n.subscribe("my_cmd_vel",1000,&QNode::dashboard_callback,this);
    start();//Qthread function to QNode::run function
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"qt_demo");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
    my_cmd_publisher = n.advertise<geometry_msgs::Twist>("my_cmd_vel",1000);
    my_cmd_sub = n.subscribe("my_cmd_vel",1000,&QNode::dashboard_callback,this);
	start();
	return true;
}

//Call back function for topic subscriber
void QNode::dashboard_callback(const geometry_msgs::Twist &msg){
    emit dashboard_update_signal(msg.linear.x, msg.linear.y, msg.angular.z);
}


void QNode::run() {
	ros::Rate loop_rate(1);
	int count = 0;
	while ( ros::ok() ) {
        //geometry_msgs::Twist command;


		std_msgs::String msg;
		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();
		chatter_publisher.publish(msg);
		log(Info,std::string("I sent: ")+msg.data);
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::set_cmd_vel(char key, float linear, float rot){
    // Map for movement keys
    std::map<char, std::vector<float>> moveBindings
    {
      {'i', {1, 0, 0, 0}},
      {'o', {1, 0, 0, -1}},
      {'j', {0, 0, 0, 1}},
      {'l', {0, 0, 0, -1}},
      {'u', {1, 0, 0, 1}},
      {',', {-1, 0, 0, 0}},
      {'.', {-1, 0, 0, 1}},
      {'m', {-1, 0, 0, -1}},
      {'O', {1, -1, 0, 0}},
      {'I', {1, 0, 0, 0}},
      {'J', {0, 1, 0, 0}},
      {'L', {0, -1, 0, 0}},
      {'U', {1, 1, 0, 0}},
      {'<', {-1, 0, 0, 0}},
      {'>', {-1, -1, 0, 0}},
      {'M', {-1, 1, 0, 0}},
      {'t', {0, 0, 1, 0}},
      {'b', {0, 0, -1, 0}},
      {'k', {0, 0, 0, 0}},
      {'K', {0, 0, 0, 0}}
    };

    // Grab the direction data
    int x = moveBindings[key][0];
    int y = moveBindings[key][1];
    int z = moveBindings[key][2];
    int th = moveBindings[key][3];

    //Pack as a message
    geometry_msgs::Twist twist;
    twist.linear.x = x*linear;
    twist.linear.y = y*linear;
    twist.linear.z = z*linear;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = th*rot;

    //Publish the message
    my_cmd_publisher.publish(twist);
}

void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace qt_demo
