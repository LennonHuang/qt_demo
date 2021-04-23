/**
 * @file /include/qt_demo/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef qt_demo_QNODE_HPP_
#define qt_demo_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qt_demo {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();
    void set_cmd_vel(char key, float linear, float rot);

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();
    void dashboard_update_signal(float x,float y,float theta);

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
    ros::Publisher my_cmd_publisher;
    QStringListModel logging_model;
    ros::Subscriber my_cmd_sub;
    void dashboard_callback(const geometry_msgs::Twist &msg);
};

}  // namespace qt_demo

#endif /* qt_demo_QNODE_HPP_ */
