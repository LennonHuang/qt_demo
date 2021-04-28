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
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <QImage>
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
    void sub_image();

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
    void power_update_signal(float p);
    void image_val(QImage);

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
    ros::Publisher my_cmd_publisher;
    QStringListModel logging_model;
    ros::Subscriber my_cmd_sub;
    ros::Publisher power_val_pub;
    ros::Subscriber power_val_sub;
    image_transport::Subscriber image_sub;
    void dashboard_callback(const geometry_msgs::Twist &msg);
    void power_callback(const std_msgs::Float32 &msg);
    void image_calback(const sensor_msgs::ImageConstPtr &msg);
    QImage Mat2QImage(cv::Mat const& src);
};

}  // namespace qt_demo

#endif /* qt_demo_QNODE_HPP_ */
