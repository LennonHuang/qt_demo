/**
 * @file /include/qt_demo/main_window.hpp
 *
 * @brief Qt based gui for qt_demo.
 *
 * @date November 2010
 **/
#ifndef qt_demo_MAIN_WINDOW_H
#define qt_demo_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtWidgets/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "DashBoard.hpp"
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QString>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace qt_demo {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

    QSerialPort *my_serialPort;

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically
    void slot_linear_value_changed(int);//slot for linear speed slider value changed
    void slot_rot_value_changed(int);//slot for rotation speed slider value changed

    void slot_key_clicked();
    void slot_dashboard_update(float x,float y,float theta);
    void slot_power_update(float power);
    void slot_ros_bag_btn_clicked();
    void slot_Sim_btn_clicked();

    void slot_update_camera(QImage);
    void slot_start_cam();
    void slot_scan_port_btn();

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
    DashBoard *lin_dashboard;
    DashBoard *rot_dashboard;
    QStringList scanPort();
};

}  // namespace qt_demo

#endif // qt_demo_MAIN_WINDOW_H
