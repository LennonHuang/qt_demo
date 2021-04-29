/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/qt_demo/main_window.hpp"
#include <cmath>
#include <QProcess>
#include <QMessageBox>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QStringList>
#include <QComboBox>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qt_demo {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc,argv)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
    setWindowIcon(QIcon(":/images/icon.png"));
    ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

    /*********************
    ** Logging
    **********************/
    ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    /*********************
    ** Auto Start
    **********************/
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }

    //Connect the speed slider value slot and signal
    connect(ui.horizontalSlider_linear,SIGNAL(valueChanged(int)),this,SLOT(slot_linear_value_changed(int)));
    connect(ui.horizontalSlider_rot,SIGNAL(valueChanged(int)),this,SLOT(slot_rot_value_changed(int)));

    //Connect the key event
    connect(ui.pushButton_i, SIGNAL(clicked()),this, SLOT(slot_key_clicked()));
    connect(ui.pushButton_j, SIGNAL(clicked()),this, SLOT(slot_key_clicked()));
    connect(ui.pushButton_l, SIGNAL(clicked()),this, SLOT(slot_key_clicked()));
    connect(ui.pushButton_m, SIGNAL(clicked()),this, SLOT(slot_key_clicked()));
    connect(ui.pushButton_14, SIGNAL(clicked()),this, SLOT(slot_key_clicked()));
    connect(ui.pushButton_o, SIGNAL(clicked()),this, SLOT(slot_key_clicked()));
    connect(ui.pushButton_u, SIGNAL(clicked()),this, SLOT(slot_key_clicked()));
    connect(ui.pushButton_13, SIGNAL(clicked()),this, SLOT(slot_key_clicked()));
    connect(ui.pushButton_space, SIGNAL(clicked()),this, SLOT(slot_key_clicked()));
    connect(ui.ros_bag_btn, SIGNAL(clicked()),this,SLOT(slot_ros_bag_btn_clicked()));
    connect(ui.Sim_btn, SIGNAL(clicked()),this,SLOT(slot_Sim_btn_clicked()));
    connect(ui.scan_port_btn, SIGNAL(clicked()),this,SLOT(slot_scan_port_btn()));


    //initialize speed dashboard
    lin_dashboard = new DashBoard(ui.widget_linear_speed);
    rot_dashboard = new DashBoard(ui.widget_rot_speed);

    //init the tree Widget
    ui.treeWidget->setEnabled(false);
    //ui.treeWidget->setColumnCount(2);
    //Global Options
    QTreeWidgetItem *global_option = new QTreeWidgetItem(QStringList() << "Global Options");
    ui.treeWidget->setHeaderHidden(true);
    ui.treeWidget->addTopLevelItem(global_option);
    global_option->setExpanded(true);
    //Fixed Frame
    QTreeWidgetItem *fixed_frame = new QTreeWidgetItem(QStringList() << "Fixed Frame");
    fixed_box = new QComboBox();
    fixed_box->addItems(QStringList() << "map" << "odom");
    fixed_box->setMaximumWidth(100);
    fixed_box->setEditable(true);
    global_option->addChild(fixed_frame);
    ui.treeWidget->setItemWidget(fixed_frame, 1 , fixed_box);
    connect(fixed_box,SIGNAL(currentTextChanged(QString)),this,SLOT(slot_fixed_frame_changed(QString)));

    //init grid
    QTreeWidgetItem *grid = new QTreeWidgetItem(QStringList() << "Grid");
    QCheckBox *grid_checkbox = new QCheckBox();
    ui.treeWidget->addTopLevelItem(grid);
    ui.treeWidget->setItemWidget(grid,1,grid_checkbox);
    grid->setExpanded(true);
    //add grid cell number
    QTreeWidgetItem *cell_count = new QTreeWidgetItem(QStringList() << "Plane Cell");
    grid->addChild(cell_count);
    cell_count_box = new QSpinBox;
    cell_count_box->setValue(14);
    ui.treeWidget->setItemWidget(cell_count,1,cell_count_box);
    //add color
    QTreeWidgetItem *grid_color = new QTreeWidgetItem(QStringList() << "Color");
    grid->addChild(grid_color);
    grid_color_box = new QComboBox();
    grid_color_box->addItem("160;160;160");//RGB
    grid_color_box->setEditable(true);
    ui.treeWidget->setItemWidget(grid_color,1,grid_color_box);
    connect(grid_checkbox,SIGNAL(stateChanged(int)),this,SLOT(slot_display_grid(int)));

    //init tf GUI
    QTreeWidgetItem *tf_gui = new QTreeWidgetItem(QStringList() << "tf");
    QCheckBox *tf_checkbox = new QCheckBox();
    ui.treeWidget->addTopLevelItem(tf_gui);
    ui.treeWidget->setItemWidget(tf_gui,1,tf_checkbox);
    connect(tf_checkbox,SIGNAL(stateChanged(int)),this,SLOT(slot_display_tf(int)));

    //init Laser scan GUI
    QTreeWidgetItem *scan_gui = new QTreeWidgetItem(QStringList() <<"Laser Scan");
    QCheckBox *scan_checkbox = new QCheckBox();
    ui.treeWidget->addTopLevelItem(scan_gui);
    ui.treeWidget->setItemWidget(scan_gui,1,scan_checkbox);
    QTreeWidgetItem *scan_topics = new QTreeWidgetItem(QStringList() << "Laser Topics");
    scan_gui->addChild(scan_topics);
    scan_topics_box = new QComboBox();
    scan_topics_box->setEditable(true);
    scan_topics_box->addItem("/scan");
    ui.treeWidget->setItemWidget(scan_topics,1,scan_topics_box);
    scan_gui->setExpanded(true);
    connect(scan_checkbox,SIGNAL(stateChanged(int)),this,SLOT(slot_display_scan(int)));

    lin_dashboard->setGeometry(ui.widget_linear_speed->rect());
    rot_dashboard->setGeometry(ui.widget_rot_speed->rect());
    lin_dashboard->set_speed(0);
    rot_dashboard->set_speed(0);
    //Connect the signal and slot of Dashboard.
    connect(&qnode,SIGNAL(dashboard_update_signal(float,float,float)),this,SLOT(slot_dashboard_update(float,float,float)));
    //Connect the power signal and slot
    connect(&qnode, SIGNAL(power_update_signal(float)),this,SLOT(slot_power_update(float)));
    //Connect the camera image signal and slot
    connect(&qnode, SIGNAL(image_val(QImage)),this,SLOT(slot_update_camera(QImage)));
    connect(ui.cam_btn,SIGNAL(clicked()),this,SLOT(slot_start_cam()));
}

//slot for scan display
void MainWindow::slot_display_scan(int state){
    bool enable = state>1?true:false;
    my_rviz->Display_scan(scan_topics_box->currentText(),enable);
}

//slot for tf display
void MainWindow::slot_display_tf(int state){
    bool enable = state>1?true:false;
    my_rviz->Display_tf(enable);
}

//slot for grid display
void MainWindow::slot_display_grid(int state){
    qDebug() << "Displaying Grid";
    bool enable = state>1?true:false;
    QStringList qli = grid_color_box->currentText().split(";");
    QColor color = QColor(qli[0].toInt(),qli[1].toInt(),qli[2].toInt());
    qDebug() << enable;
    my_rviz->Display_Grid(cell_count_box->text().toInt(), color, enable);
}

//slot for fixed frame changed
void MainWindow::slot_fixed_frame_changed(QString){
    qDebug() << "Changing frame";
    my_rviz->setFixedFrame(fixed_box->currentText());
}

//slot for the serial port
void MainWindow::slot_scan_port_btn(){
    ui.port_box->clear();
    qDebug() << "adding port";
    QStringList qls = scanPort();
    ui.port_box->addItems(qls);
}

QStringList MainWindow::scanPort(){
    QStringList scan_result;
    qDebug() << "Scanning POrt";
    foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts()) {
        scan_result.append(info.portName());
    }
    return scan_result;
}

//slot for the camera
void MainWindow::slot_update_camera(QImage qim){
    ui.label_cam->setPixmap(QPixmap::fromImage(qim));
}
void MainWindow::slot_start_cam(){
    qDebug() << "Sending q-image";
    qnode.sub_image();

}
//slot for starting the simulation
void MainWindow::slot_Sim_btn_clicked(){
    qDebug() << "Starting Simulation Environment";
    QMessageBox msg_box;
    msg_box.setText("Please wait for Simulation Environment ready...");
    msg_box.exec();
    QProcess *gazebo_sim = new QProcess;
    gazebo_sim->start("bash");
    gazebo_sim->write("roslaunch noob_robot_gazebo noob_robot_gazebo.launch \n");
    gazebo_sim->waitForStarted();
}
//slot for starting ros_bag rqt
void MainWindow::slot_ros_bag_btn_clicked(){
    qDebug() << "Starting Record Bag";
    QMessageBox msg_box;
    msg_box.setText("Please wait for the bag...");
    msg_box.exec();
    QProcess *rqt_bag = new QProcess;
    rqt_bag->start("bash");
    rqt_bag->write("rosrun rqt_bag rqt_bag \n");
    rqt_bag->waitForStarted();
}

//slot for dashboard update
void MainWindow::slot_dashboard_update(float x, float y, float theta){
    lin_dashboard->set_speed(100*std::sqrt(x*x + y*y));
    rot_dashboard->set_speed(100*theta);
}
//slot for power update
void MainWindow::slot_power_update(float power){
    ui.progressBar->setValue((int)power);
}

//slot for linear speed slider value changed
void MainWindow::slot_linear_value_changed(int value){
    ui.label_linear->setText(QString::number(value));
}
//slot for rotation speed slider value changed
void MainWindow::slot_rot_value_changed(int value){
    ui.label_rot->setText(QString::number(value));
}
//slot for key pressed event
void MainWindow::slot_key_clicked(){
    QPushButton *btn = qobject_cast<QPushButton*> (sender());
    char key = btn->text().toStdString()[0];
    //qDebug() << key;
    bool is_omni = ui.checkbox_omni->isChecked();
    float linear_speed = ui.label_linear->text().toFloat()*0.01;
    float rot_speed = ui.label_rot->text().toFloat()*0.01;

    //Node public func to publish the message.
    switch(key){
    case 'i':
        qnode.set_cmd_vel(is_omni? 'I':'i', linear_speed, rot_speed);
        break;
    case 'u':
        qnode.set_cmd_vel(is_omni? 'U':'u', linear_speed, rot_speed);
        break;
    case 'o':
        qnode.set_cmd_vel(is_omni? 'O':'o', linear_speed, rot_speed);
        break;
    case 'j':
        qnode.set_cmd_vel(is_omni? 'J':'j', linear_speed, rot_speed);
        break;
    case 'l':
        qnode.set_cmd_vel(is_omni? 'L':'l', linear_speed, rot_speed);
        break;
    case 'm':
        qnode.set_cmd_vel(is_omni? 'M':'m', linear_speed, rot_speed);
        break;
    case ',':
        qnode.set_cmd_vel(is_omni? '<':',', linear_speed, rot_speed);
        break;
    case '.':
        qnode.set_cmd_vel(is_omni? '>':'.', linear_speed, rot_speed);
        break;
    case 's':
        qnode.set_cmd_vel(is_omni? '>':'.', 0, 0);
        break;

    }

}


MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
    QMessageBox msgBox;
    msgBox.setText("Couldn't find the ros master.");
    msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check ) {
    if ( ui.checkbox_use_environment->isChecked() ) {
        if ( !qnode.init() ) {
            showNoMasterMessage();
        } else {
            ui.treeWidget->setEnabled(true);
            ui.button_connect->setEnabled(false);
            my_rviz = new qrviz(ui.Layout_rviz);//Start RVIZ
        }
    } else {
        if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
                          ui.line_edit_host->text().toStdString()) ) {
            showNoMasterMessage();
        } else {
            ui.treeWidget->setEnabled(true);
            ui.button_connect->setEnabled(false);
            ui.line_edit_master->setReadOnly(true);
            ui.line_edit_host->setReadOnly(true);
            ui.line_edit_topic->setReadOnly(true);
            my_rviz = new qrviz(ui.Layout_rviz);//Start RVIZ
        }
    }
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
    bool enabled;
    if ( state == 0 ) {
        enabled = true;
    } else {
        enabled = false;
    }
    ui.line_edit_master->setEnabled(enabled);
    ui.line_edit_host->setEnabled(enabled);
    //ui.line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
    ui.view_logging->scrollToBottom();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "qt_demo");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
        ui.line_edit_master->setEnabled(false);
        ui.line_edit_host->setEnabled(false);
        //ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "qt_demo");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
    WriteSettings();
    QMainWindow::closeEvent(event);
}

}  // namespace qt_demo

