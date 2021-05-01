#include "../include/qt_demo/qrviz.hpp"
#include <QDebug>
qrviz::qrviz(QVBoxLayout *layout)
{
    _render_panel = new rviz::RenderPanel();
    layout->addWidget(_render_panel);
    _manger = new rviz::VisualizationManager(_render_panel);
    ROS_ASSERT(_manger !=NULL);//Ensure manger, without it in older version of ROS will crash.
    //initialize render panel,zoom in zoom out. PUt it in front of RVIZ initialization
    _render_panel->initialize(_manger->getSceneManager(),_manger);
    //initialize RVIZ
    _manger->initialize();
    _manger->removeAllDisplays();
    _manger->startUpdate();
}

void qrviz::setFixedFrame(QString FixedFrame){
    _manger->setFixedFrame(FixedFrame);
    qDebug() << "Frame Changed to " << _manger->getFixedFrame();
}

void qrviz::Display_Grid(int cell_count, QColor color, bool enable){
    if(_grid != nullptr){
        delete _grid;
        _grid = nullptr;
    }
    _grid = _manger->createDisplay("rviz/Grid","my_grid",enable);
    _grid->subProp("Plane Cell Count")->setValue(cell_count);
    _grid->subProp("Color")->setValue(color);
    ROS_ASSERT(_grid != NULL);
}

void qrviz::Display_tf(double marker_scale,bool enable){
    if(_tf_display != nullptr){
        delete  _tf_display;
        _tf_display = nullptr;
    }
    _tf_display = _manger->createDisplay("rviz/TF","my_tf",enable);
    _tf_display->subProp("Marker Scale")->setValue(marker_scale);
    ROS_ASSERT(_tf_display != NULL);
}

void qrviz::Display_scan(QString scan_topics, bool enable){
    if(_scan_display != nullptr){
        delete _scan_display;
        _scan_display = nullptr;
    }
    _scan_display = _manger->createDisplay("rviz/LaserScan","my_Laser_scan",enable);
    _scan_display->subProp("Topic")->setValue(scan_topics);
    ROS_ASSERT(_scan_display != NULL);
}

void qrviz::Display_model(bool enable){
    if(_model_display != nullptr){
        delete  _model_display;
        _model_display = nullptr;
    }
    _model_display = _manger->createDisplay("rviz/RobotModel","my_model",enable);
    ROS_ASSERT(_model_display != NULL);
}
