#include "../include/qt_demo/qrviz.hpp"
#include <QDebug>
qrviz::qrviz(QVBoxLayout *layout)
{
    _render_panel = new rviz::RenderPanel();
    layout->addWidget(_render_panel);
    _manger = new rviz::VisualizationManager(_render_panel);
    ROS_ASSERT(_manger !=NULL);//Ensure manger, without it in older version of ROS will crash.

    //initialize RVIZ
    _manger->initialize();
    _manger->removeAllDisplays();
    _manger->startUpdate();

    //initialize render panel,zoom in zoom out
    _render_panel->initialize(_manger->getSceneManager(),_manger);
}

void qrviz::setFixedFrame(QString FixedFrame){
    _manger->setFixedFrame(FixedFrame);
    qDebug() << "Frame Changed to " << _manger->getFixedFrame();
}
