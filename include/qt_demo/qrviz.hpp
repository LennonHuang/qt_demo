#ifndef QRVIZ_HPP
#define QRVIZ_HPP

#include <QObject>
#include <ros/ros.h>
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/tool_manager.h>
#include <rviz/tool.h>
#include <QVBoxLayout>

class qrviz
{
public:
    qrviz(QVBoxLayout *layout);
    void setFixedFrame(QString FrameName);
private:
    rviz::RenderPanel *_render_panel;
    rviz::VisualizationManager *_manger;
};

#endif // QRVIZ_HPP
