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
    void Display_Grid(int cell_count, QColor, bool enable);
    void Display_tf(bool);
    void Display_scan(QString scan_topics,bool enable);
private:
    rviz::RenderPanel *_render_panel;
    rviz::VisualizationManager *_manger;
    rviz::Display *_grid = nullptr;
    rviz::Display *_tf_display = nullptr;
    rviz::Display *_scan_display = nullptr;
};

#endif // QRVIZ_HPP
