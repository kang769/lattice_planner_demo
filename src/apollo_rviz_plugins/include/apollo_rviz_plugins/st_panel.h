#ifndef APOLLO_RVIZ_PLUGINS_ST_PANEL_H
#define APOLLO_RVIZ_PLUGINS_ST_PANEL_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include <apollo_planning/STGraph.h>

#include <QWidget>
#include <QPainter>
#include <QTimer>

namespace Ui {
class STPanel;
}

namespace apollo_rviz_plugins {

class STCanvas : public QWidget {
  Q_OBJECT
public:
  STCanvas(QWidget* parent = nullptr);
  ~STCanvas() override = default;

  void setSTData(const apollo_planning::STGraph::ConstPtr& msg);

protected:
  void paintEvent(QPaintEvent* event) override;

private:
  apollo_planning::STGraph::ConstPtr st_data_;
  
  // 视图变换参数
  double scale_ = 10.0;
  double offset_x_ = 50.0;
  double offset_y_ = 250.0;
  
  // 坐标转换函数
  QPointF stToCanvas(double s, double t) const;
};

class STPanel : public rviz::Panel {
  Q_OBJECT
public:
  explicit STPanel(QWidget* parent = nullptr);
  ~STPanel() override;

  void onInitialize() override;
  void onEnable();
  void onDisable();

private Q_SLOTS:
  void updateCanvas();

private:
  void stCallback(const apollo_planning::STGraph::ConstPtr& msg);

  Ui::STPanel* ui_;
  STCanvas* canvas_;
  
  ros::NodeHandle nh_;
  ros::Subscriber st_sub_;
  
  QTimer* timer_;
  bool new_data_arrived_ = false;
  apollo_planning::STGraph::ConstPtr latest_st_data_;
};

} // namespace apollo_rviz_plugins

#endif // APOLLO_RVIZ_PLUGINS_ST_PANEL_H 