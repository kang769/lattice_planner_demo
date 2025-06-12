#ifndef APOLLO_RVIZ_PLUGINS_SL_PANEL_H
#define APOLLO_RVIZ_PLUGINS_SL_PANEL_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include <apollo_planning/SLGraph.h>

#include <QWidget>
#include <QPainter>
#include <QTimer>

namespace Ui {
class SLPanel;
}

namespace apollo_rviz_plugins {

class SLCanvas : public QWidget {
  Q_OBJECT
public:
  SLCanvas(QWidget* parent = nullptr);
  ~SLCanvas() override = default;

  void setSLData(const apollo_planning::SLGraph::ConstPtr& msg);

protected:
  void paintEvent(QPaintEvent* event) override;

private:
  apollo_planning::SLGraph::ConstPtr sl_data_;
  
  // 视图变换参数
  double scale_ = 10.0;
  double offset_x_ = 50.0;
  double offset_y_ = 200.0;
  
  // 坐标转换函数
  QPointF slToCanvas(double s, double l) const;
};

class SLPanel : public rviz::Panel {
  Q_OBJECT
public:
  explicit SLPanel(QWidget* parent = nullptr);
  ~SLPanel() override;

  void onInitialize() override;
  void onEnable();
  void onDisable();

private Q_SLOTS:
  void updateCanvas();

private:
  void slCallback(const apollo_planning::SLGraph::ConstPtr& msg);

  Ui::SLPanel* ui_;
  SLCanvas* canvas_;
  
  ros::NodeHandle nh_;
  ros::Subscriber sl_sub_;
  
  QTimer* timer_;
  bool new_data_arrived_ = false;
  apollo_planning::SLGraph::ConstPtr latest_sl_data_;
};

} // namespace apollo_rviz_plugins

#endif // APOLLO_RVIZ_PLUGINS_SL_PANEL_H 