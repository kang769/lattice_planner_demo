#include "apollo_rviz_plugins/st_panel.h"
#include "ui_st_panel.h"

#include <QVBoxLayout>
#include <QPainter>
#include <QPainterPath>
#include <QDateTime>

namespace apollo_rviz_plugins {

STCanvas::STCanvas(QWidget* parent)
    : QWidget(parent), st_data_(nullptr) {
  // 设置背景色
  setAutoFillBackground(true);
  QPalette pal = palette();
  pal.setColor(QPalette::Background, Qt::white);
  setPalette(pal);
  
  // 设置最小尺寸
  setMinimumSize(400, 200);
}

void STCanvas::setSTData(const apollo_planning::STGraph::ConstPtr& msg) {
  st_data_ = msg;
  update(); // 触发重绘
}

QPointF STCanvas::stToCanvas(double s, double t) const {
  // 转换ST坐标到画布坐标
  return QPointF(s * scale_ + offset_x_, -t * scale_ + offset_y_);
}

void STCanvas::paintEvent(QPaintEvent* event) {
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  
  // 绘制坐标轴
  painter.setPen(QPen(Qt::gray, 1));
  painter.drawLine(offset_x_, 0, offset_x_, height());  // T轴
  painter.drawLine(0, offset_y_, width(), offset_y_);   // S轴
  
  // 绘制坐标轴标签
  painter.drawText(width() - 30, offset_y_ + 20, "S");
  painter.drawText(offset_x_ - 20, 20, "T");
  
  // 如果没有数据，就不绘制
  if (!st_data_) {
    painter.drawText(width() / 2 - 50, height() / 2, "等待ST图数据...");
    return;
  }
  
  // 绘制ST图的范围边界
  painter.setPen(QPen(Qt::gray, 1, Qt::DashLine));
  QPointF s_min_t_min = stToCanvas(st_data_->s_min, st_data_->t_min);
  QPointF s_max_t_min = stToCanvas(st_data_->s_max, st_data_->t_min);
  QPointF s_min_t_max = stToCanvas(st_data_->s_min, st_data_->t_max);
  QPointF s_max_t_max = stToCanvas(st_data_->s_max, st_data_->t_max);
  
  painter.drawLine(s_min_t_min, s_max_t_min);  // 下边界
  painter.drawLine(s_min_t_min, s_min_t_max);  // 左边界
  painter.drawLine(s_max_t_min, s_max_t_max);  // 右边界
  painter.drawLine(s_min_t_max, s_max_t_max);  // 上边界
  
  // 绘制障碍物ST边界
  if (!st_data_->st_boundaries_s.empty()) {
    painter.setPen(QPen(Qt::red, 2));
    
    // 每个障碍物有四个点，形成一个多边形
    size_t num_obstacles = st_data_->st_boundaries_s.size() / 4;
    for (size_t i = 0; i < num_obstacles; ++i) {
      size_t base_idx = i * 4;
      
      QPointF upper_left = stToCanvas(st_data_->st_boundaries_s[base_idx], 
                                     st_data_->st_boundaries_t[base_idx]);
      QPointF upper_right = stToCanvas(st_data_->st_boundaries_s[base_idx + 1], 
                                      st_data_->st_boundaries_t[base_idx + 1]);
      QPointF lower_left = stToCanvas(st_data_->st_boundaries_s[base_idx + 2], 
                                     st_data_->st_boundaries_t[base_idx + 2]);
      QPointF lower_right = stToCanvas(st_data_->st_boundaries_s[base_idx + 3], 
                                      st_data_->st_boundaries_t[base_idx + 3]);
      
      // 绘制障碍物边界线
      painter.drawLine(upper_left, upper_right);
      painter.drawLine(upper_right, lower_right);
      painter.drawLine(lower_right, lower_left);
      painter.drawLine(lower_left, upper_left);
      
      // 绘制障碍物ID
      if (i < st_data_->obstacle_ids.size()) {
        QPointF text_pos = (upper_left + upper_right + lower_left + lower_right) / 4.0;
        painter.drawText(text_pos, QString::fromStdString(st_data_->obstacle_ids[i]));
      }
    }
  }
  
  // 绘制规划轨迹
  if (!st_data_->traj_s.empty()) {
    painter.setPen(QPen(Qt::green, 2));
    QPainterPath traj_path;
    QPointF start_point = stToCanvas(st_data_->traj_s[0], st_data_->traj_t[0]);
    traj_path.moveTo(start_point);
    
    for (size_t i = 1; i < st_data_->traj_s.size(); ++i) {
      QPointF point = stToCanvas(st_data_->traj_s[i], st_data_->traj_t[i]);
      traj_path.lineTo(point);
    }
    
    painter.drawPath(traj_path);
  }
  
  // 绘制初始点
  painter.setPen(QPen(Qt::black, 4));
  QPointF init_point = stToCanvas(st_data_->init_s, 0.0);
  painter.drawPoint(init_point);
  
  // 绘制图例
  int legend_x = 10;
  int legend_y = 10;
  int legend_height = 20;
  
  painter.setPen(QPen(Qt::red, 2));
  painter.drawLine(legend_x, legend_y, legend_x + 30, legend_y);
  painter.drawText(legend_x + 40, legend_y + 5, "障碍物边界");
  
  legend_y += legend_height;
  painter.setPen(QPen(Qt::green, 2));
  painter.drawLine(legend_x, legend_y, legend_x + 30, legend_y);
  painter.drawText(legend_x + 40, legend_y + 5, "规划轨迹");
  
  legend_y += legend_height;
  painter.setPen(QPen(Qt::black, 4));
  painter.drawPoint(legend_x + 15, legend_y);
  painter.drawText(legend_x + 40, legend_y + 5, "初始点");
}

STPanel::STPanel(QWidget* parent)
    : rviz::Panel(parent), ui_(new Ui::STPanel) {
  ui_->setupUi(this);
  
  // 创建画布
  canvas_ = new STCanvas(this);
  ui_->canvasLayout->addWidget(canvas_);
  
  // 创建定时器，用于更新画布
  timer_ = new QTimer(this);
  connect(timer_, SIGNAL(timeout()), this, SLOT(updateCanvas()));
  timer_->start(100); // 10Hz
  
  // 连接重置按钮
  connect(ui_->resetButton, &QPushButton::clicked, [this]() {
    // 重置视图逻辑可以在这里添加
    ui_->statusLabel->setText("视图已重置");
  });
}

STPanel::~STPanel() {
  delete ui_;
}

void STPanel::onInitialize() {
  // 初始化ROS订阅
  st_sub_ = nh_.subscribe("planning/st_graph", 1, &STPanel::stCallback, this);
}

void STPanel::onEnable() {
  show();
  parentWidget()->show();
}

void STPanel::onDisable() {
  hide();
  parentWidget()->hide();
}

void STPanel::stCallback(const apollo_planning::STGraph::ConstPtr& msg) {
  latest_st_data_ = msg;
  new_data_arrived_ = true;
  
  // 更新状态标签
  QMetaObject::invokeMethod(ui_->statusLabel, "setText", 
                          Q_ARG(QString, QString("数据更新: %1").arg(
                              QDateTime::currentDateTime().toString("hh:mm:ss"))));
}

void STPanel::updateCanvas() {
  if (new_data_arrived_ && latest_st_data_) {
    canvas_->setSTData(latest_st_data_);
    new_data_arrived_ = false;
  }
}

} // namespace apollo_rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(apollo_rviz_plugins::STPanel, rviz::Panel) 