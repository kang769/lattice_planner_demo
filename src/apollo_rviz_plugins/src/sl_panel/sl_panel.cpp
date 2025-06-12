#include "apollo_rviz_plugins/sl_panel.h"
#include "ui_sl_panel.h"

#include <QVBoxLayout>
#include <QPainter>
#include <QPainterPath>
#include <QDateTime>

namespace apollo_rviz_plugins {

SLCanvas::SLCanvas(QWidget* parent)
    : QWidget(parent), sl_data_(nullptr) {
  // 设置背景色
  setAutoFillBackground(true);
  QPalette pal = palette();
  pal.setColor(QPalette::Background, Qt::white);
  setPalette(pal);
  
  // 设置最小尺寸
  setMinimumSize(400, 200);
}

void SLCanvas::setSLData(const apollo_planning::SLGraph::ConstPtr& msg) {
  sl_data_ = msg;
  update(); // 触发重绘
}

QPointF SLCanvas::slToCanvas(double s, double l) const {
  // 转换SL坐标到画布坐标
  return QPointF(s * scale_ + offset_x_, -l * scale_ + offset_y_);
}

void SLCanvas::paintEvent(QPaintEvent* event) {
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  
  // 绘制坐标轴
  painter.setPen(QPen(Qt::gray, 1));
  painter.drawLine(offset_x_, 0, offset_x_, height());  // L轴
  painter.drawLine(0, offset_y_, width(), offset_y_);   // S轴
  
  // 绘制坐标轴标签
  painter.drawText(width() - 30, offset_y_ + 20, "S");
  painter.drawText(offset_x_ - 20, 20, "L");
  
  // 如果没有数据，就不绘制
  if (!sl_data_) {
    painter.drawText(width() / 2 - 50, height() / 2, "等待SL图数据...");
    return;
  }
  
  // 绘制参考线
  if (!sl_data_->ref_s.empty()) {
    painter.setPen(QPen(Qt::blue, 2));
    QPainterPath ref_path;
    QPointF start_point = slToCanvas(sl_data_->ref_s[0], sl_data_->ref_l[0]);
    ref_path.moveTo(start_point);
    
    for (size_t i = 1; i < sl_data_->ref_s.size(); ++i) {
      QPointF point = slToCanvas(sl_data_->ref_s[i], sl_data_->ref_l[i]);
      ref_path.lineTo(point);
    }
    
    painter.drawPath(ref_path);
  }
  
  // 绘制边界
  if (!sl_data_->s_samples.empty()) {
    painter.setPen(QPen(Qt::red, 1, Qt::DashLine));
    
    for (size_t i = 0; i < sl_data_->s_samples.size(); ++i) {
      double s = sl_data_->s_samples[i];
      double l_lower = sl_data_->l_lower[i];
      double l_upper = sl_data_->l_upper[i];
      
      QPointF lower_point = slToCanvas(s, l_lower);
      QPointF upper_point = slToCanvas(s, l_upper);
      
      painter.drawLine(lower_point, upper_point);
    }
  }
  
  // 绘制规划轨迹
  if (!sl_data_->traj_s.empty()) {
    painter.setPen(QPen(Qt::green, 2));
    QPainterPath traj_path;
    QPointF start_point = slToCanvas(sl_data_->traj_s[0], sl_data_->traj_l[0]);
    traj_path.moveTo(start_point);
    
    for (size_t i = 1; i < sl_data_->traj_s.size(); ++i) {
      QPointF point = slToCanvas(sl_data_->traj_s[i], sl_data_->traj_l[i]);
      traj_path.lineTo(point);
    }
    
    painter.drawPath(traj_path);
  }
  
  // 绘制初始点
  painter.setPen(QPen(Qt::black, 4));
  QPointF init_point = slToCanvas(sl_data_->init_s, sl_data_->init_l);
  painter.drawPoint(init_point);
  
  // 绘制图例
  int legend_x = 10;
  int legend_y = 10;
  int legend_height = 20;
  
  painter.setPen(QPen(Qt::blue, 2));
  painter.drawLine(legend_x, legend_y, legend_x + 30, legend_y);
  painter.drawText(legend_x + 40, legend_y + 5, "参考线");
  
  legend_y += legend_height;
  painter.setPen(QPen(Qt::red, 1, Qt::DashLine));
  painter.drawLine(legend_x, legend_y, legend_x + 30, legend_y);
  painter.drawText(legend_x + 40, legend_y + 5, "边界");
  
  legend_y += legend_height;
  painter.setPen(QPen(Qt::green, 2));
  painter.drawLine(legend_x, legend_y, legend_x + 30, legend_y);
  painter.drawText(legend_x + 40, legend_y + 5, "规划轨迹");
  
  legend_y += legend_height;
  painter.setPen(QPen(Qt::black, 4));
  painter.drawPoint(legend_x + 15, legend_y);
  painter.drawText(legend_x + 40, legend_y + 5, "初始点");
}

SLPanel::SLPanel(QWidget* parent)
    : rviz::Panel(parent), ui_(new Ui::SLPanel) {
  ui_->setupUi(this);
  
  // 创建画布
  canvas_ = new SLCanvas(this);
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

SLPanel::~SLPanel() {
  delete ui_;
}

void SLPanel::onInitialize() {
  // 初始化ROS订阅
  sl_sub_ = nh_.subscribe("planning/sl_graph", 1, &SLPanel::slCallback, this);
}

void SLPanel::onEnable() {
  show();
  parentWidget()->show();
}

void SLPanel::onDisable() {
  hide();
  parentWidget()->hide();
}

void SLPanel::slCallback(const apollo_planning::SLGraph::ConstPtr& msg) {
  latest_sl_data_ = msg;
  new_data_arrived_ = true;
  
  // 更新状态标签
  QMetaObject::invokeMethod(ui_->statusLabel, "setText", 
                          Q_ARG(QString, QString("数据更新: %1").arg(
                              QDateTime::currentDateTime().toString("hh:mm:ss"))));
}

void SLPanel::updateCanvas() {
  if (new_data_arrived_ && latest_sl_data_) {
    canvas_->setSLData(latest_sl_data_);
    new_data_arrived_ = false;
  }
}

} // namespace apollo_rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(apollo_rviz_plugins::SLPanel, rviz::Panel) 