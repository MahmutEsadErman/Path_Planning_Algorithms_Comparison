#include "grid_widget.hpp"

#include <QPointF>
#include <QPen>
#include <QBrush>

#include <algorithm>
#include <string>

#include "rclcpp/rclcpp.hpp"

#define DEFAULT_BG_R 0x22
#define DEFAULT_BG_G 0x26
#define DEFAULT_BG_B 0x2b

GridWidget::GridWidget(rclcpp::Node::SharedPtr & node_handle, QWidget * parent, Qt::WindowFlags f)
: QWidget(parent, f) {
  nh_ = node_handle;
  grid_.resize(rows_ * cols_);
  std::fill(grid_.begin(), grid_.end(), 0);

  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, &GridWidget::onUpdate);
  timer_->start(33); // ~30 FPS update
}

GridWidget::~GridWidget(){
}

void GridWidget::setGridSize(int rows, int cols, int cell_px) {
  rows_ = std::max(1, rows);
  cols_ = std::max(1, cols);
  cell_px_ = std::max(4, cell_px);
  grid_.resize(rows_ * cols_);
  std::fill(grid_.begin(), grid_.end(), 0);
  path_cells_.clear();
  updateGeometry();
  update();
}

void GridWidget::clearGrid() {
  std::fill(grid_.begin(), grid_.end(), 0);
  update();
}

void GridWidget::setObstacle(int r, int c, bool blocked) {
  if (r < 0 || c < 0 || r >= rows_ || c >= cols_) return;
  grid_[idx(r, c)] = blocked ? 1 : 0;
  update(cellRect(r, c));
}

void GridWidget::toggleObstacle(int r, int c) {
  if (r < 0 || c < 0 || r >= rows_ || c >= cols_) return;
  grid_[idx(r, c)] = grid_[idx(r, c)] ? 0 : 1;
  update(cellRect(r, c));
}

void GridWidget::setPath(const QVector<QPoint> &cells) {
  path_cells_ = cells;
  update();
}

void GridWidget::clearPath() {
  path_cells_.clear();
  update();
}

QRect GridWidget::cellRect(int r, int c) const {
  return QRect(c * cell_px_, r * cell_px_, cell_px_, cell_px_);
}

QPoint GridWidget::cellAt(const QPoint &pt) const {
  int c = pt.x() / cell_px_;
  int r = pt.y() / cell_px_;
  if (r < 0 || c < 0 || r >= rows_ || c >= cols_) return QPoint(-1, -1);
  return QPoint(r, c);
}

void GridWidget::onUpdate(){
  if (!rclcpp::ok()) {
    close();
    return;
  }

  rclcpp::spin_some(nh_);
  update();
}

void GridWidget::paintEvent(QPaintEvent * event){
  (void)event;  // NO LINT
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing, false);

  // Fill background
  QRgb bg = qRgb(DEFAULT_BG_R, DEFAULT_BG_G, DEFAULT_BG_B);
  p.fillRect(rect(), bg);

  // Grid extent in pixels
  int w = cols_ * cell_px_;
  int h = rows_ * cell_px_;

  // Draw grid background area
  p.fillRect(QRect(0, 0, w, h), QColor(40, 44, 52));

  // Draw cells: obstacles
  QBrush obstacle_brush(QColor(200, 60, 60));
  for (int r = 0; r < rows_; ++r) {
    for (int c = 0; c < cols_; ++c) {
      if (grid_[idx(r, c)]) {
        p.fillRect(cellRect(r, c), obstacle_brush);
      }
    }
  }

  // Draw grid lines
  QPen grid_pen(QColor(90, 96, 106));
  p.setPen(grid_pen);
  for (int r = 0; r <= rows_; ++r) {
    int y = r * cell_px_;
    p.drawLine(0, y, w, y);
  }
  for (int c = 0; c <= cols_; ++c) {
    int x = c * cell_px_;
    p.drawLine(x, 0, x, h);
  }

  // Draw path overlay as thick polyline through cell centers
  if (!path_cells_.isEmpty()) {
    QPen path_pen(QColor(80, 200, 120));
    path_pen.setWidth(std::max(2, cell_px_ / 3));
    p.setPen(path_pen);
    for (int i = 1; i < path_cells_.size(); ++i) {
      QPoint a = path_cells_[i-1];
      QPoint b = path_cells_[i];
      QPoint ac = QPoint(a.y() * cell_px_ + cell_px_/2, a.x() * cell_px_ + cell_px_/2); // careful: a=(r,c)
      QPoint bc = QPoint(b.y() * cell_px_ + cell_px_/2, b.x() * cell_px_ + cell_px_/2);
      p.drawLine(ac, bc);
    }
  }
}

void GridWidget::mousePressEvent(QMouseEvent *event) {
  if (event->button() == Qt::LeftButton) {
    QPoint rc = cellAt(event->pos());
    if (rc.x() >= 0) {
      toggleObstacle(rc.x(), rc.y());
    }
  }
  QWidget::mousePressEvent(event);
}