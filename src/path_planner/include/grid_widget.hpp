#ifndef GRID_WIDGET_HPP_
#define GRID_WIDGET_HPP_

#include <QWidget>
#include <QImage>
#include <QPainter>
#include <QPaintEvent>
#include <QTimer>
#include <QVector>
#include <QMouseEvent>

#include <rclcpp/rclcpp.hpp>

class GridWidget : public QWidget {
	Q_OBJECT

public:
	// Basic grid configuration and interaction API
	void setGridSize(int rows, int cols, int cell_px = 20);
	void clearGrid();
	void setObstacle(int r, int c, bool blocked);
	void toggleObstacle(int r, int c);
	void setPath(const QVector<QPoint> &cells); // path as grid coordinates (r,c)
	void clearPath();

	GridWidget(
		rclcpp::Node::SharedPtr & node_handle, QWidget * parent = nullptr,
		Qt::WindowFlags f = Qt::WindowFlags());
	~GridWidget();

protected:
	void paintEvent(QPaintEvent * event) override;
	void mousePressEvent(QMouseEvent *event) override;

private slots:
	void onUpdate();

private:
	// ROS 2 node handle for integration/spinning
	rclcpp::Node::SharedPtr nh_;

	// Grid state: 0 = free, 1 = obstacle
	int rows_ {25};
	int cols_ {25};
	int cell_px_ {24};
	QVector<unsigned char> grid_;
	QVector<QPoint> path_cells_; // list of (r,c) points composing the path

	// Helpers
	inline int idx(int r, int c) const { return r * cols_ + c; }
	QRect cellRect(int r, int c) const;
	QPoint cellAt(const QPoint &pt) const; // widget px -> (r,c)

	QTimer * timer_ {nullptr};
};

#endif  // GRID_WIDGET_HPP_
