#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>

#include <QApplication>
#include <QMainWindow>
#include <QWidget>
#include <QPainter>
#include <QTimer>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QCheckBox>
#include <QPushButton>
#include <QFrame>
#include <QFont>
#include <QFontMetrics>
#include <QFileDialog>
#include <QMessageBox>
#include <QDateTime>

#include <memory>
#include <vector>
#include <cmath>
#include <mutex>
#include <algorithm>

// Custom widget for plotting paths
class PlotWidget : public QWidget
{
    Q_OBJECT

public:
    struct Point2D {
        double x;
        double y;
    };

    PlotWidget(QWidget* parent = nullptr) : QWidget(parent)
    {
        setMinimumSize(800, 600);
        setStyleSheet("background-color: white;");
    }

    void addRealPoint(double x, double y)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        real_path_.push_back({x, y});
        updateBounds(x, y);
    }

    void addTargetPoint(double x, double y)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        target_path_.push_back({x, y});
        updateBounds(x, y);
    }

    void addReturnPoint(double x, double y)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return_path_.push_back({x, y});
        updateBounds(x, y);
    }

    void clearPaths()
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        real_path_.clear();
        target_path_.clear();
        return_path_.clear();
        min_x_ = 0; max_x_ = 1;
        min_y_ = 0; max_y_ = 1;
        bounds_initialized_ = false;
    }

    void setShowRealPath(bool show) { show_real_path_ = show; update(); }
    void setShowTargetPath(bool show) { show_target_path_ = show; update(); }
    void setShowReturnPath(bool show) { show_return_path_ = show; update(); }

    bool savePlot(const QString& filename)
    {
        QPixmap pixmap(size());
        render(&pixmap);
        return pixmap.save(filename);
    }

protected:
    void paintEvent(QPaintEvent* /*event*/) override
    {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);

        // White background
        painter.fillRect(rect(), Qt::white);

        std::lock_guard<std::mutex> lock(data_mutex_);

        // Calculate plot area with margins
        int margin = 70;
        int plot_width = width() - 2 * margin;
        int plot_height = height() - 2 * margin;

        // Calculate scale to maintain aspect ratio (equal scaling for X and Y)
        double data_range_x = padded_max_x_ - padded_min_x_;
        double data_range_y = padded_max_y_ - padded_min_y_;
        
        if (data_range_x < 0.001) data_range_x = 1.0;
        if (data_range_y < 0.001) data_range_y = 1.0;
        
        // Use the same scale for both axes (meters per pixel)
        double scale_x = data_range_x / plot_width;
        double scale_y = data_range_y / plot_height;
        scale_ = std::max(scale_x, scale_y);  // Use larger scale to fit both
        
        // Calculate actual plot dimensions with equal aspect ratio
        actual_plot_width_ = static_cast<int>(data_range_x / scale_);
        actual_plot_height_ = static_cast<int>(data_range_y / scale_);
        
        // Center the plot
        int offset_x = margin + (plot_width - actual_plot_width_) / 2;
        int offset_y = margin + (plot_height - actual_plot_height_) / 2;
        
        plot_offset_x_ = offset_x;
        plot_offset_y_ = offset_y;

        // Draw plot border
        painter.setPen(QPen(Qt::black, 1));
        painter.drawRect(offset_x, offset_y, actual_plot_width_, actual_plot_height_);

        // Draw grid
        drawGrid(painter, offset_x, offset_y, actual_plot_width_, actual_plot_height_);

        // Draw axis labels
        drawAxisLabels(painter, offset_x, offset_y, actual_plot_width_, actual_plot_height_);

        // Draw paths
        if (show_target_path_ && !target_path_.empty()) {
            drawPath(painter, target_path_, QColor(0, 100, 255), offset_x, offset_y, "Target");
        }
        if (show_real_path_ && !real_path_.empty()) {
            drawPath(painter, real_path_, QColor(0, 180, 0), offset_x, offset_y, "Real");
        }
        if (show_return_path_ && !return_path_.empty()) {
            drawPath(painter, return_path_, QColor(255, 165, 0), offset_x, offset_y, "Return");
        }

        // Draw title
        painter.setPen(Qt::black);
        QFont title_font("Arial", 14, QFont::Bold);
        painter.setFont(title_font);
        painter.drawText(margin, 25, "Path Visualization Plot");

        // Draw legend
        drawLegend(painter, width() - 130, margin + 10);

        // Draw current position marker
        if (show_real_path_ && !real_path_.empty()) {
            const auto& last = real_path_.back();
            int px = transformX(last.x, offset_x);
            int py = transformY(last.y, offset_y);
            
            painter.setBrush(QColor(255, 0, 0));
            painter.setPen(QPen(Qt::black, 1));
            painter.drawEllipse(QPoint(px, py), 6, 6);
        }
    }

private:
    void updateBounds(double x, double y)
    {
        if (!bounds_initialized_) {
            min_x_ = x; max_x_ = x;
            min_y_ = y; max_y_ = y;
            bounds_initialized_ = true;
        } else {
            min_x_ = std::min(min_x_, x);
            max_x_ = std::max(max_x_, x);
            min_y_ = std::min(min_y_, y);
            max_y_ = std::max(max_y_, y);
        }
        
        // Add padding (10% on each side)
        double range_x = max_x_ - min_x_;
        double range_y = max_y_ - min_y_;
        if (range_x < 0.1) range_x = 1.0;
        if (range_y < 0.1) range_y = 1.0;
        
        padded_min_x_ = min_x_ - range_x * 0.1;
        padded_max_x_ = max_x_ + range_x * 0.1;
        padded_min_y_ = min_y_ - range_y * 0.1;
        padded_max_y_ = max_y_ + range_y * 0.1;
    }

    int transformX(double x, int offset_x) const
    {
        return offset_x + static_cast<int>((x - padded_min_x_) / scale_);
    }

    int transformY(double y, int offset_y) const
    {
        // Flip Y axis (Qt Y increases downward, we want it to increase upward)
        return offset_y + actual_plot_height_ - static_cast<int>((y - padded_min_y_) / scale_);
    }

    void drawGrid(QPainter& painter, int offset_x, int offset_y, int plot_width, int plot_height)
    {
        painter.setPen(QPen(QColor(200, 200, 200), 1, Qt::DashLine));
        
        // Calculate grid spacing based on data range
        double data_range_x = padded_max_x_ - padded_min_x_;
        double data_range_y = padded_max_y_ - padded_min_y_;
        
        // Aim for approximately 10 grid lines
        double grid_step = std::pow(10, std::floor(std::log10(std::max(data_range_x, data_range_y) / 5)));
        if (grid_step < 0.1) grid_step = 0.1;
        
        // Vertical grid lines
        double x_start = std::ceil(padded_min_x_ / grid_step) * grid_step;
        for (double gx = x_start; gx <= padded_max_x_; gx += grid_step) {
            int px = transformX(gx, offset_x);
            if (px >= offset_x && px <= offset_x + plot_width) {
                painter.drawLine(px, offset_y, px, offset_y + plot_height);
            }
        }
        
        // Horizontal grid lines
        double y_start = std::ceil(padded_min_y_ / grid_step) * grid_step;
        for (double gy = y_start; gy <= padded_max_y_; gy += grid_step) {
            int py = transformY(gy, offset_y);
            if (py >= offset_y && py <= offset_y + plot_height) {
                painter.drawLine(offset_x, py, offset_x + plot_width, py);
            }
        }
    }

    void drawAxisLabels(QPainter& painter, int offset_x, int offset_y, int plot_width, int plot_height)
    {
        painter.setPen(Qt::black);
        QFont axis_font("Arial", 9);
        painter.setFont(axis_font);
        
        double data_range_x = padded_max_x_ - padded_min_x_;
        double data_range_y = padded_max_y_ - padded_min_y_;
        
        // Calculate nice tick spacing
        double tick_step = std::pow(10, std::floor(std::log10(std::max(data_range_x, data_range_y) / 5)));
        if (tick_step < 0.1) tick_step = 0.1;
        
        // X-axis labels
        double x_start = std::ceil(padded_min_x_ / tick_step) * tick_step;
        for (double gx = x_start; gx <= padded_max_x_; gx += tick_step) {
            int px = transformX(gx, offset_x);
            if (px >= offset_x && px <= offset_x + plot_width) {
                QString label = QString::number(gx, 'f', 1);
                painter.drawText(px - 20, offset_y + plot_height + 18, 40, 20, Qt::AlignCenter, label);
            }
        }
        
        // Y-axis labels
        double y_start = std::ceil(padded_min_y_ / tick_step) * tick_step;
        for (double gy = y_start; gy <= padded_max_y_; gy += tick_step) {
            int py = transformY(gy, offset_y);
            if (py >= offset_y && py <= offset_y + plot_height) {
                QString label = QString::number(gy, 'f', 1);
                painter.drawText(offset_x - 55, py - 10, 50, 20, Qt::AlignRight | Qt::AlignVCenter, label);
            }
        }
        
        // Axis titles
        QFont label_font("Arial", 11, QFont::Bold);
        painter.setFont(label_font);
        painter.drawText(offset_x + plot_width / 2 - 40, height() - 10, "X (meters)");
        
        painter.save();
        painter.translate(15, offset_y + plot_height / 2 + 40);
        painter.rotate(-90);
        painter.drawText(0, 0, "Y (meters)");
        painter.restore();
    }

    void drawPath(QPainter& painter, const std::vector<Point2D>& path,
                  const QColor& color, int offset_x, int offset_y,
                  const QString& /*name*/)
    {
        if (path.size() < 2) {
            // Draw single point
            if (path.size() == 1) {
                int px = transformX(path[0].x, offset_x);
                int py = transformY(path[0].y, offset_y);
                painter.setBrush(color);
                painter.setPen(color);
                painter.drawEllipse(QPoint(px, py), 4, 4);
            }
            return;
        }
        
        // Draw path line
        painter.setPen(QPen(color, 2));
        for (size_t i = 1; i < path.size(); ++i) {
            int x1 = transformX(path[i-1].x, offset_x);
            int y1 = transformY(path[i-1].y, offset_y);
            int x2 = transformX(path[i].x, offset_x);
            int y2 = transformY(path[i].y, offset_y);
            painter.drawLine(x1, y1, x2, y2);
        }
        
        // Draw start point marker
        int start_x = transformX(path[0].x, offset_x);
        int start_y = transformY(path[0].y, offset_y);
        painter.setBrush(color.lighter(130));
        painter.setPen(QPen(color.darker(120), 1));
        painter.drawEllipse(QPoint(start_x, start_y), 5, 5);
    }

    void drawLegend(QPainter& painter, int x, int y)
    {
        // Draw legend background
        painter.fillRect(x - 5, y - 5, 125, 115, QColor(255, 255, 255, 230));
        painter.setPen(QPen(Qt::gray, 1));
        painter.drawRect(x - 5, y - 5, 125, 115);
        
        painter.setPen(Qt::black);
        QFont legend_font("Arial", 10);
        painter.setFont(legend_font);
        
        int spacing = 25;
        int box_size = 15;
        
        // Real path
        if (show_real_path_) {
            painter.fillRect(x, y, box_size, box_size, QColor(0, 180, 0));
            painter.setPen(Qt::black);
            painter.drawRect(x, y, box_size, box_size);
            painter.drawText(x + box_size + 5, y + 12, "Real Path");
            y += spacing;
        }
        
        // Target path
        if (show_target_path_) {
            painter.fillRect(x, y, box_size, box_size, QColor(0, 100, 255));
            painter.setPen(Qt::black);
            painter.drawRect(x, y, box_size, box_size);
            painter.drawText(x + box_size + 5, y + 12, "Target Path");
            y += spacing;
        }
        
        // Return path
        if (show_return_path_) {
            painter.fillRect(x, y, box_size, box_size, QColor(255, 165, 0));
            painter.setPen(Qt::black);
            painter.drawRect(x, y, box_size, box_size);
            painter.drawText(x + box_size + 5, y + 12, "Return Path");
            y += spacing;
        }
        
        // Current position
        painter.setBrush(QColor(255, 0, 0));
        painter.setPen(QPen(Qt::black, 1));
        painter.drawEllipse(QPoint(x + box_size/2, y + box_size/2), 5, 5);
        painter.drawText(x + box_size + 5, y + 12, "Current Pos");
    }

    std::vector<Point2D> real_path_;
    std::vector<Point2D> target_path_;
    std::vector<Point2D> return_path_;
    
    double min_x_ = 0, max_x_ = 1;
    double min_y_ = 0, max_y_ = 1;
    double padded_min_x_ = 0, padded_max_x_ = 1;
    double padded_min_y_ = 0, padded_max_y_ = 1;
    
    bool bounds_initialized_ = false;
    
    // For equal aspect ratio
    double scale_ = 1.0;
    int actual_plot_width_ = 1;
    int actual_plot_height_ = 1;
    int plot_offset_x_ = 0;
    int plot_offset_y_ = 0;
    
    bool show_real_path_ = true;
    bool show_target_path_ = true;
    bool show_return_path_ = true;
    
    std::mutex data_mutex_;
};

// Main window containing the plot and controls
class PlotWindow : public QMainWindow
{
    Q_OBJECT

public:
    PlotWindow(QWidget* parent = nullptr) : QMainWindow(parent)
    {
        setWindowTitle("Path Plotter - ROS2 Visualization");
        setMinimumSize(900, 700);

        QWidget* central_widget = new QWidget(this);
        setCentralWidget(central_widget);
        
        QVBoxLayout* main_layout = new QVBoxLayout(central_widget);
        main_layout->setContentsMargins(10, 10, 10, 10);
        
        // Control panel
        QFrame* control_frame = new QFrame();
        control_frame->setFrameShape(QFrame::StyledPanel);
        control_frame->setStyleSheet("QFrame { background-color: #f0f0f0; border-radius: 4px; padding: 5px; }");
        
        QHBoxLayout* control_layout = new QHBoxLayout(control_frame);
        
        // Checkboxes
        real_path_cb_ = new QCheckBox("Show Real Path");
        real_path_cb_->setChecked(true);
        control_layout->addWidget(real_path_cb_);
        
        target_path_cb_ = new QCheckBox("Show Target Path");
        target_path_cb_->setChecked(true);
        control_layout->addWidget(target_path_cb_);
        
        return_path_cb_ = new QCheckBox("Show Return Path");
        return_path_cb_->setChecked(true);
        control_layout->addWidget(return_path_cb_);
        
        control_layout->addStretch();
        
        // Status labels
        status_label_ = new QLabel("Points: Real=0, Target=0, Return=0");
        control_layout->addWidget(status_label_);
        
        control_layout->addStretch();
        
        // Save button
        QPushButton* save_btn = new QPushButton("Save Plot");
        save_btn->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; padding: 5px 15px; border: none; border-radius: 3px; }"
                                "QPushButton:hover { background-color: #45a049; }");
        control_layout->addWidget(save_btn);
        
        // Clear button
        QPushButton* clear_btn = new QPushButton("Clear All");
        clear_btn->setStyleSheet("QPushButton { background-color: #f44336; color: white; padding: 5px 15px; border: none; border-radius: 3px; }"
                                 "QPushButton:hover { background-color: #da190b; }");
        control_layout->addWidget(clear_btn);
        
        main_layout->addWidget(control_frame);
        
        // Plot widget
        plot_widget_ = new PlotWidget();
        main_layout->addWidget(plot_widget_, 1);
        
        // Connect signals
        connect(real_path_cb_, &QCheckBox::toggled, plot_widget_, &PlotWidget::setShowRealPath);
        connect(target_path_cb_, &QCheckBox::toggled, plot_widget_, &PlotWidget::setShowTargetPath);
        connect(return_path_cb_, &QCheckBox::toggled, plot_widget_, &PlotWidget::setShowReturnPath);
        connect(clear_btn, &QPushButton::clicked, this, &PlotWindow::clearPaths);
        connect(save_btn, &QPushButton::clicked, this, &PlotWindow::savePlot);
        
        // Update timer
        update_timer_ = new QTimer(this);
        connect(update_timer_, &QTimer::timeout, this, &PlotWindow::updatePlot);
        update_timer_->start(100); // 10 Hz update
    }

    PlotWidget* getPlotWidget() { return plot_widget_; }

public slots:
    void clearPaths()
    {
        plot_widget_->clearPaths();
        real_count_ = 0;
        target_count_ = 0;
        return_count_ = 0;
        updateStatusLabel();
        plot_widget_->update();
    }
    
    void updatePlot()
    {
        plot_widget_->update();
    }
    
    void savePlot()
    {
        QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");
        QString default_filename = QString("path_plot_%1.png").arg(timestamp);
        
        QString filename = QFileDialog::getSaveFileName(
            this,
            "Save Plot",
            default_filename,
            "PNG Image (*.png);;JPEG Image (*.jpg);;All Files (*)"
        );
        
        if (!filename.isEmpty()) {
            if (plot_widget_->savePlot(filename)) {
                QMessageBox::information(this, "Success", 
                    QString("Plot saved to:\n%1").arg(filename));
            } else {
                QMessageBox::warning(this, "Error", 
                    "Failed to save plot.");
            }
        }
    }
    
    void incrementRealCount() { real_count_++; updateStatusLabel(); }
    void incrementTargetCount() { target_count_++; updateStatusLabel(); }
    void incrementReturnCount() { return_count_++; updateStatusLabel(); }

private:
    void updateStatusLabel()
    {
        status_label_->setText(QString("Points: Real=%1, Target=%2, Return=%3")
            .arg(real_count_).arg(target_count_).arg(return_count_));
    }

    PlotWidget* plot_widget_;
    QCheckBox* real_path_cb_;
    QCheckBox* target_path_cb_;
    QCheckBox* return_path_cb_;
    QLabel* status_label_;
    QTimer* update_timer_;
    
    int real_count_ = 0;
    int target_count_ = 0;
    int return_count_ = 0;
};

// ROS2 Node that subscribes to path data
class PathPlotterNode : public rclcpp::Node
{
public:
    PathPlotterNode(PlotWindow* window) 
        : Node("path_plotter_node"), window_(window)
    {
        plot_widget_ = window_->getPlotWidget();
        
        // QoS profile for MAVROS compatibility
        auto qos_profile = rclcpp::QoS(10)
            .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
            .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE)
            .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);

        // Subscribers
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/simulation_pose_info",
            qos_profile,
            std::bind(&PathPlotterNode::state_callback, this, std::placeholders::_1)
        );

        target_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/target_pose",
            10,
            std::bind(&PathPlotterNode::target_pose_callback, this, std::placeholders::_1)
        );

        returning_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/returning_status",
            10,
            std::bind(&PathPlotterNode::returning_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Path Plotter Node initialized");
    }

private:
    void state_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        if (msg->poses.size() > 2) {
            const auto& pose = msg->poses[2];
            
            if (!has_starting_point_) {
                starting_x_ = pose.position.x;
                starting_y_ = pose.position.y;
                has_starting_point_ = true;
                RCLCPP_INFO(this->get_logger(), "Starting point set: (%.2f, %.2f)", 
                           starting_x_, starting_y_);
            }
            
            double x = pose.position.x - starting_x_;
            double y = pose.position.y - starting_y_;
            
            if (returning_) {
                plot_widget_->addReturnPoint(x, y);
                QMetaObject::invokeMethod(window_, "incrementReturnCount", Qt::QueuedConnection);
            } else {
                plot_widget_->addRealPoint(x, y);
                QMetaObject::invokeMethod(window_, "incrementRealCount", Qt::QueuedConnection);
            }
        }
    }

    void target_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        if (!has_starting_point_) {
            return;
        }
        
        double x = msg->position.x - starting_x_;
        double y = msg->position.y - starting_y_;
        
        plot_widget_->addTargetPoint(x, y);
        QMetaObject::invokeMethod(window_, "incrementTargetCount", Qt::QueuedConnection);
    }

    void returning_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        returning_ = msg->data;
        if (returning_) {
            RCLCPP_INFO(this->get_logger(), "Switching to return path mode");
        }
    }

    PlotWindow* window_;
    PlotWidget* plot_widget_;
    
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr target_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr returning_sub_;
    
    bool has_starting_point_ = false;
    double starting_x_ = 0;
    double starting_y_ = 0;
    bool returning_ = false;
};

// Include MOC file
#include "path_plotter.moc"

int main(int argc, char** argv)
{
    // Initialize both ROS2 and Qt
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);
    
    // Create window
    PlotWindow window;
    window.show();
    
    // Create ROS2 node
    auto node = std::make_shared<PathPlotterNode>(&window);
    
    // Timer to spin ROS2
    QTimer ros_timer;
    QObject::connect(&ros_timer, &QTimer::timeout, [&node]() {
        rclcpp::spin_some(node);
    });
    ros_timer.start(10); // 100 Hz
    
    // Run Qt event loop
    int result = app.exec();
    
    rclcpp::shutdown();
    return result;
}
