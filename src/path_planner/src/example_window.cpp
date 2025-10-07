#include "example_window.h"
#include "rclcpp/rclcpp.hpp"
#include "grid_widget.hpp"
#include <QVBoxLayout>

ExampleWindow::ExampleWindow(rclcpp::Node::SharedPtr & node_handle, QWidget *parent) : QMainWindow(parent) {
    // Initialize widgets created from the .ui file
    setupUi(this);

    // Store ROS 2 node handle
    nh_ = node_handle;

    // Ensure the placeholder widget from the UI (named "grid_widget") has a layout
    // and add our custom GridWidget into it. Give the placeholder as the parent so
    // Qt takes ownership and handles lifetime automatically.
    if (!grid_widget->layout()) {
        auto vlayout = new QVBoxLayout(grid_widget);
        vlayout->setContentsMargins(0, 0, 0, 0);
        grid_widget->setLayout(vlayout);
    }

    auto canvas = new GridWidget(nh_, grid_widget);
    grid_widget->layout()->addWidget(canvas);
}
