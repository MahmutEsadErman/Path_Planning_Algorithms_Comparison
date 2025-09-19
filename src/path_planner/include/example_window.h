#ifndef EXAMPLE_WINDOW_H // Prevents the header from being included multiple times
#define EXAMPLE_WINDOW_H

#include <QMainWindow>
#include "rclcpp/rclcpp.hpp"
// #include "ui_example_window.h" // The header generated from your .ui file
#include "../ui/ui_example_window.h"
#include "grid_widget.hpp"


class ExampleWindow : public QMainWindow, private Ui::ExampleWindow
{
    Q_OBJECT

private:
    rclcpp::Node::SharedPtr nh_;

public:
    explicit ExampleWindow(rclcpp::Node::SharedPtr & node_handle, QWidget *parent = nullptr);
};

#endif // EXAMPLE_WINDOW_H