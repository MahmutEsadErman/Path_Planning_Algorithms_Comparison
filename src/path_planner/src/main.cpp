#include <QApplication>

#include <rclcpp/rclcpp.hpp>

#include "example_window.h"


class PathPlannerApp : public QApplication
{
public:
	rclcpp::Node::SharedPtr nh_;

	explicit PathPlannerApp(int & argc, char ** argv): QApplication(argc, argv){
		rclcpp::init(argc, argv);
		nh_ = rclcpp::Node::make_shared("path_planner");
	}

	~PathPlannerApp()
	{
		rclcpp::shutdown();
	}

	int exec()
	{
		ExampleWindow main_window(nh_);
		main_window.show();

		return QApplication::exec();
	}
};

int main(int argc, char ** argv)
{
	PathPlannerApp app(argc, argv);
	return app.exec();
}
