#include <QApplication>
#include <QStyleFactory>
#include <rclcpp/rclcpp.hpp>
#include "my_widget.hpp"
#include <QFile>
#include <QTextStream>


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);

    MyWidget widget;
    widget.show();

    int ret = app.exec();
    rclcpp::shutdown();
    return ret;
}

