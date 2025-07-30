#include <QApplication>
#include <QStyleFactory>
#include <rclcpp/rclcpp.hpp>
#include <QFile>
#include <QTextStream>
#include <QWidget>
#include <QVBoxLayout>
#include "q_simulation_interfaces/simulation_panel.hpp"

class StandaloneWidget : public QWidget
{
public:
    StandaloneWidget(QWidget *parent = nullptr) : QWidget(parent)
    {
        // Initialize ROS node
        node_ = rclcpp::Node::make_shared("standalone_simulation_widget");
        setWindowTitle("Simulation Interface");
        setMinimumSize(800, 600);
        
        auto layout = new QVBoxLayout(this);
        
        // Create a mock display context for the panel
        panel_ = new q_simulation_interfaces::SimulationWidget(this);
        panel_->intiliaze(node_);
        layout->addWidget(panel_);
        node_thread_ = std::thread([this]() {
              rclcpp::spin(node_);
          });

        // Ensure the node is shutdown when the widget is closed
        connect(this, &QWidget::destroyed, this, [this]() {
            if (node_thread_.joinable()) {
                node_thread_.join();
            }
            rclcpp::shutdown();
        });
    }
    
    ~StandaloneWidget()
    {
    }

private:
    q_simulation_interfaces::SimulationWidget *panel_;
    rclcpp::Node::SharedPtr node_;
    std::thread node_thread_;


};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);

    StandaloneWidget widget;
    widget.show();

    int ret = app.exec();
    rclcpp::shutdown();
    return ret;
}