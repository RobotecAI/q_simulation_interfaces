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
        setWindowTitle("Simulation Interface");
        setMinimumSize(800, 600);
        
        auto layout = new QVBoxLayout(this);
        
        // Create a mock display context for the panel
        panel_ = new q_simulation_interfaces::SimulationPanel(this);
        panel_->onInitialize();
        layout->addWidget(panel_);

    }
    
    ~StandaloneWidget()
    {
    }

private:
    q_simulation_interfaces::SimulationPanel *panel_;
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