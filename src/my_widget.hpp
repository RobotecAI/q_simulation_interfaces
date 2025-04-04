#pragma once

#include <QWidget>
#include <rclcpp/rclcpp.hpp>

namespace Ui {
class MyWidgetUI;
}

class MyWidget : public QWidget
{
  Q_OBJECT

public:
  explicit MyWidget(QWidget *parent = nullptr);
  ~MyWidget();


private:
  void GetSpawnables();
  void SpawnButton();
  void GetAllEntities();
  void GetEntityState();
  void SetEntityState();
  void DespawnButton();

  Ui::MyWidgetUI *ui_;
  rclcpp::Node::SharedPtr node_;
};

