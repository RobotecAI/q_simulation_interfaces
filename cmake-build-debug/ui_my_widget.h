/********************************************************************************
** Form generated from reading UI file 'my_widget.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MY_WIDGET_H
#define UI_MY_WIDGET_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_my_widget
{
public:
    QGridLayout *gridLayout;
    QVBoxLayout *verticalLayout;
    QLabel *label;
    QPushButton *getAllEntitiesButton;
    QComboBox *ComboEntities;
    QPushButton *despawnButton;
    QVBoxLayout *verticalLayout_3;
    QPushButton *getEntityStateButton;
    QGridLayout *gridLayout_2;
    QDoubleSpinBox *StatePosZ;
    QLabel *label_11;
    QLabel *label_7;
    QDoubleSpinBox *StatePosX;
    QLabel *label_13;
    QLabel *label_6;
    QDoubleSpinBox *StatePosY;
    QLabel *label_15;
    QLineEdit *RotVector;
    QDoubleSpinBox *StateVelZ;
    QLabel *label_9;
    QLabel *RotVectorX;
    QLabel *label_8;
    QLabel *label_14;
    QDoubleSpinBox *StateVelX;
    QDoubleSpinBox *StateVelRotY;
    QLabel *label_12;
    QDoubleSpinBox *StateVelY;
    QDoubleSpinBox *StateVelRotZ;
    QLabel *label_10;
    QDoubleSpinBox *StateVelRotX;
    QDoubleSpinBox *doubleSpinBox;
    QPushButton *setEntityStateButton;
    QVBoxLayout *verticalLayout_2;
    QLabel *label_2;
    QHBoxLayout *horizontalLayout_2;
    QPushButton *PushButtonRefresh;
    QComboBox *ComboSpawables;
    QHBoxLayout *horizontalLayout_5;
    QLabel *label_3;
    QDoubleSpinBox *doubleSpinBoxX;
    QLabel *label_4;
    QDoubleSpinBox *doubleSpinBoxY;
    QLabel *label_5;
    QDoubleSpinBox *doubleSpinBoxZ;
    QHBoxLayout *horizontalLayout;
    QLabel *Name;
    QLineEdit *lineEditName;
    QPushButton *SpawnButton;
    QSpacerItem *verticalSpacer;

    void setupUi(QWidget *my_widget)
    {
        if (my_widget->objectName().isEmpty())
            my_widget->setObjectName(QString::fromUtf8("my_widget"));
        my_widget->resize(672, 724);
        gridLayout = new QGridLayout(my_widget);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        label = new QLabel(my_widget);
        label->setObjectName(QString::fromUtf8("label"));

        verticalLayout->addWidget(label);

        getAllEntitiesButton = new QPushButton(my_widget);
        getAllEntitiesButton->setObjectName(QString::fromUtf8("getAllEntitiesButton"));

        verticalLayout->addWidget(getAllEntitiesButton);

        ComboEntities = new QComboBox(my_widget);
        ComboEntities->setObjectName(QString::fromUtf8("ComboEntities"));

        verticalLayout->addWidget(ComboEntities);

        despawnButton = new QPushButton(my_widget);
        despawnButton->setObjectName(QString::fromUtf8("despawnButton"));

        verticalLayout->addWidget(despawnButton);

        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        getEntityStateButton = new QPushButton(my_widget);
        getEntityStateButton->setObjectName(QString::fromUtf8("getEntityStateButton"));

        verticalLayout_3->addWidget(getEntityStateButton);

        gridLayout_2 = new QGridLayout();
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        StatePosZ = new QDoubleSpinBox(my_widget);
        StatePosZ->setObjectName(QString::fromUtf8("StatePosZ"));
        StatePosZ->setMinimum(-999999.000000000000000);
        StatePosZ->setMaximum(999999999999999.000000000000000);

        gridLayout_2->addWidget(StatePosZ, 2, 3, 1, 1);

        label_11 = new QLabel(my_widget);
        label_11->setObjectName(QString::fromUtf8("label_11"));

        gridLayout_2->addWidget(label_11, 5, 4, 1, 1);

        label_7 = new QLabel(my_widget);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        gridLayout_2->addWidget(label_7, 1, 1, 1, 1);

        StatePosX = new QDoubleSpinBox(my_widget);
        StatePosX->setObjectName(QString::fromUtf8("StatePosX"));
        StatePosX->setMinimum(-999999.000000000000000);
        StatePosX->setMaximum(999999999999999.000000000000000);

        gridLayout_2->addWidget(StatePosX, 0, 3, 1, 1);

        label_13 = new QLabel(my_widget);
        label_13->setObjectName(QString::fromUtf8("label_13"));

        gridLayout_2->addWidget(label_13, 1, 4, 1, 1);

        label_6 = new QLabel(my_widget);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        gridLayout_2->addWidget(label_6, 0, 1, 1, 1);

        StatePosY = new QDoubleSpinBox(my_widget);
        StatePosY->setObjectName(QString::fromUtf8("StatePosY"));
        StatePosY->setMinimum(-999999.000000000000000);
        StatePosY->setMaximum(999999999999999.000000000000000);

        gridLayout_2->addWidget(StatePosY, 1, 3, 1, 1);

        label_15 = new QLabel(my_widget);
        label_15->setObjectName(QString::fromUtf8("label_15"));

        gridLayout_2->addWidget(label_15, 4, 1, 1, 1);

        RotVector = new QLineEdit(my_widget);
        RotVector->setObjectName(QString::fromUtf8("RotVector"));

        gridLayout_2->addWidget(RotVector, 3, 3, 1, 1);

        StateVelZ = new QDoubleSpinBox(my_widget);
        StateVelZ->setObjectName(QString::fromUtf8("StateVelZ"));

        gridLayout_2->addWidget(StateVelZ, 5, 5, 1, 1);

        label_9 = new QLabel(my_widget);
        label_9->setObjectName(QString::fromUtf8("label_9"));

        gridLayout_2->addWidget(label_9, 3, 4, 1, 1);

        RotVectorX = new QLabel(my_widget);
        RotVectorX->setObjectName(QString::fromUtf8("RotVectorX"));

        gridLayout_2->addWidget(RotVectorX, 3, 1, 1, 1);

        label_8 = new QLabel(my_widget);
        label_8->setObjectName(QString::fromUtf8("label_8"));

        gridLayout_2->addWidget(label_8, 2, 1, 1, 1);

        label_14 = new QLabel(my_widget);
        label_14->setObjectName(QString::fromUtf8("label_14"));

        gridLayout_2->addWidget(label_14, 2, 4, 1, 1);

        StateVelX = new QDoubleSpinBox(my_widget);
        StateVelX->setObjectName(QString::fromUtf8("StateVelX"));

        gridLayout_2->addWidget(StateVelX, 3, 5, 1, 1);

        StateVelRotY = new QDoubleSpinBox(my_widget);
        StateVelRotY->setObjectName(QString::fromUtf8("StateVelRotY"));

        gridLayout_2->addWidget(StateVelRotY, 1, 5, 1, 1);

        label_12 = new QLabel(my_widget);
        label_12->setObjectName(QString::fromUtf8("label_12"));

        gridLayout_2->addWidget(label_12, 0, 4, 1, 1);

        StateVelY = new QDoubleSpinBox(my_widget);
        StateVelY->setObjectName(QString::fromUtf8("StateVelY"));

        gridLayout_2->addWidget(StateVelY, 4, 5, 1, 1);

        StateVelRotZ = new QDoubleSpinBox(my_widget);
        StateVelRotZ->setObjectName(QString::fromUtf8("StateVelRotZ"));

        gridLayout_2->addWidget(StateVelRotZ, 2, 5, 1, 1);

        label_10 = new QLabel(my_widget);
        label_10->setObjectName(QString::fromUtf8("label_10"));

        gridLayout_2->addWidget(label_10, 4, 4, 1, 1);

        StateVelRotX = new QDoubleSpinBox(my_widget);
        StateVelRotX->setObjectName(QString::fromUtf8("StateVelRotX"));

        gridLayout_2->addWidget(StateVelRotX, 0, 5, 1, 1);

        doubleSpinBox = new QDoubleSpinBox(my_widget);
        doubleSpinBox->setObjectName(QString::fromUtf8("doubleSpinBox"));

        gridLayout_2->addWidget(doubleSpinBox, 4, 3, 1, 1);


        verticalLayout_3->addLayout(gridLayout_2);

        setEntityStateButton = new QPushButton(my_widget);
        setEntityStateButton->setObjectName(QString::fromUtf8("setEntityStateButton"));

        verticalLayout_3->addWidget(setEntityStateButton);


        verticalLayout->addLayout(verticalLayout_3);


        gridLayout->addLayout(verticalLayout, 0, 0, 1, 1);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        label_2 = new QLabel(my_widget);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        verticalLayout_2->addWidget(label_2);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        PushButtonRefresh = new QPushButton(my_widget);
        PushButtonRefresh->setObjectName(QString::fromUtf8("PushButtonRefresh"));

        horizontalLayout_2->addWidget(PushButtonRefresh);

        ComboSpawables = new QComboBox(my_widget);
        ComboSpawables->setObjectName(QString::fromUtf8("ComboSpawables"));

        horizontalLayout_2->addWidget(ComboSpawables);


        verticalLayout_2->addLayout(horizontalLayout_2);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        label_3 = new QLabel(my_widget);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        horizontalLayout_5->addWidget(label_3);

        doubleSpinBoxX = new QDoubleSpinBox(my_widget);
        doubleSpinBoxX->setObjectName(QString::fromUtf8("doubleSpinBoxX"));
        doubleSpinBoxX->setMinimum(-999999.000000000000000);
        doubleSpinBoxX->setMaximum(999999999999999.000000000000000);

        horizontalLayout_5->addWidget(doubleSpinBoxX);

        label_4 = new QLabel(my_widget);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        horizontalLayout_5->addWidget(label_4);

        doubleSpinBoxY = new QDoubleSpinBox(my_widget);
        doubleSpinBoxY->setObjectName(QString::fromUtf8("doubleSpinBoxY"));
        doubleSpinBoxY->setMinimum(-999999.000000000000000);
        doubleSpinBoxY->setMaximum(999999999999999.000000000000000);

        horizontalLayout_5->addWidget(doubleSpinBoxY);

        label_5 = new QLabel(my_widget);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        horizontalLayout_5->addWidget(label_5);

        doubleSpinBoxZ = new QDoubleSpinBox(my_widget);
        doubleSpinBoxZ->setObjectName(QString::fromUtf8("doubleSpinBoxZ"));
        doubleSpinBoxZ->setMinimum(-999999.000000000000000);
        doubleSpinBoxZ->setMaximum(999999999999999.000000000000000);

        horizontalLayout_5->addWidget(doubleSpinBoxZ);


        verticalLayout_2->addLayout(horizontalLayout_5);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        Name = new QLabel(my_widget);
        Name->setObjectName(QString::fromUtf8("Name"));

        horizontalLayout->addWidget(Name);

        lineEditName = new QLineEdit(my_widget);
        lineEditName->setObjectName(QString::fromUtf8("lineEditName"));

        horizontalLayout->addWidget(lineEditName);


        verticalLayout_2->addLayout(horizontalLayout);

        SpawnButton = new QPushButton(my_widget);
        SpawnButton->setObjectName(QString::fromUtf8("SpawnButton"));

        verticalLayout_2->addWidget(SpawnButton);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_2->addItem(verticalSpacer);


        gridLayout->addLayout(verticalLayout_2, 1, 0, 1, 1);


        retranslateUi(my_widget);

        QMetaObject::connectSlotsByName(my_widget);
    } // setupUi

    void retranslateUi(QWidget *my_widget)
    {
        label->setText(QCoreApplication::translate("my_widget", "Entities State", nullptr));
        getAllEntitiesButton->setText(QCoreApplication::translate("my_widget", "GetAllEntities", nullptr));
        despawnButton->setText(QCoreApplication::translate("my_widget", "Despawn", nullptr));
        getEntityStateButton->setText(QCoreApplication::translate("my_widget", "GetEntityState", nullptr));
        label_11->setText(QCoreApplication::translate("my_widget", "VelZ:", nullptr));
        label_7->setText(QCoreApplication::translate("my_widget", "PosY:", nullptr));
        label_13->setText(QCoreApplication::translate("my_widget", "VelRotY", nullptr));
        label_6->setText(QCoreApplication::translate("my_widget", "PosX:", nullptr));
        label_15->setText(QCoreApplication::translate("my_widget", "RotAngle", nullptr));
        label_9->setText(QCoreApplication::translate("my_widget", "VelX:", nullptr));
        RotVectorX->setText(QCoreApplication::translate("my_widget", "RotVector", nullptr));
        label_8->setText(QCoreApplication::translate("my_widget", "PosZ:", nullptr));
        label_14->setText(QCoreApplication::translate("my_widget", "VelRotZ", nullptr));
        label_12->setText(QCoreApplication::translate("my_widget", "VelRotX", nullptr));
        label_10->setText(QCoreApplication::translate("my_widget", "VelY:", nullptr));
        setEntityStateButton->setText(QCoreApplication::translate("my_widget", "SetEntityState", nullptr));
        label_2->setText(QCoreApplication::translate("my_widget", "Spawning", nullptr));
        PushButtonRefresh->setText(QCoreApplication::translate("my_widget", "Refresh", nullptr));
        label_3->setText(QCoreApplication::translate("my_widget", "X:", nullptr));
        label_4->setText(QCoreApplication::translate("my_widget", "Y:", nullptr));
        label_5->setText(QCoreApplication::translate("my_widget", "Z:", nullptr));
        Name->setText(QCoreApplication::translate("my_widget", "Name", nullptr));
        SpawnButton->setText(QCoreApplication::translate("my_widget", "Spawn!", nullptr));
        (void)my_widget;
    } // retranslateUi

};

namespace Ui {
    class my_widget: public Ui_my_widget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MY_WIDGET_H
