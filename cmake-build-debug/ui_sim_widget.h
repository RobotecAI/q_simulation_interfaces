/********************************************************************************
** Form generated from reading UI file 'sim_widget.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SIM_WIDGET_H
#define UI_SIM_WIDGET_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QListWidget>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_simWidgetUi
{
public:
    QVBoxLayout *verticalLayout_4;
    QGridLayout *gridLayout;
    QListWidget *listCapabilities;
    QLabel *label_16;
    QPushButton *GetSimCapabilites;
    QPushButton *resetSimButton;
    QHBoxLayout *horizontalLayout_4;
    QLabel *label_17;
    QComboBox *resetModeCombo;
    QVBoxLayout *verticalLayout;
    QLabel *label;
    QPushButton *getAllEntitiesButton;
    QComboBox *ComboEntities;
    QPushButton *despawnButton;
    QPushButton *despawnAll;
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
    QDoubleSpinBox *RotAngle;
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

    void setupUi(QWidget *simWidgetUi)
    {
        if (simWidgetUi->objectName().isEmpty())
            simWidgetUi->setObjectName(QString::fromUtf8("simWidgetUi"));
        simWidgetUi->resize(672, 724);
        verticalLayout_4 = new QVBoxLayout(simWidgetUi);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        gridLayout = new QGridLayout();
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        listCapabilities = new QListWidget(simWidgetUi);
        listCapabilities->setObjectName(QString::fromUtf8("listCapabilities"));

        gridLayout->addWidget(listCapabilities, 1, 1, 1, 1);

        label_16 = new QLabel(simWidgetUi);
        label_16->setObjectName(QString::fromUtf8("label_16"));

        gridLayout->addWidget(label_16, 0, 0, 1, 1);

        GetSimCapabilites = new QPushButton(simWidgetUi);
        GetSimCapabilites->setObjectName(QString::fromUtf8("GetSimCapabilites"));

        gridLayout->addWidget(GetSimCapabilites, 1, 0, 1, 1);

        resetSimButton = new QPushButton(simWidgetUi);
        resetSimButton->setObjectName(QString::fromUtf8("resetSimButton"));

        gridLayout->addWidget(resetSimButton, 2, 0, 1, 1);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        label_17 = new QLabel(simWidgetUi);
        label_17->setObjectName(QString::fromUtf8("label_17"));

        horizontalLayout_4->addWidget(label_17);

        resetModeCombo = new QComboBox(simWidgetUi);
        resetModeCombo->setObjectName(QString::fromUtf8("resetModeCombo"));

        horizontalLayout_4->addWidget(resetModeCombo);


        gridLayout->addLayout(horizontalLayout_4, 2, 1, 1, 1);


        verticalLayout_4->addLayout(gridLayout);

        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        label = new QLabel(simWidgetUi);
        label->setObjectName(QString::fromUtf8("label"));

        verticalLayout->addWidget(label);

        getAllEntitiesButton = new QPushButton(simWidgetUi);
        getAllEntitiesButton->setObjectName(QString::fromUtf8("getAllEntitiesButton"));

        verticalLayout->addWidget(getAllEntitiesButton);

        ComboEntities = new QComboBox(simWidgetUi);
        ComboEntities->setObjectName(QString::fromUtf8("ComboEntities"));

        verticalLayout->addWidget(ComboEntities);

        despawnButton = new QPushButton(simWidgetUi);
        despawnButton->setObjectName(QString::fromUtf8("despawnButton"));

        verticalLayout->addWidget(despawnButton);

        despawnAll = new QPushButton(simWidgetUi);
        despawnAll->setObjectName(QString::fromUtf8("despawnAll"));

        verticalLayout->addWidget(despawnAll);

        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        getEntityStateButton = new QPushButton(simWidgetUi);
        getEntityStateButton->setObjectName(QString::fromUtf8("getEntityStateButton"));

        verticalLayout_3->addWidget(getEntityStateButton);

        gridLayout_2 = new QGridLayout();
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        StatePosZ = new QDoubleSpinBox(simWidgetUi);
        StatePosZ->setObjectName(QString::fromUtf8("StatePosZ"));
        StatePosZ->setMinimum(-999999.000000000000000);
        StatePosZ->setMaximum(999999999999999.000000000000000);

        gridLayout_2->addWidget(StatePosZ, 2, 3, 1, 1);

        label_11 = new QLabel(simWidgetUi);
        label_11->setObjectName(QString::fromUtf8("label_11"));

        gridLayout_2->addWidget(label_11, 5, 4, 1, 1);

        label_7 = new QLabel(simWidgetUi);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        gridLayout_2->addWidget(label_7, 1, 1, 1, 1);

        StatePosX = new QDoubleSpinBox(simWidgetUi);
        StatePosX->setObjectName(QString::fromUtf8("StatePosX"));
        StatePosX->setMinimum(-999999.000000000000000);
        StatePosX->setMaximum(999999999999999.000000000000000);

        gridLayout_2->addWidget(StatePosX, 0, 3, 1, 1);

        label_13 = new QLabel(simWidgetUi);
        label_13->setObjectName(QString::fromUtf8("label_13"));

        gridLayout_2->addWidget(label_13, 1, 4, 1, 1);

        label_6 = new QLabel(simWidgetUi);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        gridLayout_2->addWidget(label_6, 0, 1, 1, 1);

        StatePosY = new QDoubleSpinBox(simWidgetUi);
        StatePosY->setObjectName(QString::fromUtf8("StatePosY"));
        StatePosY->setMinimum(-999999.000000000000000);
        StatePosY->setMaximum(999999999999999.000000000000000);

        gridLayout_2->addWidget(StatePosY, 1, 3, 1, 1);

        label_15 = new QLabel(simWidgetUi);
        label_15->setObjectName(QString::fromUtf8("label_15"));

        gridLayout_2->addWidget(label_15, 4, 1, 1, 1);

        RotVector = new QLineEdit(simWidgetUi);
        RotVector->setObjectName(QString::fromUtf8("RotVector"));

        gridLayout_2->addWidget(RotVector, 3, 3, 1, 1);

        StateVelZ = new QDoubleSpinBox(simWidgetUi);
        StateVelZ->setObjectName(QString::fromUtf8("StateVelZ"));

        gridLayout_2->addWidget(StateVelZ, 5, 5, 1, 1);

        label_9 = new QLabel(simWidgetUi);
        label_9->setObjectName(QString::fromUtf8("label_9"));

        gridLayout_2->addWidget(label_9, 3, 4, 1, 1);

        RotVectorX = new QLabel(simWidgetUi);
        RotVectorX->setObjectName(QString::fromUtf8("RotVectorX"));

        gridLayout_2->addWidget(RotVectorX, 3, 1, 1, 1);

        label_8 = new QLabel(simWidgetUi);
        label_8->setObjectName(QString::fromUtf8("label_8"));

        gridLayout_2->addWidget(label_8, 2, 1, 1, 1);

        label_14 = new QLabel(simWidgetUi);
        label_14->setObjectName(QString::fromUtf8("label_14"));

        gridLayout_2->addWidget(label_14, 2, 4, 1, 1);

        StateVelX = new QDoubleSpinBox(simWidgetUi);
        StateVelX->setObjectName(QString::fromUtf8("StateVelX"));

        gridLayout_2->addWidget(StateVelX, 3, 5, 1, 1);

        StateVelRotY = new QDoubleSpinBox(simWidgetUi);
        StateVelRotY->setObjectName(QString::fromUtf8("StateVelRotY"));

        gridLayout_2->addWidget(StateVelRotY, 1, 5, 1, 1);

        label_12 = new QLabel(simWidgetUi);
        label_12->setObjectName(QString::fromUtf8("label_12"));

        gridLayout_2->addWidget(label_12, 0, 4, 1, 1);

        StateVelY = new QDoubleSpinBox(simWidgetUi);
        StateVelY->setObjectName(QString::fromUtf8("StateVelY"));

        gridLayout_2->addWidget(StateVelY, 4, 5, 1, 1);

        StateVelRotZ = new QDoubleSpinBox(simWidgetUi);
        StateVelRotZ->setObjectName(QString::fromUtf8("StateVelRotZ"));

        gridLayout_2->addWidget(StateVelRotZ, 2, 5, 1, 1);

        label_10 = new QLabel(simWidgetUi);
        label_10->setObjectName(QString::fromUtf8("label_10"));

        gridLayout_2->addWidget(label_10, 4, 4, 1, 1);

        StateVelRotX = new QDoubleSpinBox(simWidgetUi);
        StateVelRotX->setObjectName(QString::fromUtf8("StateVelRotX"));

        gridLayout_2->addWidget(StateVelRotX, 0, 5, 1, 1);

        RotAngle = new QDoubleSpinBox(simWidgetUi);
        RotAngle->setObjectName(QString::fromUtf8("RotAngle"));
        RotAngle->setDecimals(6);
        RotAngle->setMinimum(-99.000000000000000);
        RotAngle->setSingleStep(0.001000000000000);
        RotAngle->setStepType(QAbstractSpinBox::AdaptiveDecimalStepType);

        gridLayout_2->addWidget(RotAngle, 4, 3, 1, 1);


        verticalLayout_3->addLayout(gridLayout_2);

        setEntityStateButton = new QPushButton(simWidgetUi);
        setEntityStateButton->setObjectName(QString::fromUtf8("setEntityStateButton"));

        verticalLayout_3->addWidget(setEntityStateButton);


        verticalLayout->addLayout(verticalLayout_3);


        verticalLayout_4->addLayout(verticalLayout);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        label_2 = new QLabel(simWidgetUi);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        verticalLayout_2->addWidget(label_2);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        PushButtonRefresh = new QPushButton(simWidgetUi);
        PushButtonRefresh->setObjectName(QString::fromUtf8("PushButtonRefresh"));

        horizontalLayout_2->addWidget(PushButtonRefresh);

        ComboSpawables = new QComboBox(simWidgetUi);
        ComboSpawables->setObjectName(QString::fromUtf8("ComboSpawables"));

        horizontalLayout_2->addWidget(ComboSpawables);


        verticalLayout_2->addLayout(horizontalLayout_2);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        label_3 = new QLabel(simWidgetUi);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        horizontalLayout_5->addWidget(label_3);

        doubleSpinBoxX = new QDoubleSpinBox(simWidgetUi);
        doubleSpinBoxX->setObjectName(QString::fromUtf8("doubleSpinBoxX"));
        doubleSpinBoxX->setMinimum(-999999.000000000000000);
        doubleSpinBoxX->setMaximum(999999999999999.000000000000000);

        horizontalLayout_5->addWidget(doubleSpinBoxX);

        label_4 = new QLabel(simWidgetUi);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        horizontalLayout_5->addWidget(label_4);

        doubleSpinBoxY = new QDoubleSpinBox(simWidgetUi);
        doubleSpinBoxY->setObjectName(QString::fromUtf8("doubleSpinBoxY"));
        doubleSpinBoxY->setMinimum(-999999.000000000000000);
        doubleSpinBoxY->setMaximum(999999999999999.000000000000000);

        horizontalLayout_5->addWidget(doubleSpinBoxY);

        label_5 = new QLabel(simWidgetUi);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        horizontalLayout_5->addWidget(label_5);

        doubleSpinBoxZ = new QDoubleSpinBox(simWidgetUi);
        doubleSpinBoxZ->setObjectName(QString::fromUtf8("doubleSpinBoxZ"));
        doubleSpinBoxZ->setMinimum(-999999.000000000000000);
        doubleSpinBoxZ->setMaximum(999999999999999.000000000000000);

        horizontalLayout_5->addWidget(doubleSpinBoxZ);


        verticalLayout_2->addLayout(horizontalLayout_5);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        Name = new QLabel(simWidgetUi);
        Name->setObjectName(QString::fromUtf8("Name"));

        horizontalLayout->addWidget(Name);

        lineEditName = new QLineEdit(simWidgetUi);
        lineEditName->setObjectName(QString::fromUtf8("lineEditName"));

        horizontalLayout->addWidget(lineEditName);


        verticalLayout_2->addLayout(horizontalLayout);

        SpawnButton = new QPushButton(simWidgetUi);
        SpawnButton->setObjectName(QString::fromUtf8("SpawnButton"));

        verticalLayout_2->addWidget(SpawnButton);


        verticalLayout_4->addLayout(verticalLayout_2);


        retranslateUi(simWidgetUi);

        QMetaObject::connectSlotsByName(simWidgetUi);
    } // setupUi

    void retranslateUi(QWidget *simWidgetUi)
    {
        label_16->setText(QCoreApplication::translate("simWidgetUi", "Simulator State", nullptr));
        GetSimCapabilites->setText(QCoreApplication::translate("simWidgetUi", "GetSimCapabilities", nullptr));
        resetSimButton->setText(QCoreApplication::translate("simWidgetUi", "ResetSim", nullptr));
        label_17->setText(QCoreApplication::translate("simWidgetUi", "ResetScope", nullptr));
        label->setText(QCoreApplication::translate("simWidgetUi", "Entities State", nullptr));
        getAllEntitiesButton->setText(QCoreApplication::translate("simWidgetUi", "GetAllEntities", nullptr));
        despawnButton->setText(QCoreApplication::translate("simWidgetUi", "Despawn", nullptr));
        despawnAll->setText(QCoreApplication::translate("simWidgetUi", "DespawnAll", nullptr));
        getEntityStateButton->setText(QCoreApplication::translate("simWidgetUi", "GetEntityState", nullptr));
        label_11->setText(QCoreApplication::translate("simWidgetUi", "VelZ:", nullptr));
        label_7->setText(QCoreApplication::translate("simWidgetUi", "PosY:", nullptr));
        label_13->setText(QCoreApplication::translate("simWidgetUi", "VelRotY", nullptr));
        label_6->setText(QCoreApplication::translate("simWidgetUi", "PosX:", nullptr));
        label_15->setText(QCoreApplication::translate("simWidgetUi", "RotAngle", nullptr));
        label_9->setText(QCoreApplication::translate("simWidgetUi", "VelX:", nullptr));
        RotVectorX->setText(QCoreApplication::translate("simWidgetUi", "RotVector", nullptr));
        label_8->setText(QCoreApplication::translate("simWidgetUi", "PosZ:", nullptr));
        label_14->setText(QCoreApplication::translate("simWidgetUi", "VelRotZ", nullptr));
        label_12->setText(QCoreApplication::translate("simWidgetUi", "VelRotX", nullptr));
        label_10->setText(QCoreApplication::translate("simWidgetUi", "VelY:", nullptr));
        setEntityStateButton->setText(QCoreApplication::translate("simWidgetUi", "SetEntityState", nullptr));
        label_2->setText(QCoreApplication::translate("simWidgetUi", "Spawning", nullptr));
        PushButtonRefresh->setText(QCoreApplication::translate("simWidgetUi", "Refresh", nullptr));
        label_3->setText(QCoreApplication::translate("simWidgetUi", "X:", nullptr));
        label_4->setText(QCoreApplication::translate("simWidgetUi", "Y:", nullptr));
        label_5->setText(QCoreApplication::translate("simWidgetUi", "Z:", nullptr));
        Name->setText(QCoreApplication::translate("simWidgetUi", "Name", nullptr));
        SpawnButton->setText(QCoreApplication::translate("simWidgetUi", "Spawn!", nullptr));
        (void)simWidgetUi;
    } // retranslateUi

};

namespace Ui {
    class simWidgetUi: public Ui_simWidgetUi {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SIM_WIDGET_H
